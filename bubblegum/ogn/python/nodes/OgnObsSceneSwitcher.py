import base64
import hashlib
import json
import uuid

import omni.graph.core as og

try:
    import websocket
except Exception:
    websocket = None

# Design note:
# - Constraints: one cached OBS websocket connection, one scene switch per distinct scene name, no scene polling.
# - Trade-offs: simple request/response flow over a more robust async client or reconnect manager.
# - Rejected alternative: a generic OBS command node, because this node is intentionally scoped to scene switching.


WS_URL = "ws://127.0.0.1:4455"
WS_PASSWORD = "your_obs_password"
DEFAULT_SCENE = "Scene 1"


class OgnObsSceneSwitcher:
    class _State:
        def __init__(self):
            self.ws = None
            self.last_scene = None

    @staticmethod
    def internal_state():
        return OgnObsSceneSwitcher._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        db.outputs.execDone = og.ExecutionAttributeState.DISABLED

        scene_name = OgnObsSceneSwitcher._string_input(db.inputs.sceneName, DEFAULT_SCENE)
        ws_url = OgnObsSceneSwitcher._string_input(db.inputs.wsUrl, WS_URL)
        ws_password = OgnObsSceneSwitcher._string_input(db.inputs.wsPassword, WS_PASSWORD)

        if not scene_name:
            scene_name = DEFAULT_SCENE
        if not ws_url:
            ws_url = WS_URL
        if ws_password is None:
            ws_password = WS_PASSWORD

        if state.ws is not None and state.last_scene == scene_name:
            return True

        try:
            if state.ws is None:
                state.ws = OgnObsSceneSwitcher._connect_obs(ws_url, ws_password)

            OgnObsSceneSwitcher._switch_scene(state.ws, scene_name)
            state.last_scene = scene_name
            db.outputs.execDone = og.ExecutionAttributeState.ENABLED
            return True
        except Exception as exc:
            db.log_error(f"OBS scene switch failed: {exc}")
            OgnObsSceneSwitcher._close_ws(state)
            state.last_scene = None
            return False

    @staticmethod
    def cleanup(db):
        OgnObsSceneSwitcher._close_ws(db.per_instance_state)

    @staticmethod
    def _string_input(value, default):
        if value is None:
            return default
        value = str(value).strip()
        return value if value else default

    @staticmethod
    def _make_auth(password, salt, challenge):
        secret = base64.b64encode(
            hashlib.sha256((password + salt).encode("utf-8")).digest()
        ).decode("utf-8")
        return base64.b64encode(
            hashlib.sha256((secret + challenge).encode("utf-8")).digest()
        ).decode("utf-8")

    @staticmethod
    def _connect_obs(ws_url, ws_password):
        if websocket is None:
            raise RuntimeError("Python package 'websocket-client' is not available")

        ws = websocket.create_connection(
            ws_url,
            subprotocols=["obswebsocket.json"],
            timeout=2,
        )

        hello = json.loads(ws.recv())
        if hello.get("op") != 0:
            raise RuntimeError(f"Unexpected OBS hello: {hello}")

        identify = {
            "op": 1,
            "d": {
                "rpcVersion": 1,
            },
        }

        auth_data = hello.get("d", {}).get("authentication")
        if auth_data:
            identify["d"]["authentication"] = OgnObsSceneSwitcher._make_auth(
                ws_password,
                auth_data["salt"],
                auth_data["challenge"],
            )

        ws.send(json.dumps(identify))
        identified = json.loads(ws.recv())
        if identified.get("op") != 2:
            raise RuntimeError(f"OBS identify failed: {identified}")

        return ws

    @staticmethod
    def _switch_scene(ws, scene_name):
        payload = {
            "op": 6,
            "d": {
                "requestType": "SetCurrentProgramScene",
                "requestId": str(uuid.uuid4()),
                "requestData": {
                    "sceneName": scene_name,
                },
            },
        }

        ws.send(json.dumps(payload))
        response = json.loads(ws.recv())
        if response.get("op") != 7:
            raise RuntimeError(f"Unexpected OBS response: {response}")

        status = response.get("d", {}).get("requestStatus", {})
        if not status.get("result", False):
            raise RuntimeError(f"OBS scene switch failed: {response}")

    @staticmethod
    def _close_ws(state):
        try:
            if getattr(state, "ws", None) is not None:
                state.ws.close()
        except Exception:
            pass
        state.ws = None
