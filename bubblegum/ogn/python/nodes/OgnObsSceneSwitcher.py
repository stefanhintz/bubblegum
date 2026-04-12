import base64
import hashlib
import json
import os
import socket
import ssl
import uuid
from urllib.parse import urlparse

import omni.graph.core as og

# Design note:
# - Constraints: one cached OBS websocket connection, one scene switch per distinct scene name, no scene polling.
# - Trade-offs: simple request/response flow over a more robust async client or reconnect manager.
# - Rejected alternative: a generic OBS command node, because this node is intentionally scoped to scene switching.


WS_URL = "ws://127.0.0.1:4455"
WS_PASSWORD = ""
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
        ws = OgnObsSceneSwitcher._WebSocketClient.connect(ws_url)

        hello = json.loads(ws.recv_text())
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
            if not ws_password:
                raise RuntimeError("OBS websocket password is required for this OBS instance")
            identify["d"]["authentication"] = OgnObsSceneSwitcher._make_auth(
                ws_password,
                auth_data["salt"],
                auth_data["challenge"],
            )

        ws.send_text(json.dumps(identify))
        identified = json.loads(ws.recv_text())
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

        ws.send_text(json.dumps(payload))
        response = json.loads(ws.recv_text())
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

    class _WebSocketClient:
        def __init__(self, sock):
            self._sock = sock

        @staticmethod
        def connect(ws_url):
            parsed = urlparse(ws_url)
            if parsed.scheme not in {"ws", "wss"}:
                raise RuntimeError(f"Unsupported OBS websocket URL: {ws_url}")

            host = parsed.hostname or "127.0.0.1"
            port = parsed.port or (443 if parsed.scheme == "wss" else 80)
            path = parsed.path or "/"
            if parsed.query:
                path = f"{path}?{parsed.query}"

            sock = socket.create_connection((host, port), timeout=2)
            if parsed.scheme == "wss":
                context = ssl.create_default_context()
                sock = context.wrap_socket(sock, server_hostname=host)

            client = OgnObsSceneSwitcher._WebSocketClient(sock)
            client._handshake(host, port, path)
            return client

        def send_text(self, text):
            payload = text.encode("utf-8")
            header = bytearray()
            header.append(0x81)
            header.extend(self._mask_length(len(payload)))
            mask_key = os.urandom(4)
            header.extend(mask_key)
            masked = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
            self._sock.sendall(header + masked)

        def recv_text(self):
            while True:
                fin, opcode, payload = self._read_frame()
                if opcode == 0x1:
                    return payload.decode("utf-8")
                if opcode == 0x8:
                    self.close()
                    raise RuntimeError("OBS websocket closed the connection")
                if opcode == 0x9:
                    self._send_control_frame(0xA, payload)
                    continue
                if opcode == 0x0 and fin:
                    continue
                raise RuntimeError(f"Unsupported OBS websocket frame opcode: {opcode}")

        def close(self):
            try:
                self._sock.close()
            except Exception:
                pass

        def _handshake(self, host, port, path):
            key = base64.b64encode(os.urandom(16)).decode("utf-8")
            request = (
                f"GET {path} HTTP/1.1\r\n"
                f"Host: {host}:{port}\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                "Sec-WebSocket-Version: 13\r\n"
                "Sec-WebSocket-Protocol: obswebsocket.json\r\n"
                "\r\n"
            )
            self._sock.sendall(request.encode("utf-8"))

            response = self._read_http_response()
            if " 101 " not in response.splitlines()[0]:
                raise RuntimeError(f"OBS websocket handshake failed: {response!r}")

        def _read_http_response(self):
            data = b""
            while b"\r\n\r\n" not in data:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                data += chunk
            return data.decode("utf-8", errors="replace")

        def _read_frame(self):
            first_two = self._read_exact(2)
            first_byte, second_byte = first_two[0], first_two[1]
            fin = bool(first_byte & 0x80)
            opcode = first_byte & 0x0F
            masked = bool(second_byte & 0x80)
            length = second_byte & 0x7F

            if length == 126:
                length = int.from_bytes(self._read_exact(2), "big")
            elif length == 127:
                length = int.from_bytes(self._read_exact(8), "big")

            mask_key = self._read_exact(4) if masked else None
            payload = self._read_exact(length) if length else b""
            if masked and mask_key is not None:
                payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
            return fin, opcode, payload

        def _send_control_frame(self, opcode, payload=b""):
            header = bytearray()
            header.append(0x80 | (opcode & 0x0F))
            header.extend(self._mask_length(len(payload)))
            mask_key = os.urandom(4)
            header.extend(mask_key)
            masked = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
            self._sock.sendall(header + masked)

        def _mask_length(self, length):
            if length <= 125:
                return bytes([0x80 | length])
            if length <= 0xFFFF:
                return bytes([0x80 | 126]) + length.to_bytes(2, "big")
            return bytes([0x80 | 127]) + length.to_bytes(8, "big")

        def _read_exact(self, length):
            data = b""
            while len(data) < length:
                chunk = self._sock.recv(length - len(data))
                if not chunk:
                    raise RuntimeError("Unexpected end of OBS websocket stream")
                data += chunk
            return data
