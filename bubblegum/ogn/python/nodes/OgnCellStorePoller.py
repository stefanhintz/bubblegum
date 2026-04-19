import json
import threading
import time
from http.client import HTTPException
from json import JSONDecodeError
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import urlopen


class OgnCellStorePoller:
    # Design note:
    # - Constraints: polls a fixed set of cell-store objects and is not a generic object browser.
    # - Trade-offs: background thread plus cached outputs keeps compute non-blocking at the cost of eventual consistency.
    # - Rejected alternative: synchronous urllib calls in compute, because that would stall graph evaluation on network jitter.

    OBJECT_PATHS = (
        "agv_state_machine",
        "dices/0",
        "dices/1",
        "dices/2",
        "dices/3",
        "dices/4",
        "gripper/kuka",
        "gripper/yaskawa",
    )

    class _State:
        def __init__(self):
            self.lock = threading.Lock()
            self.worker = None
            self.base_url = ""
            self.next_poll_time = 0.0
            self.is_connected = False
            self.last_error = ""
            self.snapshot = OgnCellStorePoller._empty_snapshot()

    @staticmethod
    def internal_state():
        return OgnCellStorePoller._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        enabled = bool(db.inputs.enabled)
        base_url = OgnCellStorePoller._string_input(db.inputs.baseUrl)
        poll_interval = max(0.05, float(db.inputs.pollIntervalS))
        request_timeout = max(0.01, float(db.inputs.requestTimeoutS))

        if enabled and base_url:
            OgnCellStorePoller._start_poll_if_due(state, base_url, poll_interval, request_timeout)

        OgnCellStorePoller._write_outputs(db, state)
        return True

    @staticmethod
    def cleanup(db):
        state = db.per_instance_state
        with state.lock:
            state.worker = None

    @staticmethod
    def _string_input(value):
        if value is None:
            return ""
        return str(value).strip()

    @staticmethod
    def _start_poll_if_due(state, base_url, poll_interval, request_timeout):
        now = time.monotonic()
        with state.lock:
            worker = state.worker
            worker_alive = worker is not None and worker.is_alive()
            if worker_alive:
                return
            if state.base_url == base_url and now < state.next_poll_time:
                return

            state.base_url = base_url
            state.next_poll_time = now + poll_interval
            worker = threading.Thread(
                target=OgnCellStorePoller._poll_once,
                args=(state, base_url, request_timeout),
                daemon=True,
            )
            state.worker = worker
            worker.start()

    @staticmethod
    def _poll_once(state, base_url, request_timeout):
        try:
            snapshot = OgnCellStorePoller._fetch_snapshot(base_url, request_timeout)
        except (HTTPError, HTTPException, JSONDecodeError, OSError, URLError, ValueError) as exc:
            with state.lock:
                state.is_connected = False
                state.last_error = str(exc)
                state.worker = None
            return

        with state.lock:
            state.snapshot = snapshot
            state.is_connected = True
            state.last_error = ""
            state.worker = None

    @staticmethod
    def _fetch_snapshot(base_url, request_timeout):
        raw_objects = {}
        for object_path in OgnCellStorePoller.OBJECT_PATHS:
            raw_objects[object_path] = OgnCellStorePoller._fetch_json(base_url, object_path, request_timeout)

        snapshot = OgnCellStorePoller._empty_snapshot()

        agv = raw_objects["agv_state_machine"]
        snapshot["agvStatus"] = str(agv.get("status", ""))
        snapshot["agvRunning"] = bool(agv.get("running", False))
        snapshot["agvLowBattery"] = bool(agv.get("low_battery", False))
        snapshot["agvChargeAfterDump"] = bool(agv.get("charge_after_dump", False))
        snapshot["agvError"] = str(agv.get("error", ""))

        for index in range(5):
            dice = raw_objects[f"dices/{index}"]
            snapshot[f"dice{index}Position"] = OgnCellStorePoller._coerce_pose(dice.get("position"))
            snapshot[f"dice{index}Letter"] = str(dice.get("letter", ""))

        kuka = raw_objects["gripper/kuka"]
        yaskawa = raw_objects["gripper/yaskawa"]
        snapshot["gripperKukaGripped"] = bool(kuka.get("gripped", False))
        snapshot["gripperYaskawaGripped"] = bool(yaskawa.get("gripped", False))
        return snapshot

    @staticmethod
    def _fetch_json(base_url, object_path, request_timeout):
        url = f"{base_url.rstrip('/')}/{quote(object_path, safe='/')}"
        with urlopen(url, timeout=request_timeout) as response:
            payload = response.read().decode("utf-8")
        data = json.loads(payload)
        if not isinstance(data, dict):
            raise ValueError(f"Expected JSON object from {url}")
        return data

    @staticmethod
    def _coerce_pose(value):
        if not isinstance(value, list):
            return [0.0] * 6

        pose = []
        for item in value[:6]:
            pose.append(float(item))
        while len(pose) < 6:
            pose.append(0.0)
        return pose

    @staticmethod
    def _write_outputs(db, state):
        with state.lock:
            snapshot = dict(state.snapshot)
            is_connected = state.is_connected
            last_error = state.last_error

        db.outputs.isConnected = is_connected
        db.outputs.lastError = last_error
        db.outputs.agvStatus = snapshot["agvStatus"]
        db.outputs.agvRunning = snapshot["agvRunning"]
        db.outputs.agvLowBattery = snapshot["agvLowBattery"]
        db.outputs.agvChargeAfterDump = snapshot["agvChargeAfterDump"]
        db.outputs.agvError = snapshot["agvError"]
        db.outputs.dice0Position = snapshot["dice0Position"]
        db.outputs.dice0Letter = snapshot["dice0Letter"]
        db.outputs.dice1Position = snapshot["dice1Position"]
        db.outputs.dice1Letter = snapshot["dice1Letter"]
        db.outputs.dice2Position = snapshot["dice2Position"]
        db.outputs.dice2Letter = snapshot["dice2Letter"]
        db.outputs.dice3Position = snapshot["dice3Position"]
        db.outputs.dice3Letter = snapshot["dice3Letter"]
        db.outputs.dice4Position = snapshot["dice4Position"]
        db.outputs.dice4Letter = snapshot["dice4Letter"]
        db.outputs.gripperKukaGripped = snapshot["gripperKukaGripped"]
        db.outputs.gripperYaskawaGripped = snapshot["gripperYaskawaGripped"]

    @staticmethod
    def _empty_snapshot():
        return {
            "agvStatus": "",
            "agvRunning": False,
            "agvLowBattery": False,
            "agvChargeAfterDump": False,
            "agvError": "",
            "dice0Position": [0.0] * 6,
            "dice0Letter": "",
            "dice1Position": [0.0] * 6,
            "dice1Letter": "",
            "dice2Position": [0.0] * 6,
            "dice2Letter": "",
            "dice3Position": [0.0] * 6,
            "dice3Letter": "",
            "dice4Position": [0.0] * 6,
            "dice4Letter": "",
            "gripperKukaGripped": False,
            "gripperYaskawaGripped": False,
        }
