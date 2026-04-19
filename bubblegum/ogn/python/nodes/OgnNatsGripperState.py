import asyncio
import json
import os
import sys
import threading


class OgnNatsGripperState:
    # Design note:
    # - Constraints: subscribes to exactly two controller I/O subjects and extracts one boolean signal from each.
    # - Trade-offs: a dedicated subscriber thread keeps compute non-blocking at the cost of eventually consistent outputs.
    # - Rejected alternative: a generic NATS subject node, because for this use case the extra abstraction only makes setup harder.

    class _State:
        def __init__(self):
            self.lock = threading.Lock()
            self.thread = None
            self.loop = None
            self.stop_event = None
            self.config = None
            self.is_connected = False
            self.last_error = ""
            self.gripper_kuka_gripped = False
            self.gripper_yaskawa_gripped = False

    @staticmethod
    def internal_state():
        return OgnNatsGripperState._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        enabled = bool(db.inputs.enabled)
        config = {
            "nats_url": OgnNatsGripperState._string_input(db.inputs.natsUrl),
            "token": OgnNatsGripperState._string_input(db.inputs.token),
            "cell_name": OgnNatsGripperState._string_input(db.inputs.cellName) or "cell",
            "kuka_controller": OgnNatsGripperState._string_input(db.inputs.kukaController),
            "kuka_io_name": OgnNatsGripperState._string_input(db.inputs.kukaIoName),
            "yaskawa_controller": OgnNatsGripperState._string_input(db.inputs.yaskawaController),
            "yaskawa_io_name": OgnNatsGripperState._string_input(db.inputs.yaskawaIoName),
        }

        if enabled:
            if not config["nats_url"]:
                db.log_error("inputs:natsUrl is required.")
                return False
            if not config["kuka_controller"] or not config["kuka_io_name"]:
                db.log_error("inputs:kukaController and inputs:kukaIoName are required.")
                return False
            if not config["yaskawa_controller"] or not config["yaskawa_io_name"]:
                db.log_error("inputs:yaskawaController and inputs:yaskawaIoName are required.")
                return False
            OgnNatsGripperState._ensure_running(state, config)
        else:
            OgnNatsGripperState._stop(state)

        with state.lock:
            db.outputs.isConnected = state.is_connected
            db.outputs.lastError = state.last_error
            db.outputs.gripperKukaGripped = state.gripper_kuka_gripped
            db.outputs.gripperYaskawaGripped = state.gripper_yaskawa_gripped

        return True

    @staticmethod
    def cleanup(db):
        OgnNatsGripperState._stop(db.per_instance_state)

    @staticmethod
    def _string_input(value):
        if value is None:
            return ""
        return str(value).strip()

    @staticmethod
    def _ensure_running(state, config):
        with state.lock:
            thread = state.thread
            running = thread is not None and thread.is_alive()
            if running and state.config == config:
                return

        OgnNatsGripperState._stop(state)
        OgnNatsGripperState._start(state, config)

    @staticmethod
    def _start(state, config):
        stop_event = threading.Event()
        thread = threading.Thread(
            target=OgnNatsGripperState._run_client,
            args=(state, config, stop_event),
            daemon=True,
        )
        with state.lock:
            state.config = dict(config)
            state.stop_event = stop_event
            state.thread = thread
            state.is_connected = False
            state.last_error = ""
        thread.start()

    @staticmethod
    def _stop(state):
        with state.lock:
            stop_event = state.stop_event
            loop = state.loop
            thread = state.thread
            state.stop_event = None
            state.loop = None
            state.thread = None
            state.config = None
            state.is_connected = False

        if stop_event is not None:
            stop_event.set()
        if loop is not None:
            try:
                loop.call_soon_threadsafe(lambda: None)
            except Exception:
                pass
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    @staticmethod
    def _run_client(state, config, stop_event):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with state.lock:
            state.loop = loop

        try:
            loop.run_until_complete(OgnNatsGripperState._run_client_async(state, config, stop_event))
        finally:
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            except Exception:
                pass
            loop.close()
            with state.lock:
                if state.loop is loop:
                    state.loop = None

    @staticmethod
    async def _run_client_async(state, config, stop_event):
        nats = OgnNatsGripperState._import_nats()
        if nats is None:
            with state.lock:
                state.last_error = "Python package 'nats-py' is not available"
            return

        subject_map = {
            f"nova.v2.cells.{config['cell_name']}.controllers.{config['kuka_controller']}.ios": ("kuka", config["kuka_io_name"]),
            f"nova.v2.cells.{config['cell_name']}.controllers.{config['yaskawa_controller']}.ios": ("yaskawa", config["yaskawa_io_name"]),
        }

        async def message_handler(msg):
            try:
                data = json.loads(msg.data.decode("utf-8"))
            except Exception as exc:
                with state.lock:
                    state.last_error = str(exc)
                return

            mapping = subject_map.get(msg.subject)
            if mapping is None:
                return

            target, io_name = mapping
            value = OgnNatsGripperState._extract_io_bool(data, io_name)
            if value is None:
                return

            with state.lock:
                if target == "kuka":
                    state.gripper_kuka_gripped = value
                else:
                    state.gripper_yaskawa_gripped = value

        nc = None
        try:
            connect_kwargs = {"servers": [config["nats_url"]]}
            if config["token"]:
                connect_kwargs["token"] = config["token"]
            nc = await nats.connect(**connect_kwargs)

            for subject in subject_map:
                await nc.subscribe(subject, cb=message_handler)

            with state.lock:
                state.is_connected = True
                state.last_error = ""

            while not stop_event.is_set():
                await asyncio.sleep(0.1)
        except Exception as exc:
            with state.lock:
                state.is_connected = False
                state.last_error = str(exc)
        finally:
            if nc is not None and nc.is_connected:
                await nc.close()
            with state.lock:
                state.is_connected = False

    @staticmethod
    def _import_nats():
        vendor_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "vendor"))
        if vendor_path not in sys.path:
            sys.path.insert(0, vendor_path)
        try:
            import nats
        except ImportError:
            return None
        return nats

    @staticmethod
    def _extract_io_bool(data, io_name):
        io_values = data.get("io_values")
        if not isinstance(io_values, list):
            return None

        for io_value in io_values:
            if not isinstance(io_value, dict):
                continue
            if str(io_value.get("io", "")) != io_name:
                continue

            value = io_value.get("value")
            value_type = str(io_value.get("value_type", "")).lower()
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                lowered = value.strip().lower()
                if lowered in {"true", "1"}:
                    return True
                if lowered in {"false", "0"}:
                    return False
            if value_type == "boolean" and isinstance(value, (int, float)):
                return bool(value)
            return None

        return None
