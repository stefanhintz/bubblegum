import json
import math
import threading
import time
from http.client import HTTPException
from json import JSONDecodeError
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import omni.usd
from pxr import Gf, UsdGeom


class OgnDiceStoreSync:
    # Design note:
    # - Constraints: syncs exactly five dice objects and assumes the API shape shown by the user.
    # - Trade-offs: background polling plus cached latest data keeps compute non-blocking at the cost of eventual consistency.
    # - Rejected alternative: reuse the generic store-poller outputs as an intermediate node, because this node needs direct stage writes and that split adds no value.

    NUM_DICES = 5
    ROT_EPS = 1e-8
    IDENTITY_QUAT = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
    TOP_FACE_AXIS_LOCAL = Gf.Vec3f(0.0, 0.0, -1.0)
    FACE_UP_QUATS = {
        "P": Gf.Quatf(0.0, Gf.Vec3f(0.0, 1.0, 0.0)),
        "I": Gf.Quatf(0.70710678, Gf.Vec3f(-0.70710678, 0.0, 0.0)),
        "A": IDENTITY_QUAT,
        "C": Gf.Quatf(0.70710678, Gf.Vec3f(0.70710678, 0.0, 0.0)),
        "D": Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0)),
        "T": Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0)),
    }
    FACE_SPIN_DEGREES = {
        "P": 0.0,
        "I": 0.0,
        "A": 180.0,
        "C": 0.0,
        "D": 90.0,
        "T": 270.0,
    }

    class _State:
        def __init__(self):
            self.lock = threading.Lock()
            self.worker = None
            self.base_url = ""
            self.next_poll_time = 0.0
            self.is_connected = False
            self.last_error = ""
            self.cached_poses = [None] * OgnDiceStoreSync.NUM_DICES
            self.cached_letters = [None] * OgnDiceStoreSync.NUM_DICES
            self.last_positions = [None] * OgnDiceStoreSync.NUM_DICES
            self.last_letters = [None] * OgnDiceStoreSync.NUM_DICES

    @staticmethod
    def internal_state():
        return OgnDiceStoreSync._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        db.outputs.isConnected = state.is_connected
        db.outputs.lastError = state.last_error

        enabled = bool(db.inputs.enabled)
        base_url = OgnDiceStoreSync._string_input(db.inputs.baseUrl)
        poll_interval = max(0.05, float(db.inputs.pollIntervalS))
        request_timeout = max(0.01, float(db.inputs.requestTimeoutS))
        position_scale = float(db.inputs.positionScale)
        dice_half_height = float(db.inputs.diceHalfHeight)
        enable_letter_orientation = bool(db.inputs.enableLetterOrientation)

        if enabled and base_url:
            OgnDiceStoreSync._start_poll_if_due(state, base_url, poll_interval, request_timeout)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            db.log_error("No USD stage is available.")
            return False

        dice_prim_paths = OgnDiceStoreSync._extract_target_paths(db.inputs.dicePrimPaths)
        if len(dice_prim_paths) < OgnDiceStoreSync.NUM_DICES:
            db.log_error("inputs:dicePrimPaths must contain at least five dice prims.")
            return False

        for index in range(OgnDiceStoreSync.NUM_DICES):
            with state.lock:
                pose = None if state.cached_poses[index] is None else list(state.cached_poses[index])
                letter = state.cached_letters[index]

            if pose is None:
                continue

            if not OgnDiceStoreSync._poses_different(pose, state.last_positions[index]) and letter == state.last_letters[index]:
                continue

            try:
                OgnDiceStoreSync._set_prim_pose(
                    stage,
                    dice_prim_paths[index],
                    pose,
                    letter,
                    position_scale,
                    dice_half_height,
                    enable_letter_orientation,
                )
            except ValueError as exc:
                state.last_error = str(exc)
                db.log_warning(f"Dice {index}: {exc}")
                continue

            state.last_positions[index] = list(pose)
            state.last_letters[index] = letter

        db.outputs.isConnected = state.is_connected
        db.outputs.lastError = state.last_error
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
    def _extract_target_paths(target_value):
        if target_value is None:
            return []

        paths = getattr(target_value, "paths", None)
        if paths is not None:
            try:
                return [str(path).strip() for path in paths if str(path).strip()]
            except TypeError:
                pass

        if isinstance(target_value, (list, tuple)):
            return [str(path).strip() for path in target_value if str(path).strip()]

        value = str(target_value).strip()
        if not value or value in {"[]", "None"}:
            return []
        return [value]

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
                target=OgnDiceStoreSync._poll_once,
                args=(state, base_url, request_timeout),
                daemon=True,
            )
            state.worker = worker
            worker.start()

    @staticmethod
    def _poll_once(state, base_url, request_timeout):
        try:
            poses, letters = OgnDiceStoreSync._fetch_snapshot(base_url, request_timeout)
        except (HTTPError, HTTPException, JSONDecodeError, OSError, URLError, ValueError) as exc:
            with state.lock:
                state.is_connected = False
                state.last_error = str(exc)
                state.worker = None
            return

        with state.lock:
            state.cached_poses = poses
            state.cached_letters = letters
            state.is_connected = True
            state.last_error = ""
            state.worker = None

    @staticmethod
    def _fetch_snapshot(base_url, request_timeout):
        poses = [None] * OgnDiceStoreSync.NUM_DICES
        letters = [None] * OgnDiceStoreSync.NUM_DICES

        for index in range(OgnDiceStoreSync.NUM_DICES):
            pose, letter = OgnDiceStoreSync._fetch_die(base_url, index, request_timeout)
            poses[index] = pose
            letters[index] = letter

        return poses, letters

    @staticmethod
    def _fetch_die(base_url, index, request_timeout):
        url = f"{base_url.rstrip('/')}/{index}"
        request = Request(url, method="GET")

        try:
            with urlopen(request, timeout=request_timeout) as response:
                payload = response.read().decode("utf-8")
        except HTTPError as exc:
            if exc.code == 404:
                return None, None
            raise

        data = json.loads(payload)
        pose = data.get("position")
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError(f"Invalid pose for dice {index}: {data}")
        return [float(item) for item in pose[:6]], data.get("letter")

    @staticmethod
    def _rotation_vector_to_quat(rx, ry, rz):
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < OgnDiceStoreSync.ROT_EPS:
            return OgnDiceStoreSync.IDENTITY_QUAT

        ax = rx / angle
        ay = ry / angle
        az = rz / angle
        half = angle * 0.5
        s = math.sin(half)
        return Gf.Quatf(float(math.cos(half)), Gf.Vec3f(float(ax * s), float(ay * s), float(az * s)))

    @staticmethod
    def _axis_angle_quat(axis, degrees):
        if not degrees:
            return OgnDiceStoreSync.IDENTITY_QUAT

        axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
        if axis_len < OgnDiceStoreSync.ROT_EPS:
            return OgnDiceStoreSync.IDENTITY_QUAT

        half = math.radians(float(degrees)) * 0.5
        s = math.sin(half) / axis_len
        return Gf.Quatf(
            float(math.cos(half)),
            Gf.Vec3f(float(axis[0] * s), float(axis[1] * s), float(axis[2] * s)),
        )

    @staticmethod
    def _letter_quat(letter):
        if not letter:
            return OgnDiceStoreSync.IDENTITY_QUAT

        key = str(letter).strip().upper()
        face_up = OgnDiceStoreSync.FACE_UP_QUATS.get(key, OgnDiceStoreSync.IDENTITY_QUAT)
        top_axis = face_up.Transform(OgnDiceStoreSync.TOP_FACE_AXIS_LOCAL)
        face_spin = OgnDiceStoreSync._axis_angle_quat(top_axis, OgnDiceStoreSync.FACE_SPIN_DEGREES.get(key, 0.0))
        return face_up * face_spin

    @staticmethod
    def _ensure_xform_ops(prim):
        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()

        translate_op = None
        orient_op = None
        for op in ops:
            op_type = op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeTranslate and translate_op is None:
                translate_op = op
            elif op_type == UsdGeom.XformOp.TypeOrient and orient_op is None:
                orient_op = op

        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp()

        xformable.SetXformOpOrder([translate_op, orient_op])
        return translate_op, orient_op

    @staticmethod
    def _set_prim_pose(stage, prim_path, pose, letter, position_scale, dice_half_height, enable_letter_orientation):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")

        tx, ty, tz, rx, ry, rz = pose
        top_world = Gf.Vec3d(float(tx) * position_scale, float(ty) * position_scale, float(tz) * position_scale)

        base_quat = OgnDiceStoreSync._rotation_vector_to_quat(rx, ry, rz)
        quat = base_quat
        if enable_letter_orientation and letter:
            quat = quat * OgnDiceStoreSync._letter_quat(letter)

        top_offset_local = Gf.Vec3f(0.0, 0.0, float(dice_half_height))
        rotated_top_offset_f = base_quat.Transform(top_offset_local)
        rotated_top_offset = Gf.Vec3d(
            float(rotated_top_offset_f[0]),
            float(rotated_top_offset_f[1]),
            float(rotated_top_offset_f[2]),
        )
        center_world = top_world - rotated_top_offset

        translate_op, orient_op = OgnDiceStoreSync._ensure_xform_ops(prim)
        translate_op.Set(center_world)
        orient_op.Set(quat)

    @staticmethod
    def _poses_different(a, b, eps=1e-6):
        if a is None or b is None:
            return True
        return any(abs(float(x) - float(y)) > eps for x, y in zip(a, b))
