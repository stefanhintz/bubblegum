import math
import re

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom


class OgnAgvWaypointDriver:
    class _State:
        def __init__(self):
            self._timeline_sub = None
            self.reset()
            self._subscribe_timeline_stop()

        def reset(self):
            self.idx = 0
            self.direction = 1
            self.lin_speed = 0.0
            self.yaw_rate = 0.0
            self.waiting = False
            self.wait_remaining_s = 0.0
            self.pending_endpoint_action = None
            self.bend_state = None
            self.route_signature = None
            self.stopped = False
            self.last_active_path = None

        def _subscribe_timeline_stop(self):
            timeline = omni.timeline.get_timeline_interface()
            if timeline is None:
                return

            self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline_event
            )

        def _on_timeline_event(self, event):
            if event.type == int(omni.timeline.TimelineEventType.STOP):
                self.reset()

    @staticmethod
    def internal_state():
        return OgnAgvWaypointDriver._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        OgnAgvWaypointDriver._set_default_outputs(db)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            db.log_error("No USD stage is available.")
            return False

        agv_paths = OgnAgvWaypointDriver._extract_target_paths(db.inputs.agvPrim)
        path_root_paths = OgnAgvWaypointDriver._extract_target_paths(db.inputs.pathRoots)
        agv_path = agv_paths[0] if agv_paths else ""
        path_root_path = OgnAgvWaypointDriver._select_active_path(path_root_paths, int(db.inputs.activePathIndex))

        if not agv_path:
            db.log_error("inputs:agvPrim is required.")
            return False
        if not path_root_path:
            db.log_error("inputs:pathRoots must contain at least one valid path root.")
            return False

        agv_prim = stage.GetPrimAtPath(agv_path)
        if not agv_prim.IsValid():
            db.log_error(f"AGV prim does not exist: {agv_path}")
            return False

        agv_xform = UsdGeom.Xformable(agv_prim)
        if not agv_xform:
            db.log_error(f"AGV prim is not Xformable: {agv_path}")
            return False

        if bool(db.inputs.reset):
            state.reset()

        waypoints = OgnAgvWaypointDriver._read_waypoints(stage, path_root_path)
        db.outputs.waypointCount = len(waypoints)
        db.outputs.isRouteValid = len(waypoints) >= 2
        db.outputs.activePathName = stage.GetPrimAtPath(path_root_path).GetName() if path_root_path else ""

        if len(waypoints) < 2:
            state.reset()
            return True

        reverse_mode = bool(waypoints[0]["reverse"] and waypoints[-1]["reverse"])
        db.outputs.reverseMode = reverse_mode

        route_signature = (path_root_path, tuple(wp["name"] for wp in waypoints))
        if state.route_signature != route_signature or state.last_active_path != path_root_path:
            state.reset()
            state.route_signature = route_signature
            state.last_active_path = path_root_path

        timeline = omni.timeline.get_timeline_interface()
        if timeline is not None and not timeline.is_playing():
            return True

        db.outputs.isRunning = bool(db.inputs.run) and not state.stopped
        if not bool(db.inputs.run):
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        dt = max(0.0, float(db.inputs.deltaTime))
        if dt <= 0.0:
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        state.idx = min(max(state.idx, 0), len(waypoints) - 1)
        if state.stopped:
            db.outputs.isStopped = True
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        pos, quat = OgnAgvWaypointDriver._get_world_pose(agv_prim)
        yaw = OgnAgvWaypointDriver._yaw_from_quat(quat)

        if state.waiting:
            state.lin_speed = OgnAgvWaypointDriver._move_towards(
                state.lin_speed, 0.0, float(db.inputs.maxAccelMps2) * dt
            )
            state.yaw_rate = OgnAgvWaypointDriver._move_towards(
                state.yaw_rate, 0.0, float(db.inputs.maxYawAccelRps2) * dt
            )

            yaw_next = yaw + state.yaw_rate * dt
            pos_next = pos.copy()
            pos_next[0] += math.cos(yaw_next) * state.lin_speed * dt
            pos_next[1] += math.sin(yaw_next) * state.lin_speed * dt
            pos_next[2] = pos[2]
            OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, pos_next, yaw_next)

            state.wait_remaining_s -= dt
            db.outputs.isWaiting = True
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            if state.wait_remaining_s > 0.0:
                return True

            state.waiting = False
            action = state.pending_endpoint_action
            state.pending_endpoint_action = None
            if action == "bend":
                if state.bend_state is not None:
                    state.bend_state["pending"] = False
                return True
            if action == "stop":
                state.stopped = True
                db.outputs.isStopped = True
                return True
            if action == "reverse":
                state.direction *= -1
                state.idx += state.direction
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True
            state.idx += state.direction
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        target = waypoints[state.idx]["pos"]
        bend_active = False
        bend = None
        if state.bend_state is None:
            bend_radius = float(waypoints[state.idx]["bend_radius"])
            if bend_radius > 0.0:
                in_idx = state.idx - state.direction
                out_idx = state.idx + state.direction
                if 0 <= in_idx < len(waypoints) and 0 <= out_idx < len(waypoints):
                    bend = OgnAgvWaypointDriver._compute_bend(
                        waypoints[in_idx]["pos"],
                        waypoints[state.idx]["pos"],
                        waypoints[out_idx]["pos"],
                        bend_radius,
                    )
                    if bend:
                        corner_xy = waypoints[state.idx]["pos"][:2]
                        s = float(np.dot(corner_xy - pos[:2], bend["v1"]))
                        if 0.0 <= s <= bend["d"]:
                            wait_ms = int(waypoints[state.idx]["wait_ms"])
                            state.bend_state = {
                                "idx": state.idx,
                                "t2": bend["t2"],
                                "center": bend["center"],
                                "left": bend["left"],
                                "v2": bend["v2"],
                                "pending": wait_ms > 0,
                            }
                            if wait_ms > 0:
                                state.waiting = True
                                state.wait_remaining_s = wait_ms / 1000.0
                                state.pending_endpoint_action = "bend"
                                db.outputs.isWaiting = True
                                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                                return True

        if state.bend_state is not None and not state.bend_state["pending"]:
            bend_active = True
            bend = state.bend_state
            target = np.array([bend["t2"][0], bend["t2"][1], waypoints[state.idx]["pos"][2]], dtype=float)

        delta = target - pos
        dist = float(np.linalg.norm(delta[:2]))

        if bend_active and float(np.dot(pos[:2] - bend["t2"], bend["v2"])) >= 0.0:
            state.bend_state = None
            state.idx += state.direction
            state.idx = min(max(state.idx, 0), len(waypoints) - 1)
            target = waypoints[state.idx]["pos"]
            delta = target - pos
            dist = float(np.linalg.norm(delta[:2]))
            bend_active = False
            bend = None

        if dist < float(db.inputs.positionToleranceM) and not bend_active:
            wait_ms = int(waypoints[state.idx]["wait_ms"])
            at_first = state.idx == 0
            at_last = state.idx == len(waypoints) - 1

            endpoint_action = None
            if at_last and state.direction == 1:
                endpoint_action = "reverse" if reverse_mode else "stop"
            elif at_first and state.direction == -1:
                endpoint_action = "reverse" if reverse_mode else "stop"

            if wait_ms > 0:
                state.waiting = True
                state.wait_remaining_s = wait_ms / 1000.0
                state.pending_endpoint_action = endpoint_action
                db.outputs.isWaiting = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            if endpoint_action == "stop":
                state.stopped = True
                db.outputs.isStopped = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True
            if endpoint_action == "reverse":
                state.direction *= -1
                state.idx += state.direction
            else:
                state.idx += state.direction

            state.idx = min(max(state.idx, 0), len(waypoints) - 1)
            if endpoint_action is not None:
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            target = waypoints[state.idx]["pos"]
            delta = target - pos
            dist = float(np.linalg.norm(delta[:2]))

        if dist > 1e-6:
            dir2 = delta[:2] / dist
            aim = pos.copy()
            aim[:2] = pos[:2] + dir2 * min(float(db.inputs.lookaheadM), dist)
        else:
            aim = target

        yaw_tol = math.radians(float(db.inputs.yawToleranceDeg))
        if bend_active:
            r = pos[:2] - bend["center"]
            if float(np.linalg.norm(r)) < 1e-6:
                tangent = bend["v1"]
            elif bend["left"]:
                tangent = np.array([-r[1], r[0]])
            else:
                tangent = np.array([r[1], -r[0]])
            desired_yaw = math.atan2(tangent[1], tangent[0])
        else:
            desired_yaw = math.atan2(aim[1] - pos[1], aim[0] - pos[0])

        yaw_err = OgnAgvWaypointDriver._wrap_pi(desired_yaw - yaw)
        yaw_rate_cmd = max(
            -float(db.inputs.maxYawRateRps),
            min(float(db.inputs.maxYawRateRps), 3.0 * yaw_err),
        )
        state.yaw_rate = OgnAgvWaypointDriver._move_towards(
            state.yaw_rate, yaw_rate_cmd, float(db.inputs.maxYawAccelRps2) * dt
        )

        if bend_active:
            v_cmd = float(db.inputs.targetSpeedMps)
        elif abs(yaw_err) > yaw_tol:
            v_cmd = 0.0
        else:
            v_brake = math.sqrt(max(0.0, 2.0 * float(db.inputs.maxAccelMps2) * dist))
            v_cmd = min(float(db.inputs.targetSpeedMps), v_brake)

        state.lin_speed = OgnAgvWaypointDriver._move_towards(
            state.lin_speed, v_cmd, float(db.inputs.maxAccelMps2) * dt
        )

        yaw_next = yaw + state.yaw_rate * dt
        pos_next = pos.copy()
        pos_next[0] += math.cos(yaw_next) * state.lin_speed * dt
        pos_next[1] += math.sin(yaw_next) * state.lin_speed * dt
        pos_next[2] = pos[2]
        OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, pos_next, yaw_next)

        OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
        return True

    @staticmethod
    def _set_default_outputs(db):
        db.outputs.isRouteValid = False
        db.outputs.isWaiting = False
        db.outputs.isStopped = False
        db.outputs.isRunning = False
        db.outputs.reverseMode = False
        db.outputs.activePathName = ""
        db.outputs.currentWaypointIndex = 0
        db.outputs.currentWaypointName = ""
        db.outputs.waypointCount = 0

    @staticmethod
    def _set_waypoint_outputs(db, state, waypoints):
        db.outputs.isWaiting = state.waiting
        db.outputs.isStopped = state.stopped
        if not waypoints:
            db.outputs.currentWaypointIndex = 0
            db.outputs.currentWaypointName = ""
            return

        idx = min(max(state.idx, 0), len(waypoints) - 1)
        db.outputs.currentWaypointIndex = idx
        db.outputs.currentWaypointName = waypoints[idx]["name"]

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
    def _select_active_path(path_root_paths, active_path_index):
        if not path_root_paths:
            return ""

        clamped_index = min(max(int(active_path_index), 0), len(path_root_paths) - 1)
        return path_root_paths[clamped_index]

    @staticmethod
    def _wrap_pi(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _move_towards(current, target, max_delta):
        if abs(target - current) <= max_delta:
            return target
        return current + math.copysign(max_delta, target - current)

    @staticmethod
    def _quat_from_yaw(yaw):
        half = yaw * 0.5
        return Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))

    @staticmethod
    def _yaw_from_quat(quat):
        w = float(quat.GetReal())
        imag = quat.GetImaginary()
        x, y, z = float(imag[0]), float(imag[1]), float(imag[2])
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _get_world_pose(prim):
        xf = omni.usd.get_world_transform_matrix(prim)
        translation = xf.ExtractTranslation()
        quat = xf.ExtractRotationQuat()
        return np.array([translation[0], translation[1], translation[2]], dtype=float), quat

    @staticmethod
    def _compute_bend(prev_pos, corner_pos, next_pos, radius):
        v1 = corner_pos[:2] - prev_pos[:2]
        v2 = next_pos[:2] - corner_pos[:2]
        len1 = float(np.linalg.norm(v1))
        len2 = float(np.linalg.norm(v2))
        if len1 < 1e-6 or len2 < 1e-6:
            return None

        v1 = v1 / len1
        v2 = v2 / len2
        dot = max(-1.0, min(1.0, float(np.dot(v1, v2))))
        angle = math.acos(dot)
        if angle < math.radians(1.0):
            return None

        d = radius * math.tan(angle / 2.0)
        if d >= len1 or d >= len2:
            return None

        corner_xy = corner_pos[:2]
        t1 = corner_xy - v1 * d
        t2 = corner_xy + v2 * d
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        normal = np.array([-v1[1], v1[0]]) if cross > 0.0 else np.array([v1[1], -v1[0]])
        center = t1 + normal * radius
        return {"t2": t2, "center": center, "left": cross > 0.0, "v1": v1, "v2": v2, "d": d}

    @staticmethod
    def _set_local_pose_xformable(xformable, pos_xyz, yaw):
        pos = Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]))
        quat = OgnAgvWaypointDriver._quat_from_yaw(float(yaw))

        ops = xformable.GetOrderedXformOps()
        translate_op = None
        orient_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op

        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp()

        translate_op.Set(pos)
        orient_op.Set(quat)

    @staticmethod
    def _read_waypoints(stage, path_root_path):
        root = stage.GetPrimAtPath(path_root_path)
        if not root.IsValid():
            return []

        children = [child for child in root.GetChildren() if child.IsA(UsdGeom.Xform)]
        children.sort(key=lambda prim: prim.GetName())

        items = []
        for child in children:
            pos, _quat = OgnAgvWaypointDriver._get_world_pose(child)
            name = child.GetName()
            wait_match = re.search(r"_waitMS(\d+)", name)
            bend_match = re.search(r"_bendingCM(\d+)", name)
            items.append(
                {
                    "name": name,
                    "pos": pos,
                    "reverse": "_reverse" in name,
                    "wait_ms": int(wait_match.group(1)) if wait_match else 0,
                    "bend_radius": (float(bend_match.group(1)) / 100.0) if bend_match else 0.0,
                }
            )
        return items
