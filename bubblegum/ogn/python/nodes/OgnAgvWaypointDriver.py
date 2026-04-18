import math

import numpy as np
import omni.graph.core as og
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

# Design note:
# - Constraints: fixed waypoint Xforms under path roots; no live path editing or obstacle handling.
# - Trade-offs: simple kinematic integration and yaw control over a more physical or generic path follower.
# - Rejected alternative: a more configurable route/model layer, because the node is easier to use when it reads
#   waypoint names directly and keeps all state local.


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
            self.reset_on_play = False
            self.agv_path = ""
            self.path_root_path = ""
            self.returning_from_reverse = False
            self.dock_returning = False
            self.dock_exit_idx = 0

        def _subscribe_timeline_stop(self):
            timeline = omni.timeline.get_timeline_interface()
            if timeline is None:
                return

            self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline_event
            )

        def _on_timeline_event(self, event):
            if event.type == int(omni.timeline.TimelineEventType.PLAY):
                self.reset_on_play = True
            if event.type == int(omni.timeline.TimelineEventType.STOP):
                OgnAgvWaypointDriver._reset_tracked_agv(self)
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
        path_root_path = ""
        if path_root_paths:
            active_path_index = min(max(int(db.inputs.activePathIndex), 0), len(path_root_paths) - 1)
            path_root_path = path_root_paths[active_path_index]

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

        state.agv_path = agv_path
        state.path_root_path = path_root_path

        waypoints = OgnAgvWaypointDriver._read_waypoints(stage, path_root_path)
        db.outputs.waypointCount = len(waypoints)
        db.outputs.isRouteValid = len(waypoints) >= 2

        if len(waypoints) < 2:
            state.reset()
            return True

        if state.reset_on_play:
            state.reset()
            OgnAgvWaypointDriver._snap_agv_to_waypoint(agv_prim, agv_xform, waypoints, state.direction)
            state.route_signature = route_signature = (path_root_path, tuple(wp["name"] for wp in waypoints))
            state.reset_on_play = False
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        if db.inputs.execReset == og.ExecutionAttributeState.ENABLED:
            state.reset()
            OgnAgvWaypointDriver._snap_agv_to_waypoint(agv_prim, agv_xform, waypoints, state.direction)
            state.route_signature = (path_root_path, tuple(wp["name"] for wp in waypoints))
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        start_reverse = bool(waypoints[0]["reverse"])
        end_reverse = bool(waypoints[-1]["reverse"])
        reverse_mode = start_reverse and end_reverse
        db.outputs.reverseMode = reverse_mode

        route_signature = (path_root_path, tuple(wp["name"] for wp in waypoints))
        if state.route_signature != route_signature:
            state.reset()
            state.route_signature = route_signature

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
                    state.bend_state["phase"] = "arc"
                    state.bend_state["angle"] = state.bend_state["start_angle"]
                return True
            if action == "stop":
                state.stopped = True
                db.outputs.execFinished = og.ExecutionAttributeState.ENABLED
                db.outputs.isStopped = True
                return True
            if action == "reverse":
                state.direction *= -1
                state.returning_from_reverse = True
                state.idx += state.direction
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True
            if action == "dock":
                state.dock_returning = True
                state.dock_exit_idx = min(max(state.idx - state.direction, 0), len(waypoints) - 1)
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True
            state.idx += state.direction
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        if state.dock_returning:
            target = waypoints[state.dock_exit_idx]["pos"]
            delta = target - pos
            dist = float(np.linalg.norm(delta[:2]))

            if dist < float(db.inputs.positionToleranceM):
                state.dock_returning = False
                state.stopped = True
                db.outputs.execFinished = og.ExecutionAttributeState.ENABLED
                db.outputs.isStopped = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            if dist > 1e-6:
                dir2 = delta[:2] / dist
                aim = pos.copy()
                aim[:2] = pos[:2] + dir2 * min(float(db.inputs.lookaheadM), dist)
            else:
                aim = target

            yaw_tol = math.radians(float(db.inputs.yawToleranceDeg))
            desired_yaw = yaw
            yaw_err = OgnAgvWaypointDriver._wrap_pi(desired_yaw - yaw)
            yaw_rate_cmd = max(
                -float(db.inputs.maxYawRateRps),
                min(float(db.inputs.maxYawRateRps), 3.0 * yaw_err),
            )
            state.yaw_rate = OgnAgvWaypointDriver._move_towards(
                state.yaw_rate, yaw_rate_cmd, float(db.inputs.maxYawAccelRps2) * dt
            )

            v_brake = math.sqrt(max(0.0, 2.0 * float(db.inputs.maxAccelMps2) * dist))
            v_cmd = min(float(db.inputs.targetSpeedMps), v_brake)
            if abs(yaw_err) > yaw_tol:
                v_cmd = 0.0
            state.lin_speed = OgnAgvWaypointDriver._move_towards(
                state.lin_speed, v_cmd, float(db.inputs.maxAccelMps2) * dt
            )

            yaw_next = yaw + state.yaw_rate * dt
            pos_next = pos.copy()
            pos_next[0] -= math.cos(yaw_next) * state.lin_speed * dt
            pos_next[1] -= math.sin(yaw_next) * state.lin_speed * dt
            pos_next[2] = pos[2]
            OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, pos_next, yaw_next)

            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        target = waypoints[state.idx]["pos"]
        dock_mode = bool(waypoints[state.idx]["dock"])
        bend = state.bend_state
        bend_approach = bool(bend is not None and bend["phase"] == "approach")
        bend_arc = bool(bend is not None and bend["phase"] == "arc")

        if bend_arc:
            if state.dock_returning:
                state.bend_state = None
                state.dock_returning = False
                state.stopped = True
                db.outputs.execFinished = og.ExecutionAttributeState.ENABLED
                db.outputs.isStopped = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            radius = float(bend["radius"])
            angle = float(bend["angle"])
            end_angle = float(bend["end_angle"])
            arc_sign = 1.0 if bend["left"] else -1.0

            v_cmd = float(db.inputs.targetSpeedMps)
            state.lin_speed = OgnAgvWaypointDriver._move_towards(
                state.lin_speed, v_cmd, float(db.inputs.maxAccelMps2) * dt
            )
            omega = 0.0 if radius <= 1e-6 else arc_sign * state.lin_speed / radius
            angle_next = angle + omega * dt
            completed = (arc_sign > 0.0 and angle_next >= end_angle) or (arc_sign < 0.0 and angle_next <= end_angle)
            if completed:
                angle_next = end_angle

            center = bend["center"]
            pos_next = np.array(
                [
                    center[0] + radius * math.cos(angle_next),
                    center[1] + radius * math.sin(angle_next),
                    pos[2],
                ],
                dtype=float,
            )
            yaw_next = angle_next + (math.pi * 0.5 if arc_sign > 0.0 else -math.pi * 0.5)
            state.yaw_rate = omega
            state.bend_state["angle"] = angle_next
            OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, pos_next, yaw_next)

            if completed:
                state.bend_state = None
                state.idx += state.direction
                state.idx = min(max(state.idx, 0), len(waypoints) - 1)

            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        if state.bend_state is None and not dock_mode:
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
                                "phase": "approach",
                                "t2": bend["t2"],
                                "t1": bend["t1"],
                                "center": bend["center"],
                                "left": bend["left"],
                                "v1": bend["v1"],
                                "v2": bend["v2"],
                                "radius": bend["radius"],
                                "start_angle": bend["start_angle"],
                                "end_angle": bend["end_angle"],
                                "angle": bend["start_angle"],
                                "wait_ms": wait_ms,
                                "waited": False,
                            }

        if bend_approach:
            target = np.array([bend["t1"][0], bend["t1"][1], waypoints[state.idx]["pos"][2]], dtype=float)

        delta = target - pos
        dist = float(np.linalg.norm(delta[:2]))

        if bend_approach and dist < float(db.inputs.positionToleranceM):
            wait_ms = int(bend["wait_ms"])
            if wait_ms > 0 and not bend["waited"]:
                state.waiting = True
                state.wait_remaining_s = wait_ms / 1000.0
                state.pending_endpoint_action = "bend"
                db.outputs.isWaiting = True
                bend["waited"] = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            state.bend_state["phase"] = "arc"
            state.bend_state["angle"] = state.bend_state["start_angle"]
            OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
            return True

        if dist < float(db.inputs.positionToleranceM) and not bend_approach and not bend_arc:
            wait_ms = int(waypoints[state.idx]["wait_ms"])
            at_first = state.idx == 0
            at_last = state.idx == len(waypoints) - 1

            endpoint_action = None
            if state.returning_from_reverse:
                endpoint_action = "stop"
            elif at_last and state.direction == 1:
                endpoint_action = "reverse" if (reverse_mode or end_reverse) else "stop"
            elif at_first and state.direction == -1:
                endpoint_action = "reverse" if (reverse_mode or start_reverse) else "stop"

            if wait_ms > 0:
                state.waiting = True
                state.wait_remaining_s = wait_ms / 1000.0
                state.pending_endpoint_action = endpoint_action
                db.outputs.isWaiting = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            if endpoint_action == "stop":
                state.stopped = True
                db.outputs.execFinished = og.ExecutionAttributeState.ENABLED
                db.outputs.isStopped = True
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True
            if endpoint_action == "reverse":
                state.direction *= -1
                if not reverse_mode:
                    state.returning_from_reverse = True
                state.idx += state.direction
            else:
                state.idx += state.direction

            state.idx = min(max(state.idx, 0), len(waypoints) - 1)
            if endpoint_action is not None:
                OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
                return True

            target = waypoints[state.idx]["pos"]
            dock_mode = bool(waypoints[state.idx]["dock"])
            delta = target - pos
            dist = float(np.linalg.norm(delta[:2]))

        if dist > 1e-6:
            dir2 = delta[:2] / dist
            aim = pos.copy()
            aim[:2] = pos[:2] + dir2 * min(float(db.inputs.lookaheadM), dist)
        else:
            aim = target

        yaw_tol = math.radians(float(db.inputs.yawToleranceDeg))
        if bend_approach:
            desired_yaw = math.atan2(bend["v1"][1], bend["v1"][0])
        elif dock_mode:
            desired_yaw = yaw
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

        if bend_approach:
            v_cmd = float(db.inputs.targetSpeedMps)
        elif dock_mode:
            v_brake = math.sqrt(max(0.0, 2.0 * float(db.inputs.maxAccelMps2) * dist))
            v_cmd = min(float(db.inputs.targetSpeedMps), v_brake)
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
        motion_sign = -1.0 if dock_mode else 1.0
        pos_next[0] += math.cos(yaw_next) * state.lin_speed * dt * motion_sign
        pos_next[1] += math.sin(yaw_next) * state.lin_speed * dt * motion_sign
        pos_next[2] = pos[2]
        OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, pos_next, yaw_next)

        OgnAgvWaypointDriver._set_waypoint_outputs(db, state, waypoints)
        return True

    @staticmethod
    def _set_default_outputs(db):
        db.outputs.execFinished = og.ExecutionAttributeState.DISABLED
        db.outputs.isRouteValid = False
        db.outputs.isWaiting = False
        db.outputs.isStopped = False
        db.outputs.isRunning = False
        db.outputs.reverseMode = False
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
        start_angle = math.atan2(t1[1] - center[1], t1[0] - center[0])
        end_angle = math.atan2(t2[1] - center[1], t2[0] - center[0])
        return {
            "t1": t1,
            "t2": t2,
            "center": center,
            "left": cross > 0.0,
            "v1": v1,
            "v2": v2,
            "len2": len2,
            "d": d,
            "radius": radius,
            "start_angle": start_angle,
            "end_angle": end_angle,
        }

    @staticmethod
    def _set_local_pose_xformable(xformable, pos_xyz, yaw):
        pos_d = Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]))
        quat_d = OgnAgvWaypointDriver._quat_from_yaw(float(yaw))

        ops = xformable.GetOrderedXformOps()
        translate_op = None
        orient_op = None
        fallback_translate_op = None
        fallback_orient_op = None
        for op in ops:
            if op.IsInverseOp():
                continue

            op_name = str(op.GetName())
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                if op_name.endswith(":agvDriver"):
                    translate_op = op
                elif fallback_translate_op is None:
                    fallback_translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                if op_name.endswith(":agvDriver"):
                    orient_op = op
                elif fallback_orient_op is None:
                    fallback_orient_op = op

            if translate_op is not None and orient_op is not None:
                break

        if translate_op is None:
            translate_op = fallback_translate_op
        if orient_op is None:
            orient_op = fallback_orient_op

        if translate_op is None:
            translate_op = xformable.AddTranslateOp(opSuffix="agvDriver")
        if orient_op is None:
            orient_op = xformable.AddOrientOp(opSuffix="agvDriver")

        if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            translate_op.Set(Gf.Vec3f(float(pos_d[0]), float(pos_d[1]), float(pos_d[2])))
        else:
            translate_op.Set(pos_d)

        imag = quat_d.GetImaginary()
        if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            orient_op.Set(Gf.Quatf(float(quat_d.GetReal()), Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2]))))
        else:
            orient_op.Set(quat_d)

    @staticmethod
    def _snap_agv_to_waypoint(agv_prim, agv_xform, waypoints, direction):
        target_pos = waypoints[0]["pos"]
        target_yaw = 0.0

        if len(waypoints) >= 2:
            next_idx = min(max(0 + direction, 0), len(waypoints) - 1)
            next_pos = waypoints[next_idx]["pos"]
            delta = next_pos - target_pos
            if float(np.linalg.norm(delta[:2])) > 1e-6:
                target_yaw = math.atan2(delta[1], delta[0])
            else:
                _pos, quat = OgnAgvWaypointDriver._get_world_pose(agv_prim)
                target_yaw = OgnAgvWaypointDriver._yaw_from_quat(quat)
        else:
            _pos, quat = OgnAgvWaypointDriver._get_world_pose(agv_prim)
            target_yaw = OgnAgvWaypointDriver._yaw_from_quat(quat)

        OgnAgvWaypointDriver._set_local_pose_xformable(agv_xform, target_pos, target_yaw)

    @staticmethod
    def _reset_tracked_agv(state):
        if not state.agv_path or not state.path_root_path:
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        agv_prim = stage.GetPrimAtPath(state.agv_path)
        if not agv_prim.IsValid():
            return

        agv_xform = UsdGeom.Xformable(agv_prim)
        if not agv_xform:
            return

        waypoints = OgnAgvWaypointDriver._read_waypoints(stage, state.path_root_path)
        if len(waypoints) < 2:
            return

        OgnAgvWaypointDriver._snap_agv_to_waypoint(agv_prim, agv_xform, waypoints, 1)

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
            waypoint_data = child.GetCustomData().get("waypoint", {})
            wait_ms = waypoint_data.get("waitMs")
            bend_radius_cm = waypoint_data.get("bendRadiusCm")
            items.append(
                {
                    "name": name,
                    "pos": pos,
                    "reverse": bool(waypoint_data.get("reverse", False)),
                    "dock": bool(waypoint_data.get("dock", False) or waypoint_data.get("reverseDrive", False)),
                    "wait_ms": int(wait_ms) if wait_ms is not None else 0,
                    "bend_radius": float(bend_radius_cm) / 100.0 if bend_radius_cm is not None else 0.0,
                }
            )
        return items
