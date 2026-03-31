import fnmatch

import omni.graph.core as og
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics


class OgnBubblegumPy:
    class _State:
        def __init__(self):
            self.attached_prim_path = ""
            self.attached_to_helper = Gf.Matrix4d(1.0)
            self.restore_kinematic_enabled = None
            self.original_local_transforms = {}
            self.original_kinematic_enabled = {}
            self.touched_prim_paths = set()
            self._timeline_sub = None
            self._subscribe_timeline_stop()

        def _subscribe_timeline_stop(self):
            timeline = omni.timeline.get_timeline_interface()
            if timeline is None:
                return

            self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline_event
            )

        def _on_timeline_event(self, event):
            if event.type != int(omni.timeline.TimelineEventType.STOP):
                return

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return

            OgnBubblegumPy._restore_all_touched_objects(stage, self)

    @staticmethod
    def internal_state():
        return OgnBubblegumPy._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        OgnBubblegumPy._set_exec_out(db)
        db.outputs.didAttach = False
        db.outputs.didRelease = False
        db.outputs.isAttached = bool(state.attached_prim_path)
        db.outputs.attachedPrimPath = state.attached_prim_path

        if not db.inputs.enabled:
            return True

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            db.log_error("No USD stage is available.")
            return False

        timeline = omni.timeline.get_timeline_interface()
        is_playing = timeline.is_playing() if timeline is not None else True
        if not is_playing:
            OgnBubblegumPy._restore_all_touched_objects(stage, state)
            db.outputs.isAttached = False
            db.outputs.attachedPrimPath = ""
            return True

        helper_paths = OgnBubblegumPy._extract_target_paths(db.inputs.helperPrim)
        helper_path = helper_paths[0] if helper_paths else ""
        if not helper_path:
            db.log_error("inputs:helperPrim is required.")
            return False

        helper_prim = stage.GetPrimAtPath(helper_path)
        if not helper_prim.IsValid():
            db.log_error(f"Helper prim does not exist: {helper_path}")
            return False

        if not db.inputs.stick and state.attached_prim_path:
            OgnBubblegumPy._clear_attachment_state(stage, state, restore_transform=False)
            db.outputs.didRelease = True

        if state.attached_prim_path:
            attached_prim = stage.GetPrimAtPath(state.attached_prim_path)
            if not attached_prim.IsValid():
                db.log_error(f"Attached prim no longer exists: {state.attached_prim_path}")
                return False
            OgnBubblegumPy._snap_attached_prim(helper_prim, attached_prim, state.attached_to_helper)
        elif db.inputs.stick:
            candidate_prim = OgnBubblegumPy._find_candidate_prim(
                db=db,
                stage=stage,
                helper_prim=helper_prim,
                candidate_paths=OgnBubblegumPy._normalize_candidate_paths(db.inputs.candidatePrimPaths),
            )
            if candidate_prim is not None:
                candidate_path = candidate_prim.GetPath().pathString
                current_local = OgnBubblegumPy._prepare_transform_control(candidate_prim)
                if candidate_path not in state.original_local_transforms and current_local is not None:
                    state.original_local_transforms[candidate_path] = Gf.Matrix4d(current_local)
                state.touched_prim_paths.add(candidate_path)
                state.attached_prim_path = candidate_prim.GetPath().pathString
                state.attached_to_helper = OgnBubblegumPy._compute_attach_offset(helper_prim, candidate_prim)
                state.restore_kinematic_enabled = OgnBubblegumPy._set_kinematic_while_held(candidate_prim)
                if candidate_path not in state.original_kinematic_enabled:
                    state.original_kinematic_enabled[candidate_path] = state.restore_kinematic_enabled
                db.outputs.didAttach = True

        db.outputs.isAttached = bool(state.attached_prim_path)
        db.outputs.attachedPrimPath = state.attached_prim_path
        return True

    @staticmethod
    def _set_exec_out(db):
        if hasattr(db.outputs, "execOut"):
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

    @staticmethod
    def _normalize_candidate_paths(candidate_paths):
        normalized = []
        for path in candidate_paths:
            value = str(path).strip()
            if value:
                normalized.append(value)
        return normalized

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
    def _clear_attachment_state(stage, state, restore_transform):
        if state.attached_prim_path:
            released_prim = stage.GetPrimAtPath(state.attached_prim_path)
            if released_prim.IsValid():
                if restore_transform:
                    original_local = state.original_local_transforms.get(state.attached_prim_path)
                    if original_local is not None:
                        OgnBubblegumPy._restore_local_transform(released_prim, original_local)
                OgnBubblegumPy._reset_rigid_body_after_release(released_prim, state.restore_kinematic_enabled)

        state.attached_prim_path = ""
        state.attached_to_helper = Gf.Matrix4d(1.0)
        state.restore_kinematic_enabled = None

    @staticmethod
    def _restore_all_touched_objects(stage, state):
        if state.attached_prim_path:
            state.touched_prim_paths.add(state.attached_prim_path)

        for prim_path in list(state.touched_prim_paths):
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                continue

            original_local = state.original_local_transforms.get(prim_path)
            if original_local is not None:
                OgnBubblegumPy._restore_local_transform(prim, original_local)

            OgnBubblegumPy._reset_rigid_body_after_release(prim, state.original_kinematic_enabled.get(prim_path))

        state.attached_prim_path = ""
        state.attached_to_helper = Gf.Matrix4d(1.0)
        state.restore_kinematic_enabled = None
        state.touched_prim_paths.clear()
        state.original_local_transforms.clear()
        state.original_kinematic_enabled.clear()

    @staticmethod
    def _find_candidate_prim(db, stage, helper_prim, candidate_paths):
        helper_bounds = OgnBubblegumPy._compute_world_aligned_bounds(helper_prim)
        if helper_bounds.IsEmpty():
            db.log_error(
                "Helper prim has no world bounds. Use a helper prim with visible geometry or bounded children: "
                f"{helper_prim.GetPath().pathString}"
            )
            return None

        helper_path = helper_prim.GetPath()

        if candidate_paths:
            for candidate_prim in stage.Traverse():
                if not OgnBubblegumPy._matches_candidate_filter(candidate_prim, candidate_paths):
                    continue
                if OgnBubblegumPy._is_attachable_candidate(helper_path, candidate_prim, helper_bounds):
                    return candidate_prim
            return None

        for candidate_prim in stage.Traverse():
            if OgnBubblegumPy._is_attachable_candidate(helper_path, candidate_prim, helper_bounds):
                return candidate_prim

        return None

    @staticmethod
    def _matches_candidate_filter(candidate_prim, candidate_filters):
        candidate_path = candidate_prim.GetPath().pathString
        candidate_path_without_root = candidate_path.lstrip("/")

        for candidate_filter in candidate_filters:
            normalized_filter = candidate_filter.strip()
            if not normalized_filter:
                continue

            if (
                candidate_path == normalized_filter
                or candidate_path_without_root == normalized_filter
                or fnmatch.fnmatchcase(candidate_path, normalized_filter)
                or fnmatch.fnmatchcase(candidate_path_without_root, normalized_filter)
                or fnmatch.fnmatchcase(candidate_path, f"*/{normalized_filter}")
                or fnmatch.fnmatchcase(candidate_path_without_root, f"*/{normalized_filter}")
            ):
                return True

        return False

    @staticmethod
    def _is_attachable_candidate(helper_path, candidate_prim, helper_bounds):
        if not candidate_prim.IsValid():
            return False
        if not candidate_prim.IsActive():
            return False
        if candidate_prim.IsInstanceProxy():
            return False

        candidate_path = candidate_prim.GetPath()
        if candidate_path == helper_path:
            return False
        if candidate_path.HasPrefix(helper_path) or helper_path.HasPrefix(candidate_path):
            return False

        if not UsdGeom.Xformable(candidate_prim) or not UsdGeom.Imageable(candidate_prim):
            return False

        candidate_bounds = OgnBubblegumPy._compute_world_aligned_bounds(candidate_prim)
        if candidate_bounds.IsEmpty():
            return False

        return OgnBubblegumPy._ranges_intersect(helper_bounds, candidate_bounds)

    @staticmethod
    def _compute_world_aligned_bounds(prim):
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            useExtentsHint=True,
        )
        return bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()

    @staticmethod
    def _ranges_intersect(a, b):
        a_min = a.GetMin()
        a_max = a.GetMax()
        b_min = b.GetMin()
        b_max = b.GetMax()
        return (
            a_min[0] <= b_max[0]
            and a_max[0] >= b_min[0]
            and a_min[1] <= b_max[1]
            and a_max[1] >= b_min[1]
            and a_min[2] <= b_max[2]
            and a_max[2] >= b_min[2]
        )

    @staticmethod
    def _compute_attach_offset(helper_prim, attached_prim):
        helper_world = omni.usd.get_world_transform_matrix(helper_prim)
        attached_world = omni.usd.get_world_transform_matrix(attached_prim)
        return attached_world * helper_world.GetInverse()

    @staticmethod
    def _set_kinematic_while_held(prim):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return None

        try:
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_api.GetRigidBodyEnabledAttr().Set(True)
            kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
            was_kinematic = kinematic_attr.Get()
            if was_kinematic is None:
                was_kinematic = False
            kinematic_attr.Set(True)
            return was_kinematic
        except Exception:
            return None

    @staticmethod
    def _reset_rigid_body_after_release(prim, enabled):
        if enabled is None or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return

        try:
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_enabled_attr = rigid_body_api.GetRigidBodyEnabledAttr()
            kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()

            # Explicit reset sequence:
            # 1. disable rigid body
            # 2. enable kinematic
            # 3. restore requested kinematic state
            # 4. re-enable rigid body
            rigid_body_enabled_attr.Set(False)
            kinematic_attr.Set(True)
            kinematic_attr.Set(enabled)
            rigid_body_enabled_attr.Set(True)

            if not enabled:
                OgnBubblegumPy._wake_rigid_body(prim)
        except Exception:
            pass

    @staticmethod
    def _wake_rigid_body(prim):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return

        try:
            import numpy as np
            import omni.physics.tensors as physics_tensors

            simulation_view = physics_tensors.create_simulation_view("numpy")
            rigid_body_view = simulation_view.create_rigid_body_view(prim.GetPath().pathString)
            if rigid_body_view.count <= 0:
                return

            indices = np.arange(rigid_body_view.count, dtype=np.int32)
            rigid_body_view.wake_up(indices)
        except Exception:
            pass

    @staticmethod
    def _prepare_transform_control(prim):
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return None

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        current_local = xform_cache.GetLocalTransformation(prim)
        if isinstance(current_local, tuple):
            current_local = current_local[0]

        transform_op = OgnBubblegumPy._get_or_create_transform_op(xformable, current_local)
        transform_op.Set(current_local)
        return current_local

    @staticmethod
    def _restore_local_transform(prim, local_transform):
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return

        transform_op = OgnBubblegumPy._get_or_create_transform_op(xformable, local_transform)
        transform_op.Set(local_transform)

    @staticmethod
    def _snap_attached_prim(helper_prim, attached_prim, attached_to_helper):
        helper_world = omni.usd.get_world_transform_matrix(helper_prim)
        target_world = attached_to_helper * helper_world

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        current_local = xform_cache.GetLocalTransformation(attached_prim)
        if isinstance(current_local, tuple):
            current_local = current_local[0]
        current_scale = OgnBubblegumPy._extract_scale(current_local)
        parent_world = xform_cache.GetParentToWorldTransform(attached_prim)
        target_local = target_world * parent_world.GetInverse()

        target_matrix = OgnBubblegumPy._compose_matrix(
            translation=target_local.ExtractTranslation(),
            rotation=target_local.ExtractRotation(),
            scale=current_scale,
        )

        xformable = UsdGeom.Xformable(attached_prim)
        transform_op = OgnBubblegumPy._get_or_create_transform_op(xformable, current_local)
        transform_op.Set(target_matrix)

    @staticmethod
    def _get_or_create_transform_op(xformable, current_local):
        ordered_ops = list(xformable.GetOrderedXformOps())
        for op in ordered_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform and not op.IsInverseOp():
                OgnBubblegumPy._remove_extra_xform_ops(xformable, keep_op=op)
                xformable.SetXformOpOrder([op], resetXformStack=False)
                op.Set(current_local)
                return op

        xformable.ClearXformOpOrder()
        transform_op = xformable.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble)
        transform_op.Set(current_local)
        OgnBubblegumPy._remove_extra_xform_ops(xformable, keep_op=transform_op)
        return transform_op

    @staticmethod
    def _remove_extra_xform_ops(xformable, keep_op):
        keep_name = str(keep_op.GetName())
        for op in list(xformable.GetOrderedXformOps()):
            op_name = str(op.GetName())
            if op_name != keep_name:
                xformable.GetPrim().RemoveProperty(op_name)

        # Also remove authored xform ops that may no longer be ordered, so downstream tools
        # do not try to keep driving stale orient/rotate/scale properties.
        prim = xformable.GetPrim()
        for prop in list(prim.GetProperties()):
            prop_name = prop.GetName()
            if prop_name.startswith("xformOp:") and prop_name != keep_name:
                prim.RemoveProperty(prop_name)

    @staticmethod
    def _extract_scale(matrix):
        rotation_matrix = matrix.ExtractRotationMatrix()
        return Gf.Vec3d(*(axis.GetLength() for axis in rotation_matrix))

    @staticmethod
    def _compose_matrix(translation, rotation, scale):
        scale_matrix = Gf.Matrix4d(1.0)
        scale_matrix.SetScale(scale)

        rotation_matrix = Gf.Matrix4d(1.0)
        rotation_matrix.SetRotate(rotation)

        translation_matrix = Gf.Matrix4d(1.0)
        translation_matrix.SetTranslate(translation)

        return scale_matrix * rotation_matrix * translation_matrix
