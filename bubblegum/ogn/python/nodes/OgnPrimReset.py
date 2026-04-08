import omni.graph.core as og
from pxr import Gf, UsdGeom, UsdPhysics


class OgnPrimReset:
    class _State:
        def __init__(self):
            self.prim_path = ""
            self.local_transform = None
            self.xform_state = None
            self.rigid_body_enabled = None
            self.kinematic_enabled = None

        def clear(self):
            self.prim_path = ""
            self.local_transform = None
            self.xform_state = None
            self.rigid_body_enabled = None
            self.kinematic_enabled = None

        def is_captured(self):
            return bool(self.prim_path)

    @staticmethod
    def internal_state():
        return OgnPrimReset._State()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        db.outputs.isCaptured = state.is_captured()
        db.outputs.trackedPrimPath = state.prim_path

        stage = OgnPrimReset._get_stage()
        if stage is None:
            db.log_error("No USD stage is available.")
            return False

        prim_paths = OgnPrimReset._extract_target_paths(db.inputs.prim)
        prim_path = prim_paths[0] if prim_paths else ""
        if not prim_path:
            state.clear()
            db.outputs.isCaptured = False
            db.outputs.trackedPrimPath = ""
            return True

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            db.log_error(f"Prim does not exist: {prim_path}")
            return False

        if state.prim_path != prim_path or not state.is_captured():
            OgnPrimReset._capture_state(prim, state)

        if OgnPrimReset._execution_requested(db.inputs, "execReset"):
            OgnPrimReset._restore_state(prim, state)
            db.outputs.execResetDone = og.ExecutionAttributeState.ENABLED

        db.outputs.isCaptured = state.is_captured()
        db.outputs.trackedPrimPath = state.prim_path
        return True

    @staticmethod
    def _get_stage():
        import omni.usd

        return omni.usd.get_context().get_stage()

    @staticmethod
    def _execution_requested(inputs, attr_name):
        value = getattr(inputs, attr_name, None)
        if value is None:
            return False
        return value == og.ExecutionAttributeState.ENABLED

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
    def _capture_state(prim, state):
        xformable = UsdGeom.Xformable(prim)
        local_transform = None
        if xformable:
            local_transform = OgnPrimReset._get_local_transformation(prim)
            state.xform_state = OgnPrimReset._capture_xform_state(prim)
        else:
            state.xform_state = None

        state.prim_path = prim.GetPath().pathString
        state.local_transform = Gf.Matrix4d(local_transform) if local_transform is not None else None
        state.rigid_body_enabled = None
        state.kinematic_enabled = None

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_enabled = rigid_body_api.GetRigidBodyEnabledAttr().Get()
            kinematic_enabled = rigid_body_api.GetKinematicEnabledAttr().Get()
            state.rigid_body_enabled = bool(rigid_enabled) if rigid_enabled is not None else None
            state.kinematic_enabled = bool(kinematic_enabled) if kinematic_enabled is not None else False

    @staticmethod
    def _restore_state(prim, state):
        if state.xform_state is not None:
            OgnPrimReset._restore_xform_state(prim, state.xform_state)
        elif state.local_transform is not None:
            OgnPrimReset._restore_local_transform(prim, state.local_transform)

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            if state.rigid_body_enabled is not None:
                rigid_body_api.GetRigidBodyEnabledAttr().Set(state.rigid_body_enabled)
            if state.kinematic_enabled is not None:
                rigid_body_api.GetKinematicEnabledAttr().Set(state.kinematic_enabled)
            OgnPrimReset._zero_rigid_body_velocities(prim)

    @staticmethod
    def _zero_rigid_body_velocities(prim):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return

        zero_vec_f = Gf.Vec3f(0.0, 0.0, 0.0)
        zero_vec_d = Gf.Vec3d(0.0, 0.0, 0.0)

        try:
            from pxr import PhysxSchema

            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            linear_attr = physx_api.GetVelocityAttr() or physx_api.CreateVelocityAttr()
            angular_attr = physx_api.GetAngularVelocityAttr() or physx_api.CreateAngularVelocityAttr()
            linear_attr.Set(zero_vec_f)
            angular_attr.Set(zero_vec_f)
        except Exception:
            pass

        try:
            from pxr import PhysxSchema

            rigid_api = PhysxSchema.PhysxRigidBodyAPI(prim)
            linear_attr = rigid_api.GetVelocityAttr()
            angular_attr = rigid_api.GetAngularVelocityAttr()
            if linear_attr:
                linear_attr.Set(zero_vec_f)
            if angular_attr:
                angular_attr.Set(zero_vec_f)
        except Exception:
            pass

        try:
            import numpy as np
            import omni.physics.tensors as physics_tensors

            simulation_view = physics_tensors.create_simulation_view("numpy")
            rigid_body_view = simulation_view.create_rigid_body_view(prim.GetPath().pathString)
            if rigid_body_view.count <= 0:
                return

            indices = np.arange(rigid_body_view.count, dtype=np.int32)
            zero_linear = np.zeros((rigid_body_view.count, 3), dtype=np.float32)
            zero_angular = np.zeros((rigid_body_view.count, 3), dtype=np.float32)
            rigid_body_view.set_linear_velocities(zero_linear, indices)
            rigid_body_view.set_angular_velocities(zero_angular, indices)
        except Exception:
            pass

        try:
            UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(zero_vec_d)
        except Exception:
            pass

    @staticmethod
    def _get_local_transformation(prim):
        xform_cache = UsdGeom.XformCache()
        local_transform = xform_cache.GetLocalTransformation(prim)
        if isinstance(local_transform, tuple):
            local_transform = local_transform[0]
        return local_transform

    @staticmethod
    def _capture_xform_state(prim):
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return None

        ordered_ops = list(xformable.GetOrderedXformOps())
        reset_stack = False
        if hasattr(xformable, "GetResetXformStack"):
            reset_stack = bool(xformable.GetResetXformStack())

        ops = []
        for op in ordered_ops:
            try:
                value = op.Get()
            except Exception:
                value = None

            op_name = str(op.GetName())
            name_parts = op_name.split(":")
            suffix = ":".join(name_parts[2:]) if len(name_parts) > 2 else ""
            ops.append(
                {
                    "op_type": op.GetOpType(),
                    "precision": op.GetPrecision(),
                    "suffix": suffix,
                    "is_inverse": op.IsInverseOp(),
                    "value": value,
                }
            )

        return {"reset_stack": reset_stack, "ops": ops}

    @staticmethod
    def _restore_xform_state(prim, xform_state):
        xformable = UsdGeom.Xformable(prim)
        if not xformable or xform_state is None:
            return

        OgnPrimReset._remove_all_xform_ops(xformable)
        restored_ops = []
        for op_state in xform_state["ops"]:
            op = xformable.AddXformOp(
                op_state["op_type"],
                precision=op_state["precision"],
                opSuffix=op_state["suffix"],
                isInverseOp=op_state["is_inverse"],
            )
            if op_state["value"] is not None:
                op.Set(op_state["value"])
            restored_ops.append(op)

        xformable.SetXformOpOrder(restored_ops, resetXformStack=xform_state["reset_stack"])

    @staticmethod
    def _restore_local_transform(prim, local_transform):
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return

        transform_op = OgnPrimReset._get_or_create_transform_op(xformable, local_transform)
        transform_op.Set(local_transform)

    @staticmethod
    def _get_or_create_transform_op(xformable, current_local):
        ordered_ops = list(xformable.GetOrderedXformOps())
        for op in ordered_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform and not op.IsInverseOp():
                OgnPrimReset._remove_extra_xform_ops(xformable, keep_op=op)
                xformable.SetXformOpOrder([op], resetXformStack=False)
                op.Set(current_local)
                return op

        xformable.ClearXformOpOrder()
        transform_op = xformable.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble)
        transform_op.Set(current_local)
        OgnPrimReset._remove_extra_xform_ops(xformable, keep_op=transform_op)
        return transform_op

    @staticmethod
    def _remove_extra_xform_ops(xformable, keep_op):
        keep_name = str(keep_op.GetName())
        for op in list(xformable.GetOrderedXformOps()):
            op_name = str(op.GetName())
            if op_name != keep_name:
                xformable.GetPrim().RemoveProperty(op_name)

        prim = xformable.GetPrim()
        for prop in list(prim.GetProperties()):
            prop_name = prop.GetName()
            if prop_name.startswith("xformOp:") and prop_name != keep_name:
                prim.RemoveProperty(prop_name)

    @staticmethod
    def _remove_all_xform_ops(xformable):
        xformable.ClearXformOpOrder()
        prim = xformable.GetPrim()
        for prop in list(prim.GetProperties()):
            prop_name = prop.GetName()
            if prop_name.startswith("xformOp:"):
                prim.RemoveProperty(prop_name)
