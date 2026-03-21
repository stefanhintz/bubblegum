import omni.graph.core as og
import omni.usd
from pxr import Gf, Usd, UsdGeom


class _BubblegumStickyState:
    def __init__(self):
        self.attached_prim_path = ""

    def clear(self):
        self.attached_prim_path = ""


class OgnBubblegumStickyPick:
    @staticmethod
    def internal_state():
        return _BubblegumStickyState()

    @staticmethod
    def compute(db) -> bool:
        OgnBubblegumStickyPick._set_exec_out(db)
        db.outputs.didAttach = False
        db.outputs.didRelease = False

        state = OgnBubblegumStickyPick._get_state(db)
        db.outputs.isAttached = bool(state.attached_prim_path)
        db.outputs.attachedPrimPath = state.attached_prim_path

        if not db.inputs.enabled:
            return True

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            db.log_error("No USD stage is available.")
            return False

        helper_path = str(db.inputs.helperPrimPath).strip()
        if not helper_path:
            db.log_error("inputs:helperPrimPath is required.")
            return False

        helper_prim = stage.GetPrimAtPath(helper_path)
        if not helper_prim.IsValid():
            db.log_error(f"Helper prim does not exist: {helper_path}")
            return False

        if db.inputs.release and state.attached_prim_path:
            state.clear()
            db.outputs.didRelease = True

        if state.attached_prim_path:
            attached_prim = stage.GetPrimAtPath(state.attached_prim_path)
            if not attached_prim.IsValid():
                db.log_error(f"Attached prim no longer exists: {state.attached_prim_path}")
                return False
            OgnBubblegumStickyPick._snap_attached_prim(helper_prim, attached_prim)
        elif db.inputs.stick:
            candidate_prim = OgnBubblegumStickyPick._find_candidate_prim(
                db=db,
                stage=stage,
                helper_prim=helper_prim,
                candidate_paths=OgnBubblegumStickyPick._normalize_candidate_paths(db.inputs.candidatePrimPaths),
            )
            if candidate_prim is not None:
                state.attached_prim_path = candidate_prim.GetPath().pathString
                OgnBubblegumStickyPick._snap_attached_prim(helper_prim, candidate_prim)
                db.outputs.didAttach = True

        db.outputs.isAttached = bool(state.attached_prim_path)
        db.outputs.attachedPrimPath = state.attached_prim_path
        return True

    @staticmethod
    def _get_state(db):
        if hasattr(db, "per_instance_state"):
            return db.per_instance_state
        return db.internal_state

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
    def _find_candidate_prim(db, stage, helper_prim, candidate_paths):
        helper_bounds = OgnBubblegumStickyPick._compute_world_aligned_bounds(helper_prim)
        if helper_bounds.IsEmpty():
            db.log_error(
                f"Helper prim has no world bounds. Use a helper prim with visible geometry or bounded children: "
                f"{helper_prim.GetPath().pathString}"
            )
            return None

        helper_path = helper_prim.GetPath()

        if candidate_paths:
            for candidate_path in candidate_paths:
                candidate_prim = stage.GetPrimAtPath(candidate_path)
                if not candidate_prim.IsValid():
                    db.log_error(f"Candidate prim does not exist: {candidate_path}")
                    return None
                if OgnBubblegumStickyPick._is_attachable_candidate(helper_path, candidate_prim, helper_bounds):
                    return candidate_prim
            return None

        for candidate_prim in stage.Traverse():
            if OgnBubblegumStickyPick._is_attachable_candidate(helper_path, candidate_prim, helper_bounds):
                return candidate_prim
        return None

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

        xformable = UsdGeom.Xformable(candidate_prim)
        imageable = UsdGeom.Imageable(candidate_prim)
        if not xformable or not imageable:
            return False

        candidate_bounds = OgnBubblegumStickyPick._compute_world_aligned_bounds(candidate_prim)
        if candidate_bounds.IsEmpty():
            return False

        return OgnBubblegumStickyPick._ranges_intersect(helper_bounds, candidate_bounds)

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
    def _snap_attached_prim(helper_prim, attached_prim):
        helper_world = omni.usd.get_world_transform_matrix(helper_prim)

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        current_local = xform_cache.GetLocalTransformation(attached_prim)
        current_scale = OgnBubblegumStickyPick._extract_scale(current_local)

        parent_world = xform_cache.GetParentToWorldTransform(attached_prim)
        target_local = helper_world * parent_world.GetInverse()

        target_translation = target_local.ExtractTranslation()
        target_rotation = target_local.ExtractRotation()
        target_matrix = OgnBubblegumStickyPick._compose_matrix(
            translation=target_translation,
            rotation=target_rotation,
            scale=current_scale,
        )

        xformable = UsdGeom.Xformable(attached_prim)
        transform_op = OgnBubblegumStickyPick._get_or_create_transform_op(xformable)
        transform_op.Set(target_matrix)

    @staticmethod
    def _get_or_create_transform_op(xformable):
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform and not op.IsInverseOp():
                xformable.SetXformOpOrder([op], resetXformStack=False)
                return op

        xformable.ClearXformOpOrder()
        return xformable.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble)

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
