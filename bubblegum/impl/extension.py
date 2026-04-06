import omni.ext
import omni.usd

from ..ogn.python.nodes.OgnBubblegumPy import OgnBubblegumPy


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._stage_subscription = None

        usd_context = omni.usd.get_context()
        if usd_context is None:
            return

        self._stage_subscription = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event
        )

        stage = usd_context.get_stage()
        if stage is not None:
            OgnBubblegumPy.restore_stage_from_metadata(stage)

    def on_shutdown(self):
        self._stage_subscription = None

    def _on_stage_event(self, event):
        if event.type != int(omni.usd.StageEventType.OPENED):
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        OgnBubblegumPy.restore_stage_from_metadata(stage)
