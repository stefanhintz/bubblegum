import glob
import os
import sys

import carb
import omni.ext


class BubblegumExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._log_startup_diagnostics()

    def on_shutdown(self):
        self._ext_id = None

    def _log_startup_diagnostics(self):
        module_dir = os.path.dirname(os.path.abspath(__file__))
        python_dir = os.path.dirname(module_dir)
        extension_root = os.path.dirname(python_dir)

        carb.log_warn(f"[bubblegum] startup ext_id={self._ext_id}")
        carb.log_warn(f"[bubblegum] __file__={__file__}")
        carb.log_warn(f"[bubblegum] extension_root={extension_root}")
        carb.log_warn(f"[bubblegum] python_dir={python_dir}")
        carb.log_warn(f"[bubblegum] module_dir={module_dir}")
        carb.log_warn(f"[bubblegum] sys.path[0:5]={sys.path[:5]}")

        candidate_dirs = [
            os.path.join(python_dir, "ogn", "python", "nodes"),
            os.path.join(module_dir, "ogn", "python", "nodes"),
            os.path.join(module_dir, "nodes"),
            os.path.join(python_dir, "nodes"),
            os.path.join(extension_root, "ogn", "python", "nodes"),
        ]

        for path in candidate_dirs:
            exists = os.path.isdir(path)
            ogn_files = sorted(glob.glob(os.path.join(path, "*.ogn"))) if exists else []
            py_files = sorted(glob.glob(os.path.join(path, "Ogn*.py"))) if exists else []
            carb.log_warn(
                f"[bubblegum] probe path={path} exists={exists} ogn_files={ogn_files} node_py_files={py_files}"
            )
