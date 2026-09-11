# Deprecated location: this module moved to tools/colored_map/build_lidar_init.py.
# The shim loads the canonical module and redirects future imports to it.
import importlib.util as _ilu
import sys as _sys
from pathlib import Path as _Path

_target = _Path(__file__).resolve().parents[1] / 'colored_map' / 'build_lidar_init.py'
_spec = _ilu.spec_from_file_location('_colored_map_build_lidar_init', _target)
_mod = _ilu.module_from_spec(_spec)
_sys.modules[_spec.name] = _mod
_spec.loader.exec_module(_mod)
_sys.modules[__name__] = _mod
globals().update(_mod.__dict__)

if __name__ == '__main__':
    raise SystemExit(_mod.main())
