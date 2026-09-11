# Deprecated location: canonical module is tools/colored_map/geometry_aware_fusion.py.
import importlib.util as _ilu
import sys as _sys
from pathlib import Path as _Path

_target = (_Path(__file__).resolve().parents[1] / 'colored_map' /
           'geometry_aware_fusion.py')
_spec = _ilu.spec_from_file_location('_colored_map_geometry_aware_fusion', _target)
_mod = _ilu.module_from_spec(_spec)
_sys.modules[_spec.name] = _mod
_spec.loader.exec_module(_mod)
_sys.modules[__name__] = _mod
globals().update(_mod.__dict__)
