"""Platform-A entry point for the shared independent implementation."""

import importlib.util
import sys
from pathlib import Path

_shared_path = Path(__file__).resolve().parents[2] / "pry_buckle" / "horizontal_diameter.py"
_spec = importlib.util.spec_from_file_location("_pry_buckle_shared_horizontal_diameter", _shared_path)
if _spec is None or _spec.loader is None:
    raise ImportError(f"cannot load shared pry-buckle algorithm: {_shared_path}")
_module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _module
_spec.loader.exec_module(_module)
CameraIntrinsics = _module.CameraIntrinsics
HorizontalDiameterEstimator = _module.HorizontalDiameterEstimator

__all__ = ["CameraIntrinsics", "HorizontalDiameterEstimator"]
