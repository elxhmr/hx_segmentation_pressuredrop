"""Legacy heat transfer package kept for backward compatibility.

This package forwards imports to the new ``vclibpy.components.heat_transfer``
package so existing code (e.g., examples e1-e7) can keep running unchanged.
"""
from importlib import import_module
import sys
from types import ModuleType

_TARGET_PACKAGE = "vclibpy.components.heat_transfer"
_target_module = import_module(_TARGET_PACKAGE)

HeatTransfer = getattr(_target_module, "HeatTransfer")
TwoPhaseHeatTransfer = getattr(_target_module, "TwoPhaseHeatTransfer")
calc_reynolds_pipe = getattr(_target_module, "calc_reynolds_pipe")

__all__ = [
    "HeatTransfer",
    "TwoPhaseHeatTransfer",
    "calc_reynolds_pipe",
]

_SUBMODULE_REDIRECTS = {
    "constant": f"{_TARGET_PACKAGE}.constant",
    "wall": f"{_TARGET_PACKAGE}.wall",
    "air_to_wall": f"{_TARGET_PACKAGE}.air_to_wall",
    "pipe_to_wall": f"{_TARGET_PACKAGE}.pipe_to_wall",
    "vdi_atlas_air_to_wall": f"{_TARGET_PACKAGE}.vdi_atlas_air_to_wall",
    "heat_transfer": f"{_TARGET_PACKAGE}.heat_transfer",
}

for legacy_name, target_path in _SUBMODULE_REDIRECTS.items():
    module_name = f"{__name__}.{legacy_name}"
    module: ModuleType = import_module(target_path)
    sys.modules[module_name] = module
    globals()[legacy_name] = module
