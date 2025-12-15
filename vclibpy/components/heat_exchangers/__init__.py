from .heat_exchanger import HeatExchanger
from .moving_boundary_ntu import MovingBoundaryNTUCondenser, MovingBoundaryNTUEvaporator
from .segmentation import SegmentationConfig

# Backwards compatibility alias: heat transfer correlations now live in
# ``vclibpy.components.heat_transfer``. Importing them via the heat_exchangers
# namespace still works for legacy callers and documentation examples.
from vclibpy.components import heat_transfer  # noqa: E402
