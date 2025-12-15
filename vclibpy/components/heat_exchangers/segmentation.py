"""Configuration helpers for future segmented heat exchanger modes.

This module intentionally keeps the configuration lean because segmented
heat-exchanger physics are not implemented yet. Providing a dedicated
configuration object already allows callers to declare their intent without
changing the legacy moving-boundary NTU calculations.
"""

from dataclasses import dataclass
from typing import Optional

from vclibpy.media import ThermodynamicState


@dataclass(frozen=True)
class SegmentationConfig:
    """User-facing configuration for segmented heat exchanger calculations.

    The legacy moving-boundary NTU calculation remains the default. Passing a
    ``SegmentationConfig`` declares the caller's intent to run a segmented path
    once it is available while keeping the current legacy results intact.

    Args:
        N_sc: Number of subsegments in the subcooled region. Defaults to ``1``
            when not provided.
        N_lat: Number of subsegments in the two-phase (latent) region. Defaults
            to ``1`` when not provided.
        N_sh: Number of subsegments in the superheated region. Defaults to
            ``1`` when not provided.
        tol: Convergence tolerance for future segment-level iterations. The
            value is stored only; no new solver logic is triggered yet.
        max_iter: Maximum number of iterations for future segment-level
            corrector steps. Stored only, not used yet.
        relaxation: Relaxation factor for future iterations. Stored only, not
            used yet.
        enabled: Whether segmented mode should be considered active. When set
            to ``False`` the configuration is ignored and the legacy mode is
            used.
        name: Optional identifier that downstream tooling can use for
            diagnostics or logging. It does not influence calculations.
    """

    N_sc: int = 1
    N_lat: int = 1
    N_sh: int = 1
    tol: float = 1e-6
    max_iter: int = 50
    relaxation: float = 1.0
    enabled: bool = True
    name: Optional[str] = None

    def __post_init__(self):
        if self.N_sc <= 0 or self.N_lat <= 0 or self.N_sh <= 0:
            raise ValueError("Segment counts N_sc, N_lat, and N_sh must be positive integers")
        if self.max_iter <= 0:
            raise ValueError("max_iter must be positive")
        if self.tol <= 0:
            raise ValueError("tol must be positive")
        if self.relaxation <= 0:
            raise ValueError("relaxation must be positive")

    @property
    def counts(self) -> tuple[int, int, int]:
        """Return the (N_sc, N_lat, N_sh) triple for convenience."""

        return self.N_sc, self.N_lat, self.N_sh


@dataclass
class SegmentResult:
    """Result container for a single segmented heat-exchanger slice."""

    phase: str
    segment_index: int
    total_segments: int
    Q: float
    dT_max: float
    dT_min: float
    state_inlet: ThermodynamicState
    state_outlet: ThermodynamicState
    p_inlet: float
    p_outlet: float
    total_phase_heat: float = 0.0

    def get_segment_fraction(self) -> float:
        """Return the share of phase heat this segment accounts for."""

        if self.total_phase_heat == 0:
            return 0.0
        return self.Q / self.total_phase_heat

