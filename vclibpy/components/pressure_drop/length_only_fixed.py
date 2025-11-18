from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class FixedGradientPressureDrop(PressureDrop):
    """
    Länge-abhängige, **flussunabhängige** Δp-Korrelation mit festem Druckgefälle:

        Δp = (grad_pa_per_m) * L

    Standard: 0.05 bar pro Meter => 0.05 * 1e5 Pa/m = 5_000 Pa/m.

    - grad_pa_per_m: [Pa/m]
    - L: [m]
    """

    name = "FixedGradientPressureDrop"

    def __init__(self, L: float, grad_pa_per_m: float = 5_000.0) -> None:
        """
        Parameters
        ----------
        L : float
            Bauteillänge [m].
        grad_pa_per_m : float, optional
            Festes Druckgefälle [Pa/m], default 5_000 Pa/m (0.05 bar/m).
        """
        if L < 0:
            raise ValueError("L muss >= 0 sein.")
        if grad_pa_per_m < 0:
            raise ValueError("grad_pa_per_m muss >= 0 sein.")
        self.L = float(L)
        self.grad_pa_per_m = float(grad_pa_per_m)

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        # m_flow wird absichtlich ignoriert (modellunabhängig von Durchfluss).
        return self.grad_pa_per_m * self.L
