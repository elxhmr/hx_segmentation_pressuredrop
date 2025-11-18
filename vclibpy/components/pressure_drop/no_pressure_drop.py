from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class NoPressureDrop(PressureDrop):
    """
    Dummy-Korrelation: liefert stets Δp = 0 Pa.
    Praktisch für Tests oder idealisierte Bauteile.
    """

    name = "NoPressureDrop"

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        return 0.0
