from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class ConstantPressureDrop(PressureDrop):
    """
    Dummy-Korrelation: liefert stets Δp = 10_000 Pa.
    Praktisch für Tests.
    """

    name = "ConstantPressureDrop"

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        return 3_000.0
