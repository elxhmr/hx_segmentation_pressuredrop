from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class ConstantPressureDrop(PressureDrop):
    """Einfaches Modell mit konstantem Druckverlust.

    Standardmäßig wird ein fixer Δp in Pascal verwendet, kann aber
    über den Parameter ``dp`` angepasst werden.

    Beispiel::

        ConstantPressureDrop()          # nutzt Standardwert 3000 Pa
        ConstantPressureDrop(dp=5000)   # nutzt 5000 Pa
    """

    name = "ConstantPressureDrop"

    def __init__(self, dp: float = 3_000.0) -> None:
        self.dp = float(dp)

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        # Gibt immer denselben Betrag Δp in Pa zurück (unabhängig von m_flow).
        return abs(self.dp)
