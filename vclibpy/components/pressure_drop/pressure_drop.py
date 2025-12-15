from __future__ import annotations

import abc
from typing import Any


class PressureDrop(abc.ABC):
    """
    Abstrakte Basis für Druckverlust-Modelle.

    Konventionen:
    - calc(...) gibt den gesamten Druckverlust Δp in Pascal (Pa) zurück, immer positiv.
    - Einheit m_flow: kg/s (Massenstrom)
    - transport_properties ist optional (z.B. für spätere Modelle); darf ignoriert werden.

    Implementierungen sollten **keine** Seiten­effekte haben und rein deterministisch sein.
    """

    name: str = "BasePressureDrop"

    @abc.abstractmethod
    def calc(self, transport_properties: Any, m_flow: float) -> float:
        """
        Berechne Δp [Pa] für gegebenen Zustand/Massenstrom.
        'transport_properties' kann None sein, wenn das Modell sie nicht benötigt.
        """
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}>"
