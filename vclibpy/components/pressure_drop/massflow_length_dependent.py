from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class LinearMassFlowPressureDrop(PressureDrop):
    """
    Sehr einfache Δp-Korrelation: linear in Massenstrom UND Länge.

        Δp = k * |m_flow| * L

    - k:   [Pa / (kg/s) / m]  (Druckgefälle pro kg/s und pro Meter)
    - L:   [m]
    - m_flow: [kg/s]

    Hinweise:
    - abs(m_flow): Druckverlust ist per Definition positiv.
    - 'transport_properties' wird nicht verwendet, ist aber für API-Gleichheit vorhanden.
    - Sinnvoll als Platzhalter/Kalibrier-Modell, bis komplexere Korrelationen (z.B. Darcy-Weisbach)
      mit Reibungszahl eingeführt werden.
    """

    name = "LinearMassFlowPressureDrop"

    def __init__(self, L: float, k_pa_per_kgps_per_m: float = 1000.0) -> None:
        """
        Parameters
        ----------
        L : float
            Bauteillänge [m].
        k_pa_per_kgps_per_m : float, optional
            Proportionalitätskonstante k [Pa/(kg/s)/m], default 1000.0.
        """
        if L < 0:
            raise ValueError("L muss >= 0 sein.")
        if k_pa_per_kgps_per_m < 0:
            raise ValueError("k muss >= 0 sein.")
        self.L = float(L)
        self.k = float(k_pa_per_kgps_per_m)

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        return self.k * abs(m_flow) * self.L
