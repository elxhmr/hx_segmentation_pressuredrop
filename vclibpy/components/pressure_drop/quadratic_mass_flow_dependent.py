from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .pressure_drop import PressureDrop


@dataclass
class QuadraticMassFlowDependent(PressureDrop):
    """Quadratic mass-flow dependent pressure drop with linear behaviour near zero.

    Python-Port des Modelica-Modells ``QuadraticMassFlowDependent``:

        pressureDrop = b * squareFunction(mdotHydraulic, massFlowLimit)

    mit

        b = pressureDrop_nominal / mdot_nominal**2 * (length / length_nominal).

    Parameter
    ---------
    mdot_nominal : float
        Nominaler Massenstrom ``m_dot,nom`` [kg/s].
    pressureDrop_nominal : float
        Nominaler Druckverlust ``Δp_nom`` [Pa] über ``length_nominal``.
    length_nominal : float
        Nominale Länge ``L_nom`` [m], für die ``pressureDrop_nominal`` gilt.
    massFlowLimit : float, optional
        Schwellwert für die Regularisierung [kg/s]. Standard ist
        ``mdot_nominal/100``.

    Hinweise
    --------
        - Für ``|m_flow| < massFlowLimit`` wird eine geglättete Quadratik
      verwendet, um eine lineare Charakteristik um den Ursprung zu erhalten.
    """

    mdot_nominal: float
    pressureDrop_nominal: float
    length_nominal: float
    massFlowLimit: float | None = None

    name: str = "QuadraticMassFlowDependent"

    def __post_init__(self) -> None:
        if self.mdot_nominal <= 0:
            raise ValueError("mdot_nominal muss > 0 sein.")
        if self.length_nominal <= 0:
            raise ValueError("length_nominal muss > 0 sein.")
        if self.massFlowLimit is None:
            self.massFlowLimit = self.mdot_nominal / 100.0

    @staticmethod
    def _square_function(mdot: float, mass_flow_limit: float) -> float:
        """Glättende Quadratik mit linearem Übergang um ``mass_flow_limit``.

        Annäherung an TIL.Utilities.Numerics.squareFunction:

        - Für |m_dot| >= mass_flow_limit: mdot**2
        - Für |m_dot| <  mass_flow_limit: 2*m_limit*|m_dot| - m_limit**2
        """

        m_abs = abs(mdot)
        m_lim = abs(mass_flow_limit)
        if m_lim <= 0:
            return m_abs * m_abs
        if m_abs >= m_lim:
            return m_abs * m_abs
        return 2.0 * m_lim * m_abs - m_lim * m_lim

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        """Berechne den Druckverlust in Pa für den gegebenen Massenstrom.

        Die Korrelation ist quadratisch in ``m_flow`` und wird um den
        Ursprungsbereich mit ``squareFunction`` regularisiert.
        """

        # b = dp_nom / mdot_nom^2  (Längen-Skalierung erfolgt außerhalb
        # der Korrelation über L_seg / length_nominal)
        b = self.pressureDrop_nominal / (self.mdot_nominal ** 2)

        return b * self._square_function(m_flow, self.massFlowLimit or 0.0)
