from __future__ import annotations

from typing import Any
from .pressure_drop import PressureDrop


class FixedPressureDropPerLength(PressureDrop):
    """Längenabhängige, **flussunabhängige** Δp-Korrelation.

    Definition in diesem Projekt:

    - ``dp_per_length`` ist der Druckverlust über die Referenzlänge ``length``,
      angegeben in Pa. Beispiel: ``dp_per_length = 3000`` und ``length = 5``
      bedeutet **3 kPa über 5 m gesamt**.
    - Der Druckverlust pro Meter ergibt sich dann zu

        Δp/m = dp_per_length / length.

    Für ein Segment mit Länge ``L_seg`` gilt damit

        Δp_seg = (dp_per_length / length) * L_seg

    Diese Segment-Skalierung erfolgt im Wärmetauscher-Code; diese Klasse liefert
    nur ``dp_per_length`` und die zugehörige Referenzlänge ``length``.
    """

    name = "FixedPressureDropPerLength"

    def __init__(
        self,
        dp_per_length: float = 5_000.0,
        length: float = 1.0,
    ) -> None:
        """Initialisiere die Korrelation.

        Parameters
        ----------
        dp_per_length : float, optional
            Druckverlust über die Referenzlänge ``length`` [Pa].
            Beispiel: ``FixedPressureDropPerLength(3000, 5)`` bedeutet
            3_000 Pa über 5 m.
        length : float, optional
            Referenzbauteillänge [m], Standard 1 m.
        """
        if length < 0:
            raise ValueError("length muss >= 0 sein.")
        if dp_per_length < 0:
            raise ValueError("dp_per_length muss >= 0 sein.")
        self.length = float(length)
        self.dp_per_length = float(dp_per_length)

    def calc(self, transport_properties: Any, m_flow: float) -> float:
        """Berechne den Druckverlust in Pa über die Referenzlänge.

        Rückgabewert ist genau ``dp_per_length`` (Druckverlust über ``length``).
        Segmentweise Skalierung mit ``L_seg`` erfolgt im HX-Modell.
        """
        return self.dp_per_length
