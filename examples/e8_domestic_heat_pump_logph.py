# e7_logph_only.py
# Minimalbeispiel: Standard-WP rechnen und NUR log(p)-h-Diagramm anzeigen.

import numpy as np
import matplotlib.pyplot as plt

from vclibpy import Inputs
from vclibpy.flowsheets import StandardCycle
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor

from vclibpy.utils.plotting import plot_cycle


def main():
    # --- Komponenten wie in e6: einfache, konstante HTCs usw. ---
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5.0,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1.0,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15.0,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10.0,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25),
    )
    expansion_valve = Bernoulli(A=0.1)
    compressor = RotaryCompressor(N_max=125, V_h=19e-6)

    hp = StandardCycle(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )

    # --- Ein (einziger) Arbeitspunkt ---
    inputs = Inputs(
        n=0.7,
        T_eva_in=273.15,          # 0 °C
        T_con_in=323.15,          # 50 °C
        m_flow_eva=0.9,
        m_flow_con=0.2,
        dT_eva_superheating=5.0,  # 5 K Überhitzung
        dT_con_subcooling=5.0,    # 0 K Unterkühlung
    )

    # --- Steady State rechnen ---
    fs = hp.calc_steady_state(inputs=inputs)  # nutzt intern StandardCycle-Logik

    # --- Diagramme zeichnen (T–h & log(p)–h) – automatisch aus Flowsheet/FS-State ---
    from vclibpy.utils.plotting import plot_cycle_for_flowsheet
    plot_cycle_for_flowsheet(
        flowsheet=hp,
        fs_state=fs,
        show_segments_overlay=False,
        show=True,
    )

    # Aufräumen (CoolProp/RefProp-Wrapper beenden)
    hp.terminate()


if __name__ == "__main__":
    main()
