"""Demonstrate the StandardCycleWithDP API with segmentation configuration.

This example keeps the numerical behaviour identical to the legacy path while
showing how a segmentation configuration can be provided to the heat
exchangers. Pressure-drop physics are not yet active; the example relies on the
existing moving-boundary NTU calculations.
"""

import logging

from vclibpy import Inputs
from vclibpy.flowsheets import StandardCycleWithDP
from vclibpy.components.heat_exchangers import SegmentationConfig, moving_boundary_ntu
from vclibpy.components import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor

from vclibpy.utils.printing import print_all_states, print_inlet_outlet_comp


def main():
    segmentation = SegmentationConfig()  # Defaults to (1, 1, 1) subsegments

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        segmentation=segmentation,
    )

    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=100),
        segmentation=segmentation,
    )

    expansion_valve = Bernoulli(A=0.1)
    compressor = RotaryCompressor(N_max=125, V_h=19e-6)

    hp = StandardCycleWithDP(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )

    inputs = Inputs(
        n=0.7,
        T_eva_in=273.15,
        T_con_in=323.15,
        m_flow_eva=0.9,
        m_flow_con=0.2,
        dT_eva_superheating=5.0,
        dT_con_subcooling=5.0,
    )

    logging.basicConfig(level=logging.INFO, format="[vclibpy] %(message)s")

    print("Berechne Wärmepumpen-Kreislauf mit Segmentierungskonfiguration (Legacy-Berechnung)...")
    fs = hp.calc_steady_state(inputs=inputs, show_iteration=False)

    if fs is not None:
        print("[+] Berechnung erfolgreich\n")
        states_for_plot = hp.get_states_in_order_for_plotting()
        print_all_states(fs, states=states_for_plot)
        print_inlet_outlet_comp(hp)
    else:
        print("[!] Berechnung fehlgeschlagen")

    hp.terminate()


if __name__ == "__main__":
    main()
