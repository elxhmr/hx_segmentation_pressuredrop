# e9_domestic_heat_pump_segmented.py
# Beispiel mit Segmentierung und Druckverlustberechnung

import numpy as np
import matplotlib.pyplot as plt

from vclibpy import Inputs
from vclibpy.flowsheets import StandardCycle
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor

from vclibpy.utils.plotting import plot_cycle
from vclibpy.utils.printing import print_states, print_hx_segmentation

def main():
    """
    Wärmepumpen-Beispiel mit Segmentierung und Druckverlustberechnung.
    
    Im Gegensatz zu e8 werden hier:
    1. Heat Exchanger in Phasen unterteilt (Liquid, Two-Phase, Gas)
    2. Jede Phase wird in mehrere Segmente unterteilt
    """
    
    # --- Komponenten mit Segmentierungs-Konfiguration und Druckverlust ---
    from vclibpy.components.pressure_drop import ConstantPressureDrop
    
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
        use_segmentation=True,  # Standardmäßig aktiviert
        n_segments_sc=3,      # SC in 3 Segmente, Standardwert
        n_segments_lat=5,     # LAT in 5 Segmente, Standardwert
        n_segments_sh=3,      # SH in 3 Segmente, Standardwert
        pressure_drop_model=ConstantPressureDrop()  # Druckverlust-Modell
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
        use_segmentation=True,  # Standardmäßig aktiviert
        n_segments_sc=3,      # SC in 3 Segmente, Standardwert
        n_segments_lat=5,     # LAT in 5 Segmente, Standardwert
        n_segments_sh=3,      # SH in 3 Segmente, Standardwert
        pressure_drop_model=ConstantPressureDrop()  # Druckverlust-Modell
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

    # --- Steady State rechnen mit Segmentierung ---
    print("Berechne Wärmepumpen-Kreislauf mit Segmentierung und Druckverlusten...")
    fs = hp.calc_steady_state(inputs=inputs, show_iteration=False)

    # --- Ergebnisse ausgeben ---
    if fs is not None:
        print("[+] Berechnung erfolgreich\n")
        
        # --- Thermodynamische States ausgeben ---
        print("=" * 90)
        print("THERMODYNAMIC STATES")
        print("=" * 90)
        states_for_plot = hp.get_states_in_order_for_plotting()
        if states_for_plot:
            state_kwargs = {f"state_{i}": state for i, state in enumerate(states_for_plot)}
            print_states(**state_kwargs)
        
        # --- Segment-Informationen aus FlowsheetState auslesen und ausgeben ---
        print_hx_segmentation(fs)

        # --- Diagramme zeichnen (T–h & log(p)–h) - DEAKTIVIERT FÜR SCHNELLE TESTS ---
        # print("\n[*] Plotting cycle diagram...")
        # plot_cycle(
        #     med_prop=hp.med_prop,
        #     states=states_for_plot,
        #     save_path=None,
        #     show=True
        # )
    else:
        print("[!] Berechnung fehlgeschlagen")

    # Aufräumen
    hp.terminate()


if __name__ == "__main__":
    main()
