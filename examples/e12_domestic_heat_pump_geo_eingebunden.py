# e10_domestic_heat_pump_dp.py
# Beispiel mit Segmentierung UND Druckverlustberechnung mit Plot

import numpy as np
import matplotlib.pyplot as plt
import logging

from vclibpy import Inputs
from vclibpy.flowsheets import StandardCycle
from vclibpy.components.heat_exchangers import moving_boundary_ntu, hx_model
from vclibpy.components import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor

from vclibpy.utils.plotting import plot_cycle_for_flowsheet, show_in_hx, show_in_hx2, show_hx_temperature_profiles
from vclibpy.utils.printing import print_all_states, print_inlet_outlet_comp
from vclibpy.components.pressure_drop import ConstantPressureDrop

def main():
    """
    Wärmepumpen-Beispiel mit Segmentierung und Druckverlustberechnung.
    
    Im Gegensatz zu e8 werden hier:
    1. Heat Exchanger in Phasen unterteilt (Liquid, Two-Phase, Gas)
    2. Jede Phase wird in mehrere Segmente unterteilt
    """

    
    # --- Komponenten mit Segmentierungs-Konfiguration und phasenspezifischen Druckverlusten ---
    geom_condenser = hx_model.load_geometry("plate_demo")
    geom_evaporator = hx_model.load_geometry("fin_tube_demo")

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        geometry="plate_demo",
        secondary_medium="water",
        flow_type="counter",
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(
            lambda_=236,
            thickness=geom_condenser.wall_thickness_m or 2e-3,
        ),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        use_segmentation=True,  # Standardmäßig aktiviert
        n_segments_sc=10,      # SC in 3 Segmente, Standardwert
        n_segments_lat=5,     # LAT in 5 Segmente, Standardwert
        n_segments_sh=3,      # SH in 3 Segmente, Standardwert
        # Phasenspezifische Druckverlust-Korrelationen (hier alle konstant als Beispiel)
        two_phase_pressure_drop=ConstantPressureDrop(dp=3000),
        gas_pressure_drop=ConstantPressureDrop(dp=2000),
        liquid_pressure_drop=ConstantPressureDrop(dp=2000),
    )
    
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        geometry="fin_tube_demo",
        secondary_medium="air",
        flow_type="counter",
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(
            lambda_=236,
            thickness=geom_evaporator.wall_thickness_m or 2e-3,
        ),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25),
        use_segmentation=True,  # Standardmäßig aktiviert
        n_segments_sc=3,      # SC in 3 Segmente, Standardwert
        n_segments_lat=5,     # LAT in 5 Segmente, Standardwert
        n_segments_sh=3,      # SH in 3 Segmente, Standardwert
        # Phasenspezifische Druckverlust-Korrelationen (hier alle konstant als Beispiel)
        two_phase_pressure_drop=ConstantPressureDrop(dp=3000),
        gas_pressure_drop=ConstantPressureDrop(dp=2000),
        liquid_pressure_drop=ConstantPressureDrop(dp=2000),
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
        dT_eva_superheating=0.0,  # 5 K Überhitzung
        dT_con_subcooling=0.0,    # 0 K Unterkühlung
    )

    # --- Logging für temporäre Outer-Loop-Ausgabe aktivieren ---
    logging.basicConfig(level=logging.INFO, format="[vclibpy] %(message)s")

    # --- Steady State rechnen mit Segmentierung ---
    print("Berechne Wärmepumpen-Kreislauf mit Segmentierung und Druckverlusten...")
    fs = hp.calc_steady_state(inputs=inputs, show_iteration=False)

    # --- Ergebnisse ausgeben ---
    if fs is not None:
        print("[+] Berechnung erfolgreich\n")
        # --- States (Tabelle) und Segmentierung in einem Rutsch ---
        states_for_plot = hp.get_states_in_order_for_plotting()
        # Einzigartige Zustände + separate Komponenten-IO Tabellen
        print_all_states(fs, states=states_for_plot)
        print_inlet_outlet_comp(hp)

        # --- Diagramme zeichnen (T–h & log(p)–h) – Automatisch aus Flowsheet/FS-State ---
        print("\n[*] Plotting cycle diagram...")
        plot_cycle_for_flowsheet(
            flowsheet=hp,
            fs_state=fs,
            show_segments_overlay=False,  # bei Bedarf True setzen
            show=True,
        )

        # --- HX-Detaildarstellung über Fläche (beide HX übereinander) ---
        print("\n[*] Zeige HX-Detailplots über Fläche...")
        show_in_hx(
            flowsheet=hp,
            fs_state=fs,
            inputs=inputs,
            show=True,
        )
        
        # --- Temperaturverläufe in beiden Wärmeübertragern ---
        print("\n[*] Zeige Temperaturverläufe (Kältemittel & Sekundärmedium)...")
        show_hx_temperature_profiles(
            flowsheet=hp,
            fs_state=fs,
            inputs=inputs,
            show=True,
        )
    else:
        print("[!] Berechnung fehlgeschlagen")

    # Aufräumen
    hp.terminate()


if __name__ == "__main__":
    main()


    #geometrie für Verdampfer und Kondensator hinzufügen (wie Bibliothek) für bessere Alpha und Dp
    #bessere DP Korrelationen hinzufügen
    #debug ob alles passt
    #rohre hinzufügen (wärmeübergang + druckverluste)
    #segmentierung dynamisch anpassen
