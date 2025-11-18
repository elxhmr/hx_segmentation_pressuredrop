#!/usr/bin/env python
"""
Schneller Test für Druckverlustberechnung
"""
import sys
sys.path.insert(0, ".")

from vclibpy import Inputs
from vclibpy.flowsheets import StandardCycle
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.components.pressure_drop import ConstantPressureDrop
from vclibpy.utils.printing import print_states, print_hx_segmentation

print("Creating heat pump with pressure drops...")

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
    use_segmentation=True,
    n_segments_sc=3,
    n_segments_lat=5,
    n_segments_sh=3,
    pressure_drop_model=ConstantPressureDrop()
)

evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=15.0,
    secondary_medium="air",
    flow_type="counter",
    ratio_outer_to_inner_area=10.0,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25),
    use_segmentation=True,
    n_segments_sc=3,
    n_segments_lat=5,
    n_segments_sh=3,
    pressure_drop_model=ConstantPressureDrop()
)

compressor = RotaryCompressor(N_max=125, V_h=19e-6)

expansion_valve = Bernoulli(A=0.1)

print("Initializing StandardCycle...")

hp = StandardCycle(
    compressor=compressor,
    condenser=condenser,
    evaporator=evaporator,
    expansion_valve=expansion_valve,
    fluid="Propane"
)

print("Setting inputs...")
inputs = Inputs(
    n=0.7,
    T_eva_in=273.15,
    T_con_in=323.15,
    m_flow_eva=0.9,
    m_flow_con=0.2,
    dT_eva_superheating=5.0,
    dT_con_subcooling=5.0,
)

print("Calculating steady state...")
fs_state = hp.calc_steady_state(
    inputs=inputs,
    show_iteration=False,
    max_num_iterations=50
)

print("\n[+] SUCCESS - HP calculated successfully!")
if fs_state is not None:
    print_states()
    print_hx_segmentation()
else:
    print("[!] Calculation failed")

# Print pressure drops
try:
    eva_dp = fs_state.get(name="Evaporator_total_dp").value
    con_dp = fs_state.get(name="Condenser_total_dp").value
    print(f"\nPressure Drops:")
    print(f"  Evaporator: {eva_dp/1000:.2f} kPa")
    print(f"  Condenser: {con_dp/1000:.2f} kPa")
except Exception as e:
    print(f"Could not get pressure drops: {e}")

hp.terminate()
print("\n[*] Test completed successfully!")
