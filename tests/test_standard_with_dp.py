import pytest

from vclibpy import Inputs
from vclibpy.components import heat_transfer
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.heat_exchangers import SegmentationConfig, moving_boundary_ntu
from vclibpy.components.pressure_drop.quadratic_mass_flow_dependent import QuadraticMassFlowDependent
from vclibpy.datamodels import FlowsheetState
from vclibpy.flowsheets import StandardCycleWithDP


def _build_cycle(segmentation=None, dp_model=None):
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=2000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        segmentation=segmentation,
        dp_model=dp_model,
    )

    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=10,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1500),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=200),
        segmentation=segmentation,
        dp_model=dp_model,
    )

    compressor = RotaryCompressor(N_max=125, V_h=19e-6)
    expansion_valve = Bernoulli(A=0.1)

    hp = StandardCycleWithDP(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )

    return hp


def _example_inputs():
    return Inputs(
        n=0.7,
        T_eva_in=273.15,
        T_con_in=323.15,
        m_flow_eva=0.9,
        m_flow_con=0.2,
        dT_eva_superheating=5.0,
        dT_con_subcooling=5.0,
    )


def _single_pass_cycle(hp: StandardCycleWithDP, inputs: Inputs):
    hp.setup_new_fluid(hp.fluid)
    fs_state = FlowsheetState()
    T_1_start = inputs.T_eva_in - inputs.dT_eva_superheating
    T_3_start = inputs.T_con_in + inputs.dT_con_subcooling
    p_1 = hp.med_prop.calc_state("TQ", T_1_start, 1).p
    p_2 = hp.med_prop.calc_state("TQ", T_3_start, 0).p

    hp.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
    hp.evaporator.calc(inputs=inputs, fs_state=fs_state)
    hp.condenser.calc(inputs=inputs, fs_state=fs_state)
    return fs_state, p_1, p_2


def test_standard_with_dp_legacy_path_matches_port_pressures():
    hp = _build_cycle()
    fs, p_1, p_2 = _single_pass_cycle(hp, _example_inputs())
    hp.calc_states(p_1, p_2, inputs=_example_inputs(), fs_state=fs)

    assert fs is not None
    assert fs.get("p_con_in").value == pytest.approx(fs.get("p_con_out").value)
    assert fs.get("p_eva_in").value == pytest.approx(fs.get("p_eva_out").value)

    hp.terminate()


def test_standard_with_dp_segmentation_runs_without_dp_model():
    segmentation = SegmentationConfig()
    hp = _build_cycle(segmentation=segmentation)
    fs, p_1, p_2 = _single_pass_cycle(hp, _example_inputs())
    hp.calc_states(p_1, p_2, inputs=_example_inputs(), fs_state=fs)

    assert fs is not None
    assert fs.get("dp_con_total").value == pytest.approx(0)
    assert fs.get("dp_eva_total").value == pytest.approx(0)

    hp.terminate()


def test_standard_with_dp_propagates_pressure_drop():
    segmentation = SegmentationConfig(N_sc=1, N_lat=4, N_sh=1)
    dp_model = QuadraticMassFlowDependent(
        mdot_nominal=0.01,
        pressureDrop_nominal=2000,
        length_nominal=5,
    )
    hp = _build_cycle(segmentation=segmentation, dp_model=dp_model)
    fs, p_1, p_2 = _single_pass_cycle(hp, _example_inputs())
    hp.calc_states(p_1, p_2, inputs=_example_inputs(), fs_state=fs)

    dp_con_total = fs.get("dp_con_total").value
    dp_eva_total = fs.get("dp_eva_total").value
    assert dp_con_total > 0
    assert dp_eva_total > 0

    assert fs.get("p_con_in").value == pytest.approx(fs.get("p_con_out").value + dp_con_total, rel=1e-2)
    assert fs.get("p_eva_in").value == pytest.approx(fs.get("p_eva_out").value + dp_eva_total, rel=1e-2)

    hp.terminate()
