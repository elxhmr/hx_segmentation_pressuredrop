import pytest

from vclibpy import FlowsheetState, Inputs
from vclibpy.components import heat_transfer
from vclibpy.components.heat_exchangers import SegmentationConfig, moving_boundary_ntu
from vclibpy.components.heat_exchangers.hx_model import load_geometry
from vclibpy.components.pressure_drop.quadratic_mass_flow_dependent import QuadraticMassFlowDependent
from vclibpy.media import CoolProp


@pytest.fixture(scope="module")
def med_prop():
    return CoolProp(fluid_name="Propane")


def _evaporator_example(med_prop, segmentation=None, dp_model=None):
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25),
        segmentation=segmentation,
        dp_model=dp_model,
    )
    evaporator.med_prop = med_prop
    evaporator.start_secondary_med_prop()
    evaporator.m_flow = 0.01

    T_eva_in = 273.15 + 2
    dT_eva_superheating = 10
    inputs = Inputs(
        T_eva_in=T_eva_in,
        m_flow_eva=0.47,
        dT_eva_superheating=dT_eva_superheating,
        dT_con_subcooling=0,
    )

    p_evaporation = med_prop.calc_state("TQ", T_eva_in - dT_eva_superheating, 1).p
    state_condenser_outlet = med_prop.calc_state("TQ", 273.15 + 40, 0)
    evaporator.state_inlet = med_prop.calc_state("PH", p_evaporation, state_condenser_outlet.h)
    T_evaporation = med_prop.calc_state("PQ", p_evaporation, 1).T
    evaporator.state_outlet = med_prop.calc_state(
        "PT", p_evaporation, T_evaporation + dT_eva_superheating
    )
    return evaporator, inputs


def _condenser_example(med_prop, *, segmentation=None, dp_model=None):
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        segmentation=segmentation,
        dp_model=dp_model,
    )
    condenser.med_prop = med_prop
    condenser.start_secondary_med_prop()
    condenser.m_flow = 0.01

    T_con_in = 273.15 + 30
    inputs = Inputs(T_con_in=T_con_in, m_flow_con=0.2, dT_con_subcooling=5)

    p_cond = med_prop.calc_state("TQ", 273.15 + 40, 0).p
    condenser.state_outlet = med_prop.calc_state("PT", p_cond, (273.15 + 40) - inputs.dT_con_subcooling)
    condenser.state_inlet = med_prop.calc_state("PT", p_cond, (273.15 + 40) + 10)
    return condenser, inputs


class _ConstantDP:
    def __init__(self, value: float):
        self.value = value

    def calc(self, transport_properties, m_flow):
        return self.value


def test_evaporator_legacy_regression(med_prop):
    evaporator, inputs = _evaporator_example(med_prop)
    error, dT_min = evaporator.calc(inputs=inputs, fs_state=FlowsheetState())

    assert error == pytest.approx(-93.90527369110305)
    assert dT_min == pytest.approx(-5.684341886080802e-14)
    assert evaporator.calc_Q_flow() == pytest.approx(2755.7053929305007)
    assert evaporator.uses_legacy_mode()


def test_condenser_legacy_regression(med_prop):
    condenser, inputs = _condenser_example(med_prop)
    error, dT_min = condenser.calc(inputs=inputs, fs_state=FlowsheetState())

    assert error == pytest.approx(3.97533564430117)
    assert dT_min == pytest.approx(5.0)
    assert condenser.calc_Q_flow() == pytest.approx(3436.210341369809)
    assert condenser.uses_legacy_mode()


def test_segmented_evaporator_energy_balance(med_prop):
    segmentation = SegmentationConfig(N_sc=1, N_lat=5, N_sh=2)
    evaporator, inputs = _evaporator_example(med_prop, segmentation=segmentation)
    fs_state = FlowsheetState()

    error, dT_min = evaporator.calc(inputs=inputs, fs_state=fs_state)

    segments_sc = fs_state.get("segments_eva_sc").value
    segments_lat = fs_state.get("segments_eva_lat").value
    segments_sh = fs_state.get("segments_eva_sh").value
    all_segments = segments_sc + segments_lat + segments_sh

    enthalpies = [seg.state_inlet.h for seg in all_segments] + [all_segments[-1].state_outlet.h]
    assert enthalpies == sorted(enthalpies)

    Q_segments = sum(seg.Q for seg in all_segments)
    assert Q_segments == pytest.approx(
        evaporator.m_flow * (evaporator.state_outlet.h - evaporator.state_inlet.h), rel=1e-3
    )

    min_dT_segments = min(seg.dT_min for seg in all_segments)
    assert dT_min == pytest.approx(min_dT_segments, rel=1e-3)
    assert abs(error) < 5


def test_segmented_config_defaults_to_legacy_results(med_prop):
    segmentation = SegmentationConfig()
    condenser, inputs = _condenser_example(med_prop, segmentation=segmentation)

    error, dT_min = condenser.calc(inputs=inputs, fs_state=FlowsheetState())

    assert condenser.uses_segmented_mode()
    assert not condenser.uses_legacy_mode()
    assert segmentation.counts == (1, 1, 1)
    # Segmented (1,1,1) should closely match legacy outputs
    assert abs(error) < 5
    assert dT_min == pytest.approx(5.0, rel=1e-3)
    assert condenser.calc_Q_flow() == pytest.approx(3436.210341369809, rel=1e-3)


def test_segmented_condenser_profiles(med_prop):
    segmentation = SegmentationConfig(N_sc=1, N_lat=5, N_sh=1)
    condenser, inputs = _condenser_example(med_prop, segmentation=segmentation)
    fs_state = FlowsheetState()

    error, dT_min = condenser.calc(inputs=inputs, fs_state=fs_state)

    segments_lat = fs_state.get("segments_con_lat").value
    segments_sc = fs_state.get("segments_con_sc").value
    segments_sh = fs_state.get("segments_con_sh").value

    all_segments = segments_sc + segments_lat + segments_sh

    # Enthalpy should increase monotonically along the condenser path
    enthalpies = [seg.state_inlet.h for seg in all_segments] + [all_segments[-1].state_outlet.h]
    assert enthalpies == sorted(enthalpies)

    # Energy balance
    Q_segments = sum(seg.Q for seg in all_segments)
    assert Q_segments == pytest.approx(
        condenser.m_flow * (condenser.state_inlet.h - condenser.state_outlet.h), rel=1e-3
    )

    # Pinch consistency
    min_dT_segments = min(seg.dT_min for seg in all_segments)
    assert dT_min == pytest.approx(min_dT_segments, rel=1e-3)
    assert abs(error) < 5


def test_segmented_condenser_pressure_drop_profile(med_prop):
    segmentation = SegmentationConfig(N_sc=1, N_lat=4, N_sh=1)
    dp_model = QuadraticMassFlowDependent(
        mdot_nominal=0.01,
        pressureDrop_nominal=2000,
        length_nominal=5,
    )
    condenser, inputs = _condenser_example(med_prop, segmentation=segmentation, dp_model=dp_model)
    fs_state = FlowsheetState()

    _, _ = condenser.calc(inputs=inputs, fs_state=fs_state)

    segments_lat = fs_state.get("segments_con_lat").value
    segments_sc = fs_state.get("segments_con_sc").value
    segments_sh = fs_state.get("segments_con_sh").value
    all_segments = segments_sc + segments_lat + segments_sh

    pressures = [seg.p_inlet for seg in all_segments] + [all_segments[-1].p_outlet]
    assert all(p > 0 for p in pressures)
    assert pressures == sorted(pressures, reverse=True)

    dp_total = fs_state.get("dp_con_total").value
    summed_dp = sum(seg.p_inlet - seg.p_outlet for seg in all_segments)
    assert dp_total == pytest.approx(summed_dp)
    assert dp_total > 0


def test_segmented_pressure_drop_disabled_by_default(med_prop):
    segmentation = SegmentationConfig(N_sc=1, N_lat=3, N_sh=1)
    condenser, inputs = _condenser_example(med_prop, segmentation=segmentation)
    fs_state = FlowsheetState()

    _, _ = condenser.calc(inputs=inputs, fs_state=fs_state)

    assert fs_state.get("dp_con_total").value == pytest.approx(0)


def test_geometry_sets_area_and_ratio():
    geometry = load_geometry("fin_tube_demo")

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        geometry=geometry,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=None,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    )

    assert condenser.A == pytest.approx(geometry.outer_area_m2)
    expected_ratio = geometry.outer_area_m2 / geometry.inner_area_m2
    assert condenser.ratio_outer_to_inner_area == pytest.approx(expected_ratio)


def test_phase_specific_pressure_drop_models(med_prop):
    segmentation = SegmentationConfig(N_sc=1, N_lat=1, N_sh=1)
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        segmentation=segmentation,
        liquid_pressure_drop=_ConstantDP(100.0),
        gas_pressure_drop=_ConstantDP(0.0),
        two_phase_pressure_drop=_ConstantDP(0.0),
    )
    condenser.med_prop = med_prop
    condenser.start_secondary_med_prop()
    condenser.m_flow = 0.01

    T_con_in = 273.15 + 30
    inputs = Inputs(T_con_in=T_con_in, m_flow_con=0.2, dT_con_subcooling=5)

    p_cond = med_prop.calc_state("TQ", 273.15 + 40, 0).p
    condenser.state_outlet = med_prop.calc_state("PT", p_cond, (273.15 + 40) - inputs.dT_con_subcooling)
    condenser.state_inlet = med_prop.calc_state("PT", p_cond, (273.15 + 40) + 10)

    fs_state = FlowsheetState()
    _, _ = condenser.calc(inputs=inputs, fs_state=fs_state)
    segments_sc = fs_state.get("segments_con_sc").value
    segments_lat = fs_state.get("segments_con_lat").value
    segments_sh = fs_state.get("segments_con_sh").value

    assert any(seg.p_inlet > seg.p_outlet for seg in segments_sc)
    assert all(seg.p_inlet == pytest.approx(seg.p_outlet) for seg in segments_lat)
    assert all(seg.p_inlet == pytest.approx(seg.p_outlet) for seg in segments_sh)
