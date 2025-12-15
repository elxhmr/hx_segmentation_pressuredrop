import math

import pytest

from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.flowsheets.standardWITHdp import StandardCycleWITHdp
from vclibpy.media.states import ThermodynamicState
from vclibpy.components.heat_exchangers.heat_exchanger import HeatExchanger
from vclibpy.components.heat_exchangers.moving_boundary_ntu import MovingBoundaryNTU


class DummyMedProp:
    def __init__(self):
        self.fluid_name = "TEST"

    def calc_state(self, mode, a, b):
        if mode == "PT":
            return ThermodynamicState(p=a, T=b, h=a + b)
        if mode == "PQ":
            return ThermodynamicState(p=a, T=250 + 10 * b, h=a + b)
        if mode == "PH":
            return ThermodynamicState(p=a, h=b, T=b / 1000)
        raise ValueError("unsupported mode")

    def calc_mean_transport_properties(self, *_args, **_kwargs):
        return None

    def get_critical_point(self):
        return None, 1e7, None

    def terminate(self):
        return None


class DummyPressureDropModel:
    def __init__(self, dp):
        self.dp = dp
        self.length_nominal = 1.0

    def calc(self, _state, _m_flow):
        return self.dp


class DummyCompressor:
    def __init__(self):
        self.state_inlet = None
        self.state_outlet = None
        self.m_flow = 0.1
        self.med_prop = DummyMedProp()

    def calc_state_outlet(self, p_outlet, inputs, fs_state):
        self.state_outlet = self.med_prop.calc_state(
            "PH", p_outlet, getattr(self.state_inlet, "h", 0) + 10000
        )

    def calc_m_flow(self, inputs, fs_state):
        self.m_flow = 0.1

    def calc_electrical_power(self, inputs, fs_state):
        return 1000.0


class DummyExpansionValve:
    def __init__(self):
        self.state_inlet = None
        self.state_outlet = None
        self.m_flow = 0.1
        self.med_prop = DummyMedProp()

    def calc_outlet(self, p_outlet):
        self.state_outlet = self.med_prop.calc_state("PH", p_outlet, self.state_inlet.h)

    def calc_opening_at_m_flow(self, m_flow):
        return 0.5


class DummyHX(HeatExchanger):
    def __init__(self, dp_model=None, **kwargs):
        super().__init__(secondary_medium="water", **kwargs)
        self.pressure_drop_model = dp_model
        self.apply_pressure_drops = dp_model is not None
        self.m_flow = 0.1
        self.med_prop = DummyMedProp()

    def calc(self, inputs, fs_state):
        dp = 0.0
        if self.apply_pressure_drops and self.pressure_drop_model:
            dp = self.pressure_drop_model.calc(None, self.m_flow)
        if self.state_outlet is not None:
            self.state_outlet = self.med_prop.calc_state(
                "PH", max(self.state_outlet.p - dp, 1000), self.state_outlet.h
            )
        fs_state.set(
            name=f"{self.__class__.__name__}_total_dp",
            value=dp,
            unit="Pa",
            description="dummy dp",
        )
        return 0.0, 5.0

    def calc_Q_flow(self):
        return 0.0

    def calc_secondary_Q_flow(self, _Q):
        return 0.0

    def calc_transport_properties_secondary_medium(self, _T):
        return None


def _build_cycle(dp_value=None):
    dp_model = DummyPressureDropModel(dp_value) if dp_value is not None else None
    evaporator = DummyHX(
        dp_model=dp_model,
        A=1.0,
        wall_heat_transfer=None,
        secondary_heat_transfer=None,
        gas_heat_transfer=None,
        liquid_heat_transfer=None,
        two_phase_heat_transfer=None,
    )
    condenser = DummyHX(
        dp_model=dp_model,
        A=1.0,
        wall_heat_transfer=None,
        secondary_heat_transfer=None,
        gas_heat_transfer=None,
        liquid_heat_transfer=None,
        two_phase_heat_transfer=None,
    )
    compressor = DummyCompressor()
    expansion_valve = DummyExpansionValve()
    cycle = StandardCycleWITHdp(
        fluid="TEST",
        evaporator=evaporator,
        condenser=condenser,
        compressor=compressor,
        expansion_valve=expansion_valve,
    )
    dummy_prop = DummyMedProp()
    cycle.med_prop = dummy_prop
    for comp in cycle.get_all_components():
        comp.med_prop = dummy_prop
    compressor.med_prop = dummy_prop
    expansion_valve.med_prop = dummy_prop
    return cycle


def test_pinch_penalty_signs():
    error = 10.0
    penalty_con = MovingBoundaryNTU._apply_pinch_penalty(error, -1.0, False)
    penalty_eva = MovingBoundaryNTU._apply_pinch_penalty(error, -0.5, True)
    assert penalty_con < 0
    assert penalty_eva < 0


@pytest.mark.parametrize("dp_value", [None, 5000.0])
def test_calc_states_with_dp_iteration(dp_value):
    cycle = _build_cycle(dp_value)
    inputs = Inputs(
        n=0.5,
        T_eva_in=280.0,
        T_con_in=300.0,
        m_flow_eva=0.1,
        m_flow_con=0.1,
        dT_eva_superheating=5.0,
        dT_con_subcooling=5.0,
    )
    fs_state = FlowsheetState()
    target_p1 = 200000.0
    target_p2 = 600000.0

    cycle.calc_states(target_p1, target_p2, inputs=inputs, fs_state=fs_state)

    assert math.isclose(cycle.evaporator.state_outlet.p, target_p1, rel_tol=0, abs_tol=100)
    assert math.isclose(cycle.condenser.state_outlet.p, target_p2, rel_tol=0, abs_tol=100)
    if dp_value:
        assert cycle.condenser.state_inlet.p > cycle.condenser.state_outlet.p
