from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Tuple

from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.heat_exchangers import HeatExchanger
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.flowsheets import BaseCycle

logger = logging.getLogger(__name__)


@dataclass
class _DpIterationState:
    """Container for the internal Δp iteration state."""

    p_eva_out_target: float
    p_con_out_target: float
    p_eva_in_work: float
    p_con_in_work: float
    dp_eva_last: float = 0.0
    dp_con_last: float = 0.0


class StandardCycleWITHdp(BaseCycle):
    """Standard cycle variant with encapsulated pressure-drop iteration."""

    flowsheet_name = "Standard (WITH Δp)"

    def __init__(
        self,
        compressor: Compressor,
        expansion_valve: ExpansionValve,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve = expansion_valve

    def get_all_components(self):
        return super().get_all_components() + [self.compressor, self.expansion_valve]

    def get_states_in_order_for_plotting(self):
        # Reuse plotting behaviour from the standard cycle
        from vclibpy.flowsheets.standard import StandardCycle

        return StandardCycle.get_states_in_order_for_plotting(self)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        if not getattr(self.evaporator, "apply_pressure_drops", False) or getattr(
            self.evaporator, "pressure_drop_model", None
        ) is None:
            # Legacy path: identical to StandardCycle
            return self._calc_states_no_dp(p_1, p_2, inputs, fs_state)

        iteration = _DpIterationState(
            p_eva_out_target=p_1,
            p_con_out_target=p_2,
            p_eva_in_work=p_1,
            p_con_in_work=p_2,
        )

        max_iter = 20
        tol = 50.0  # Pa
        relax = 0.5
        for _ in range(max_iter):
            self._calc_states_with_dp_levels(iteration, inputs, fs_state)

            error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
            error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)

            dp_eva = self._get_total_dp(self.evaporator, fs_state)
            dp_con = self._get_total_dp(self.condenser, fs_state)

            eva_res = iteration.p_eva_out_target - getattr(self.evaporator.state_outlet, "p", iteration.p_eva_out_target)
            con_res = iteration.p_con_out_target - getattr(self.condenser.state_outlet, "p", iteration.p_con_out_target)

            iteration.p_eva_in_work = iteration.p_eva_out_target + dp_eva + relax * eva_res
            iteration.p_con_in_work = iteration.p_con_out_target + dp_con + relax * con_res

            if abs(eva_res) < tol and abs(con_res) < tol:
                break
            iteration.dp_eva_last = dp_eva
            iteration.dp_con_last = dp_con

        # Final deterministic pass with last pressures
        self._calc_states_with_dp_levels(iteration, inputs, fs_state)
        self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        self.condenser.calc(inputs=inputs, fs_state=fs_state)

    def _calc_states_no_dp(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        from vclibpy.flowsheets.standard import StandardCycle

        return StandardCycle.calc_states(self, p_1, p_2, inputs, fs_state)

    def _calc_states_with_dp_levels(
        self,
        iteration: _DpIterationState,
        inputs: Inputs,
        fs_state: FlowsheetState,
    ) -> None:
        p_con_out_set = iteration.p_con_out_target + max(iteration.dp_con_last, 0.0)
        p_eva_out_set = iteration.p_eva_out_target + max(iteration.dp_eva_last, 0.0)
        # Condenser outlet fixed by target pressure level
        self.set_condenser_outlet_based_on_subcooling(
            p_con=p_con_out_set, inputs=inputs
        )
        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=iteration.p_eva_in_work)

        # Evaporator inlet/outlet with decoupled pressures
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        self.set_evaporator_outlet_based_on_superheating(
            p_eva=p_eva_out_set, inputs=inputs
        )

        # Compressor between the working inlet/outlet pressures
        self.compressor.state_inlet = self.evaporator.state_outlet
        self.compressor.calc_state_outlet(
            p_outlet=iteration.p_con_in_work, inputs=inputs, fs_state=fs_state
        )
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow

        fs_state.set(
            name="y_EV",
            value=self.expansion_valve.calc_opening_at_m_flow(
                m_flow=self.expansion_valve.m_flow
            ),
            unit="-",
            description="Expansion valve opening",
        )
        fs_state.set(
            name="T_1",
            value=self.evaporator.state_outlet.T,
            unit="K",
            description="Refrigerant temperature at evaporator outlet",
        )
        fs_state.set(
            name="T_2",
            value=self.compressor.state_outlet.T,
            unit="K",
            description="Compressor outlet temperature",
        )
        fs_state.set(
            name="T_3",
            value=self.condenser.state_outlet.T,
            unit="K",
            description="Refrigerant temperature at condenser outlet",
        )
        fs_state.set(
            name="T_4",
            value=self.evaporator.state_inlet.T,
            unit="K",
            description="Refrigerant temperature at evaporator inlet",
        )
        fs_state.set(
            name="p_con", value=iteration.p_con_out_target, unit="Pa", description="Condensation pressure"
        )
        fs_state.set(
            name="p_eva", value=iteration.p_eva_out_target, unit="Pa", description="Evaporation pressure"
        )

    @staticmethod
    def _get_total_dp(component: HeatExchanger, fs_state: FlowsheetState) -> float:
        key = f"{component.__class__.__name__}_total_dp"
        try:
            if key in fs_state.get_variable_names():
                return float(fs_state.get(key).value)
        except Exception:
            pass
        return 0.0

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
