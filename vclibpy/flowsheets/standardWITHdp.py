"""Pressure-drop aware standard cycle.

``p1`` and ``p2`` are interpreted as evaporation and condensation *level*
pressures. The actual port pressures are derived from the pressure-drop totals
calculated inside the segmented condenser and evaporator. Legacy behaviour is
preserved when segmentation or ``dp_model`` hooks are not configured on the
heat exchangers.
"""

from vclibpy.flowsheets.standard import StandardCycle


class StandardCycleWithDP(StandardCycle):
    """Standard cycle that propagates pressure drops into port pressures."""

    flowsheet_name = "StandardWithDP"

    def __init__(self, *args, dp_model=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.dp_model = dp_model

    @staticmethod
    def _get_dp(fs_state, key: str) -> float:
        try:
            return max(fs_state.get(key).value, 0)
        except (KeyError, AttributeError):
            return 0.0

    def calc_states(self, p_1, p_2, inputs, fs_state):
        """Calculate states with condenser/evaporator pressure-drop awareness."""

        dp_con = self._get_dp(fs_state, "dp_con_total")
        dp_eva = self._get_dp(fs_state, "dp_eva_total")

        # Level pressures are provided by the steady-state solver; derive port
        # pressures from the most recent pressure-drop calculation.
        p_con_out = p_2
        p_con_in = p_con_out + dp_con
        p_eva_out = p_1
        p_eva_in = p_eva_out + dp_eva

        self.set_condenser_outlet_based_on_subcooling(p_con=p_con_out, inputs=inputs)
        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=p_eva_in)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_eva_out, inputs=inputs)
        self.compressor.state_inlet = self.evaporator.state_outlet
        self.compressor.calc_state_outlet(p_outlet=p_con_in, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow
        fs_state.set(
            name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            unit="-", description="Expansion valve opening",
        )
        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet",
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature",
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T, unit="K",
            description="Refrigerant temperature at condenser outlet",
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet",
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation level pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation level pressure")
        fs_state.set(name="p_con_in", value=p_con_in, unit="Pa", description="Condenser inlet pressure")
        fs_state.set(name="p_con_out", value=p_con_out, unit="Pa", description="Condenser outlet pressure")
        fs_state.set(name="p_eva_in", value=p_eva_in, unit="Pa", description="Evaporator inlet pressure")
        fs_state.set(name="p_eva_out", value=p_eva_out, unit="Pa", description="Evaporator outlet pressure")
