from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve


class StandardCycle(BaseCycle):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    flowsheet_name = "Standard"

    def __init__(
            self,
            compressor: Compressor,
            expansion_valve: ExpansionValve,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve = expansion_valve

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve
        ]

    def get_states_in_order_for_plotting(self):
        # Ensure compressor inlet matches evaporator outlet
        self.compressor.state_inlet = self.evaporator.state_outlet
        
        # If segmentation is enabled, don't add artificial saturation points
        # because the segments already provide the correct path
        has_segmentation = (hasattr(self.evaporator, 'use_segmentation') and self.evaporator.use_segmentation) or \
                          (hasattr(self.condenser, 'use_segmentation') and self.condenser.use_segmentation)
        
        if has_segmentation:
            # With segmentation: build path with ALL segment states (inlets AND outlets)
            states_list = [self.evaporator.state_inlet]
            
            # Add ALL evaporator segment states if available
            if hasattr(self.evaporator, 'segments_dict') and self.evaporator.segments_dict:
                segments_dict = self.evaporator.segments_dict
                # Phase order for evaporator: SC → LAT → SH
                phase_order = ['sc', 'lat', 'sh']
                for phase in phase_order:
                    if phase in segments_dict:
                        segments = segments_dict[phase]
                        for segment in segments:
                            # Add BOTH inlet and outlet of each segment to match orange visualization
                            if segment.state_inlet:
                                states_list.append(segment.state_inlet)
                            if segment.state_outlet:
                                states_list.append(segment.state_outlet)
            else:
                # No segments, just add outlet
                states_list.append(self.evaporator.state_outlet)
            
            # Add compressor
            states_list.append(self.compressor.state_outlet)
            states_list.append(self.condenser.state_inlet)
            
            # Add ALL condenser segment states if available
            if hasattr(self.condenser, 'segments_dict') and self.condenser.segments_dict:
                segments_dict = self.condenser.segments_dict
                # Phase order for condenser: SH → LAT → SC
                phase_order = ['sh', 'lat', 'sc']
                for phase in phase_order:
                    if phase in segments_dict:
                        segments = segments_dict[phase]
                        for segment in segments:
                            # Add BOTH inlet and outlet of each segment to match orange visualization
                            if segment.state_inlet:
                                states_list.append(segment.state_inlet)
                            if segment.state_outlet:
                                states_list.append(segment.state_outlet)
                # Determine final condenser outlet (last available segment outlet)
                final_condenser_outlet = None
                for phase in reversed(phase_order):
                    if phase in segments_dict:
                        for seg in reversed(segments_dict[phase]):
                            if getattr(seg, 'state_outlet', None) is not None:
                                final_condenser_outlet = seg.state_outlet
                                break
                        if final_condenser_outlet is not None:
                            break
                if final_condenser_outlet is not None:
                    # Overwrite global condenser outlet and ensure EV inlet continuity
                    self.condenser.state_outlet = final_condenser_outlet
                    self.expansion_valve.state_inlet = final_condenser_outlet
            else:
                # No segments, just add outlet
                states_list.append(self.condenser.state_outlet)

            # Add expansion valve (avoid duplicate if already last)
            if not states_list or states_list[-1] is not self.expansion_valve.state_inlet:
                states_list.append(self.expansion_valve.state_inlet)
            states_list.append(self.expansion_valve.state_outlet)
            
            return states_list
        else:
            # Without segmentation: include artificial saturation points for clarity
            return [
                self.evaporator.state_inlet,
                self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
                self.evaporator.state_outlet,
                self.evaporator.state_outlet,  # Compressor inlet = Evaporator outlet
                self.compressor.state_outlet,
                self.condenser.state_inlet,
                self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 1),
                self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 0),
                self.condenser.state_outlet,
                self.expansion_valve.state_inlet,
                self.expansion_valve.state_outlet,
            ]

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.evaporator.state_outlet
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow
        fs_state.set(
            name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            unit="-", description="Expansion valve opening"
        )
        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T, unit="K",
            description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet"
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
