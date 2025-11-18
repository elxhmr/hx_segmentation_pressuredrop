"""
Module for segmenting heat exchanger phases into computational segments.

This module provides functionality to divide heat exchanger phases (subcooling, latent, superheat)
into multiple segments for improved heat transfer calculations.
"""

from dataclasses import dataclass
from typing import List, Tuple, Dict
import numpy as np
from vclibpy.components.pressure_drop import PressureDrop


@dataclass
class PhaseSegment:
    """
    Represents a single segment within a heat exchanger phase.
    
    Attributes:
        phase_type (str): Type of phase ('sc', 'lat', 'sh')
        segment_index (int): Index of this segment within the phase (0-based)
        total_segments (int): Total number of segments for this phase
        A (float): Area assigned to this segment [m2]
        Q (float): Heat transferred in this segment [W] (computed later)
        dT_max (float): Maximum temperature difference for this segment
        state_inlet: Thermodynamic state at inlet of segment
        state_outlet: Thermodynamic state at outlet of segment
    """
    phase_type: str  # 'sc', 'lat', 'sh'
    segment_index: int
    total_segments: int
    A: float
    dT_max: float
    Q: float = 0.0
    state_inlet: object = None
    state_outlet: object = None
    
    def get_segment_fraction(self) -> float:
        """Get the fraction of heat for this segment relative to total phase heat."""
        return 1.0 / self.total_segments


class HeatExchangerSegmentation:
    """
    Manages segmentation of heat exchanger phases into computational segments.
    
    This class handles the division of subcooling (SC), latent (LAT), and superheat (SH)
    phases into multiple segments for improved accuracy in heat transfer calculations.
    
    Attributes:
        n_segments_sc (int): Number of segments for subcooling phase (default: 3)
        n_segments_lat (int): Number of segments for latent/two-phase (default: 5)
        n_segments_sh (int): Number of segments for superheat phase (default: 3)
    """
    
    def __init__(self, n_segments_sc: int = 3, n_segments_lat: int = 5, n_segments_sh: int = 3):
        """
        Initialize the HeatExchangerSegmentation.
        
        Args:
            n_segments_sc (int): Number of segments for subcooling phase. Default: 3
            n_segments_lat (int): Number of segments for latent/two-phase. Default: 5
            n_segments_sh (int): Number of segments for superheat phase. Default: 3
        """
        self.n_segments_sc = max(1, n_segments_sc)  # Ensure at least 1 segment
        self.n_segments_lat = max(1, n_segments_lat)
        self.n_segments_sh = max(1, n_segments_sh)
    
    def get_segment_count(self, phase_type: str) -> int:
        """
        Get the number of segments for a specific phase type.
        
        Args:
            phase_type (str): Type of phase ('sc', 'lat', 'sh')
            
        Returns:
            int: Number of segments for the phase type
        """
        if phase_type == 'sc':
            return self.n_segments_sc
        elif phase_type == 'lat':
            return self.n_segments_lat
        elif phase_type == 'sh':
            return self.n_segments_sh
        else:
            raise ValueError(f"Unknown phase type: {phase_type}")
    
    def segment_phase(self, phase_type: str, A_phase: float, dT_max: float) -> List[PhaseSegment]:
        """
        Segment a single phase into multiple computational segments.
        
        Args:
            phase_type (str): Type of phase ('sc', 'lat', 'sh')
            A_phase (float): Total area for this phase [m2]
            dT_max (float): Maximum temperature difference for this phase
            
        Returns:
            List[PhaseSegment]: List of segments for this phase
        """
        if A_phase <= 0:
            return []
        
        n_segments = self.get_segment_count(phase_type)
        segments = []
        
        # Distribute area equally among segments
        A_segment = A_phase / n_segments
        
        for i in range(n_segments):
            segment = PhaseSegment(
                phase_type=phase_type,
                segment_index=i,
                total_segments=n_segments,
                A=A_segment,
                dT_max=dT_max  # Same dT_max for all segments in a phase
            )
            segments.append(segment)
        
        return segments
    
    def segment_all_phases(self, A_sc: float, A_lat: float, A_sh: float,
                          dT_max_sc: float, dT_max_lat: float, dT_max_sh: float) -> Dict[str, List[PhaseSegment]]:
        """
        Segment all heat exchanger phases into computational segments.
        
        Args:
            A_sc (float): Area for subcooling phase [m2]
            A_lat (float): Area for latent/two-phase [m2]
            A_sh (float): Area for superheat phase [m2]
            dT_max_sc (float): Maximum temperature difference for subcooling
            dT_max_lat (float): Maximum temperature difference for latent phase
            dT_max_sh (float): Maximum temperature difference for superheat
            
        Returns:
            Dict[str, List[PhaseSegment]]: Dictionary with phase types as keys and segment lists as values
        """
        result = {}
        
        if A_sc > 0:
            result['sc'] = self.segment_phase('sc', A_sc, dT_max_sc)
        
        if A_lat > 0:
            result['lat'] = self.segment_phase('lat', A_lat, dT_max_lat)
        
        if A_sh > 0:
            result['sh'] = self.segment_phase('sh', A_sh, dT_max_sh)
        
        return result
    
    def get_total_Q(self, segments_dict: Dict[str, List[PhaseSegment]]) -> float:
        """
        Calculate total heat from all segments.
        
        Args:
            segments_dict (Dict[str, List[PhaseSegment]]): Dictionary of segmented phases
            
        Returns:
            float: Total heat across all segments
        """
        total_Q = 0.0
        for phase_segments in segments_dict.values():
            total_Q += sum(seg.Q for seg in phase_segments)
        return total_Q
    
    def get_segment_distribution(self, segments_dict: Dict[str, List[PhaseSegment]]) -> Dict[str, float]:
        """
        Get the heat distribution across phases and their segments.
        
        Args:
            segments_dict (Dict[str, List[PhaseSegment]]): Dictionary of segmented phases
            
        Returns:
            Dict[str, float]: Heat distribution information
        """
        total_Q = self.get_total_Q(segments_dict)
        distribution = {}
        
        for phase_type, segments in segments_dict.items():
            phase_Q = sum(seg.Q for seg in segments)
            if total_Q > 0:
                distribution[phase_type] = {
                    'total_Q': phase_Q,
                    'fraction': phase_Q / total_Q,
                    'n_segments': len(segments)
                }
        
        return distribution
    
    def merge_segments_by_phase(self, segments_dict: Dict[str, List[PhaseSegment]]) -> Dict[str, float]:
        """
        Merge all segments back to their phase totals.
        This is useful for comparison or rollback to non-segmented calculations.
        
        Args:
            segments_dict (Dict[str, List[PhaseSegment]]): Dictionary of segmented phases
            
        Returns:
            Dict[str, float]: Heat for each phase
        """
        result = {}
        for phase_type, segments in segments_dict.items():
            result[phase_type] = sum(seg.Q for seg in segments)
        return result
    
    def calculate_segment_states(self, phase_type: str, state_inlet, state_outlet, 
                                 med_prop, segments: List[PhaseSegment]) -> List[PhaseSegment]:
        """
        Calculate inlet and outlet states for each segment by interpolation.
        
        This method interpolates the thermodynamic states at the inlet and outlet
        of each segment based on enthalpy gradients.
        
        Args:
            phase_type (str): Type of phase ('sc', 'lat', 'sh')
            state_inlet: Inlet state at beginning of phase
            state_outlet: Outlet state at end of phase
            med_prop: Media property wrapper for state calculations
            segments (List[PhaseSegment]): List of segments to update with states
            
        Returns:
            List[PhaseSegment]: Updated segments with state_inlet and state_outlet set
        """
        if not segments:
            return segments
        
        total_Q = sum(seg.Q for seg in segments)
        if total_Q <= 0:
            return segments
        
        # For SC and SH phases: interpolate linearly by enthalpy
        if phase_type in ['sc', 'sh']:
            h_inlet = state_inlet.h
            h_outlet = state_outlet.h
            p_inlet = state_inlet.p
            
            # Total enthalpy change across phase
            dh_total = h_outlet - h_inlet
            dh_per_segment = dh_total / len(segments)
            
            # Cascade segments: outlet of segment i becomes inlet of segment i+1
            for i, segment in enumerate(segments):
                # Inlet of segment: previous outlet or phase inlet
                h_seg_inlet = h_inlet + i * dh_per_segment
                
                try:
                    segment.state_inlet = med_prop.calc_state("PH", p_inlet, h_seg_inlet)
                except:
                    segment.state_inlet = state_inlet
                
                # Outlet of segment
                h_seg_outlet = h_inlet + (i + 1) * dh_per_segment
                
                try:
                    segment.state_outlet = med_prop.calc_state("PH", p_inlet, h_seg_outlet)
                except:
                    segment.state_outlet = state_outlet
                
                # Recalculate dT_max based on actual calculated states
                if segment.state_inlet and segment.state_outlet:
                    segment.dT_max = abs(segment.state_outlet.T - segment.state_inlet.T)
        
        # For LAT phase: interpolate by quality
        elif phase_type == 'lat':
            p = state_inlet.p  # Constant pressure in two-phase
            
            # Quality at start and end of phase
            try:
                state_q0 = med_prop.calc_state("PQ", p, 0)  # Saturated liquid
                state_q1 = med_prop.calc_state("PQ", p, 1)  # Saturated vapor
                
                h_q0 = state_q0.h
                h_q1 = state_q1.h
                h_inlet_lat = state_inlet.h
                h_outlet_lat = state_outlet.h
                
                # Calculate quality at start and end of phase
                q_inlet = (h_inlet_lat - h_q0) / (h_q1 - h_q0) if (h_q1 - h_q0) != 0 else 0.5
                q_outlet = (h_outlet_lat - h_q0) / (h_q1 - h_q0) if (h_q1 - h_q0) != 0 else 0.5
                
                # Total quality change across phase
                dq_total = q_outlet - q_inlet
                dq_per_segment = dq_total / len(segments)
                
                # Cascade segments: outlet of segment i becomes inlet of segment i+1
                for i, segment in enumerate(segments):
                    # Inlet of segment: progressive quality increase
                    q_seg_inlet = q_inlet + i * dq_per_segment
                    q_seg_inlet = max(0, min(1, q_seg_inlet))  # Clamp to [0, 1]
                    
                    try:
                        segment.state_inlet = med_prop.calc_state("PQ", p, q_seg_inlet)
                    except:
                        segment.state_inlet = state_inlet
                    
                    # Outlet of segment
                    q_seg_outlet = q_inlet + (i + 1) * dq_per_segment
                    q_seg_outlet = max(0, min(1, q_seg_outlet))  # Clamp to [0, 1]
                    
                    try:
                        segment.state_outlet = med_prop.calc_state("PQ", p, q_seg_outlet)
                    except:
                        segment.state_outlet = state_outlet
                    
                    # Recalculate dT_max based on actual calculated states
                    # In latent phase, T should be constant but round-off errors may occur
                    if segment.state_inlet and segment.state_outlet:
                        segment.dT_max = abs(segment.state_outlet.T - segment.state_inlet.T)
            except:
                # Fallback if state calculation fails
                for segment in segments:
                    segment.state_inlet = state_inlet
                    segment.state_outlet = state_outlet
        
        return segments


class SegmentPressureDropCalculator:
    """
    Calculates pressure drops for each heat exchanger segment.
    
    This calculator:
    - Computes pressure drop for each segment based on its inlet state and mass flow
    - Accumulates pressure drops through the heat exchanger
    - Tracks segment-wise pressure information
    
    Attributes:
        pressure_drop_model (PressureDrop): Model for pressure drop calculation
    """
    
    def __init__(self, pressure_drop_model: PressureDrop = None):
        """
        Initialize the segment pressure drop calculator.
        
        Args:
            pressure_drop_model (PressureDrop): Pressure drop model to use.
                                               If None, no pressure drops are calculated.
        """
        self.pressure_drop_model = pressure_drop_model
    
    def calculate_segment_pressure_drops(self, 
                                        phase_type: str,
                                        segments: List[PhaseSegment],
                                        m_flow: float,
                                        med_prop) -> Dict[str, List[float]]:
        """
        Calculate pressure drops for all segments in a phase.
        
        For each segment, computes:
        - Pressure drop based on inlet state and mass flow
        - Outlet pressure (inlet pressure - dp)
        
        Args:
            phase_type (str): Type of phase ('sc', 'lat', 'sh')
            segments (List[PhaseSegment]): List of segments to calculate pressure drops for
            m_flow (float): Mass flow rate in kg/s
            med_prop: Media properties wrapper for state recalculation
        
        Returns:
            Dict with keys 'inlet_pressures', 'outlet_pressures', 'pressure_drops'
        """
        if not segments or self.pressure_drop_model is None:
            return {
                'inlet_pressures': [],
                'outlet_pressures': [],
                'pressure_drops': [],
                'segments_with_pressure': segments
            }
        
        inlet_pressures = []
        outlet_pressures = []
        pressure_drops_list = []
        segments_updated = []
        
        for i, segment in enumerate(segments):
            if segment.state_inlet is None:
                continue
            
            # Get inlet state
            p_inlet = segment.state_inlet.p
            inlet_pressures.append(p_inlet)
            
            # Calculate pressure drop using the model
            transport_props = None  # Could be extended for viscosity, density, etc.
            dp = self.pressure_drop_model.calc(transport_props, m_flow)
            pressure_drops_list.append(dp)
            
            # Calculate outlet pressure
            p_outlet = max(p_inlet - dp, 1000)  # Ensure minimum pressure
            outlet_pressures.append(p_outlet)
            
            # Recalculate outlet state with new pressure
            # This updates the segment's state_outlet with the new pressure
            try:
                # Use enthalpy from current state_outlet but apply new pressure
                h_outlet = segment.state_outlet.h if segment.state_outlet else segment.state_inlet.h
                segment.state_outlet = med_prop.calc_state("PH", p_outlet, h_outlet)
            except:
                # If state calculation fails, keep old outlet state
                pass
            
            segments_updated.append(segment)
        
        return {
            'inlet_pressures': inlet_pressures,
            'outlet_pressures': outlet_pressures,
            'pressure_drops': pressure_drops_list,
            'segments_with_pressure': segments_updated
        }
    
    def calculate_all_phase_pressure_drops(self,
                                          segments_dict: Dict[str, List[PhaseSegment]],
                                          m_flow: float,
                                          med_prop) -> Dict:
        """
        Calculate pressure drops for all phases.
        
        Args:
            segments_dict (Dict): Dictionary with phase types as keys and segment lists as values
            m_flow (float): Mass flow rate in kg/s
            med_prop: Media properties wrapper
        
        Returns:
            Dict with pressure drop data organized by phase
        """
        result = {}
        
        for phase_type, segments in segments_dict.items():
            result[phase_type] = self.calculate_segment_pressure_drops(
                phase_type, segments, m_flow, med_prop
            )
        
        return result
