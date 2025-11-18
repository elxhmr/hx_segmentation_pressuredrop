# -*- coding: utf-8 -*-
"""
Created on 24.04.2023

@author: Christoph Hoeges

Last Update: 24.04.2023
"""
import pandas as pd


def print_states(**kwargs):
    """
    Transforms given states to DataFrame and prints table layout.
    Declaration of state must start with 'state'

    Returns:
    :return pandas.DataFrame df_states:
        DataFrame with states
    """
    # Remove keys in given data that are not label as "state"
    for key in kwargs.keys():
        if not key.startswith("state"):
            print(
                f"Given value {key} is removed from data transformation! "
                f"Value must be declared as 'state'"
            )
            kwargs.pop(key, None)

    # Extract data from ThermodynamicStates
    data_states = {key: {"state": key[5:], "T": data.T, "p": data.p, "h": data.h, "s": data.s, "d": data.d}
                   for key, data in kwargs.items()}
    df_states = pd.DataFrame.from_dict(data=data_states)

    # Similar to previous results - now with rounded data
    data_states_rounded = {key: {"State": key[5:],
                                 "T in °C": round(data.T - 273.15, 2),
                                 "p in bar": round(data.p * 1e-5, 3),
                                 "h in kJ/kg": round(data.h * 1e-3, 3),
                                 "s in kJ/kg": round(data.s * 1e-3, 3),
                                 "d in kg/m3": round(data.d, 3)}
                           for key, data in kwargs.items()}
    df_states_rounded = pd.DataFrame.from_dict(data=data_states_rounded)
    print(df_states_rounded.T)

    return df_states


def print_segment_states(segments_dict, phase_type=None):
    """
    Prints segment states from heat exchanger segmentation in one combined table format.
    
    This function takes the segmented phases (containing state_inlet and state_outlet)
    and displays them in a single formatted table with all relevant data.
    
    Args:
        segments_dict (dict): Dictionary with phase types as keys and segment lists as values.
                             Each segment should have state_inlet and state_outlet attributes.
        phase_type (str, optional): If specified, only print segments of this phase ('sc', 'lat', 'sh').
                                    If None, print all phases.
    
    Example:
        >>> from vclibpy.utils.printing import print_segment_states
        >>> print_segment_states(segments_dict)  # Print all segments
        >>> print_segment_states(segments_dict, phase_type='lat')  # Only latent phase
    """
    if not segments_dict:
        print("No segment data available.")
        return
    
    # Map phase types to readable names
    phase_names = {
        'sc': 'Subcooling (SC)',
        'lat': 'Two-Phase (TP)',
        'sh': 'Superheat (SH)'
    }
    
    # Filter by phase_type if specified
    phases_to_print = {phase_type: segments_dict[phase_type]} if phase_type and phase_type in segments_dict else segments_dict
    
    first_phase = True
    for phase, segments in phases_to_print.items():
        if not first_phase:
            print(f"\n{'-'*180}")
        first_phase = False
        
        phase_name = phase_names.get(phase, phase.upper())
        print(f"\n{phase_name.upper()} - {len(segments)} Segments")
        print(f"{'-'*180}")
        
        # Build data for DataFrame
        data = []
        for segment in segments:
            data.append({
                "Segment": f"{segment.segment_index + 1}/{segment.total_segments}",
                "Q (W)": f"{segment.Q:.2f}",
                "% of Phase": f"{segment.get_segment_fraction()*100:.1f}%",
                "dT_max (K)": f"{segment.dT_max:.2f}",
                "T_inlet (°C)": f"{segment.state_inlet.T - 273.15:.2f}" if segment.state_inlet else "N/A",
                "p_inlet (bar)": f"{segment.state_inlet.p*1e-5:.3f}" if segment.state_inlet else "N/A",
                "h_inlet (kJ/kg)": f"{segment.state_inlet.h*1e-3:.3f}" if segment.state_inlet else "N/A",
                "T_outlet (°C)": f"{segment.state_outlet.T - 273.15:.2f}" if segment.state_outlet else "N/A",
                "p_outlet (bar)": f"{segment.state_outlet.p*1e-5:.3f}" if segment.state_outlet else "N/A",
                "h_outlet (kJ/kg)": f"{segment.state_outlet.h*1e-3:.3f}" if segment.state_outlet else "N/A",
            })
        
        df_segments = pd.DataFrame(data)
        print(df_segments.to_string(index=False))
    
    print(f"\n{'-'*180}\n")


def print_segment_summary(segments_dict, hx_type='condenser'):
    """
    Prints a summary table of all segments with their heat distribution and state information.
    
    Args:
        segments_dict (dict): Dictionary with phase types as keys and segment lists as values.
        hx_type (str): 'condenser' or 'evaporator' to determine phase ordering.
    
    Example:
        >>> from vclibpy.utils.printing import print_segment_summary
        >>> print_segment_summary(segments_dict)
    """
    if not segments_dict:
        print("No segment data available.")
        return
    
    print()
    
    data = []
    total_Q = 0
    
    # Define phase ordering based on HX type
    if hx_type == 'condenser':
        phase_order = ['sh', 'lat', 'sc']  # Condenser: SH → LAT → SC
    else:  # evaporator
        phase_order = ['sc', 'lat', 'sh']  # Evaporator: SC → LAT → SH
    
    # Process phases in defined order
    for phase in phase_order:
        if phase in segments_dict:
            segments = segments_dict[phase]
            for segment in segments:
                data.append({
                    "Phase": phase.upper(),
                    "Segment": f"{segment.segment_index + 1}/{segment.total_segments}",
                    "Q (W)": f"{segment.Q:.2f}",
                    "% of Phase": f"{segment.get_segment_fraction()*100:.1f}%",
                    "dT_max (K)": f"{segment.dT_max:.2f}",
                    "T_inlet (°C)": f"{segment.state_inlet.T - 273.15:.4f}" if segment.state_inlet else "N/A",
                    "p_inlet (bar)": f"{segment.state_inlet.p*1e-5:.3f}" if segment.state_inlet else "N/A",
                    "h_inlet (kJ/kg)": f"{segment.state_inlet.h*1e-3:.3f}" if segment.state_inlet else "N/A",
                    "T_outlet (°C)": f"{segment.state_outlet.T - 273.15:.4f}" if segment.state_outlet else "N/A",
                    "p_outlet (bar)": f"{segment.state_outlet.p*1e-5:.3f}" if segment.state_outlet else "N/A",
                    "h_outlet (kJ/kg)": f"{segment.state_outlet.h*1e-3:.3f}" if segment.state_outlet else "N/A",
                })
                total_Q += segment.Q
    
    df_summary = pd.DataFrame(data)
    print(df_summary.to_string(index=False))
    print(f"\n{'-'*180}")
    print(f"Total Heat: {total_Q:.2f} W")
    print(f"{'='*180}\n")


def print_hx_segmentation(flowsheet_state, hx_type='both'):
    """
    Print heat exchanger segmentation data from FlowsheetState.
    
    Retrieves and formats segment data for condenser, evaporator, or both.
    
    Args:
        flowsheet_state: FlowsheetState object containing segment data
        hx_type (str): 'condenser', 'evaporator', or 'both' (default: 'both')
    
    Example:
        >>> from vclibpy.utils.printing import print_hx_segmentation
        >>> print_hx_segmentation(fs)  # Print both condenser and evaporator
        >>> print_hx_segmentation(fs, hx_type='condenser')  # Only condenser
    """
    var_names = flowsheet_state.get_variable_names()
    
    # Condenser segmentation
    if hx_type in ['both', 'condenser']:
        if "segments_con_sc" in var_names or "segments_con_lat" in var_names or "segments_con_sh" in var_names:
            segments_con = {}
            if "segments_con_sc" in var_names:
                var_obj = flowsheet_state.get("segments_con_sc")
                segments_con['sc'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            if "segments_con_lat" in var_names:
                var_obj = flowsheet_state.get("segments_con_lat")
                segments_con['lat'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            if "segments_con_sh" in var_names:
                var_obj = flowsheet_state.get("segments_con_sh")
                segments_con['sh'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            
            if segments_con:
                print("\n" + "=" * 180)
                print("CONDENSER SEGMENTATION")
                print("=" * 180)
                print_segment_summary(segments_con, hx_type='condenser')
    
    # Evaporator segmentation
    if hx_type in ['both', 'evaporator']:
        if "segments_eva_sc" in var_names or "segments_eva_lat" in var_names or "segments_eva_sh" in var_names:
            segments_eva = {}
            if "segments_eva_sc" in var_names:
                var_obj = flowsheet_state.get("segments_eva_sc")
                segments_eva['sc'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            if "segments_eva_lat" in var_names:
                var_obj = flowsheet_state.get("segments_eva_lat")
                segments_eva['lat'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            if "segments_eva_sh" in var_names:
                var_obj = flowsheet_state.get("segments_eva_sh")
                segments_eva['sh'] = var_obj.value if hasattr(var_obj, 'value') else var_obj
            
            if segments_eva:
                print("\n" + "=" * 180)
                print("EVAPORATOR SEGMENTATION")
                print("=" * 180)
                print_segment_summary(segments_eva, hx_type='evaporator')


def print_all_states(flowsheet_state, states=None):
    """
    Convenience one-liner to print both the thermodynamic states table and
    the heat-exchanger segment tables.

    Args:
        flowsheet_state: FlowsheetState containing segment variables
        states (list | dict | None): Optional collection of states to print as the
            main table. If a list is provided, it will be converted to kwargs like
            state_0, state_1, ... If a dict is provided, its items will be forwarded
            to print_states (keys not starting with 'state' will be auto-mapped).

    Notes:
        - This does not modify the legacy print_states API.
        - If no states are provided, only segmentation is printed.
    """
    # Print thermodynamic states table if provided
    if states is not None:
        try:
            # Deduplicate by (p,h,T) triple to avoid repeated identical states
            if isinstance(states, dict):
                raw_list = list(states.values())
            else:
                raw_list = list(states)
            seen = set()
            unique_states = []
            for st in raw_list:
                if st is None:
                    continue
                key = (round(st.p, 6), round(st.h, 3), round(st.T, 4))
                if key in seen:
                    continue
                seen.add(key)
                unique_states.append(st)
            # Map to legacy state_* kwargs
            mapped = {f"state_{i}": s for i, s in enumerate(unique_states)}
            print("\n" + "=" * 90)
            print("THERMODYNAMIC STATES (UNIQUE)")
            print("=" * 90)
            print_states(**mapped)
        except Exception as err:
            print(f"[print_all_states] Failed to print thermodynamic states: {err}")
    # Segmentation
    try:
        print_hx_segmentation(flowsheet_state, hx_type='both')
    except Exception as err:
        print(f"[print_all_states] Failed to print segmentation: {err}")


def print_inlet_outlet_comp(flowsheet):
    """Print only inlet/outlet thermodynamic states for each component.

    Args:
        flowsheet: Cycle/flowsheet object with component attributes.
    """
    try:
        components = []
        for attr in ("evaporator", "compressor", "condenser", "expansion_valve"):
            if hasattr(flowsheet, attr):
                comp = getattr(flowsheet, attr)
                if hasattr(comp, 'state_inlet') and hasattr(comp, 'state_outlet'):
                    components.append((attr, comp))
        if not components:
            print("[print_inlet_outlet_comp] No components with inlet/outlet states found.")
            return
        rows = []
        for name, comp in components:
            inlet = getattr(comp, 'state_inlet', None)
            outlet = getattr(comp, 'state_outlet', None)
            rows.append({
                'Component': name,
                'T_in (°C)': f"{inlet.T - 273.15:.3f}" if inlet else 'N/A',
                'p_in (bar)': f"{inlet.p*1e-5:.3f}" if inlet else 'N/A',
                'h_in (kJ/kg)': f"{inlet.h*1e-3:.3f}" if inlet else 'N/A',
                'T_out (°C)': f"{outlet.T - 273.15:.3f}" if outlet else 'N/A',
                'p_out (bar)': f"{outlet.p*1e-5:.3f}" if outlet else 'N/A',
                'h_out (kJ/kg)': f"{outlet.h*1e-3:.3f}" if outlet else 'N/A',
            })
        import pandas as _pd
        df_io = _pd.DataFrame(rows)
        print("\n" + "=" * 120)
        print("COMPONENT INLET / OUTLET STATES")
        print("=" * 120)
        print(df_io.to_string(index=False))
    except Exception as err:
        print(f"[print_inlet_outlet_comp] Failed: {err}")

