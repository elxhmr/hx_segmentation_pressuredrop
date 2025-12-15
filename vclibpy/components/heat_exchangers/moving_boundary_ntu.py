import abc
import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.ntu import BasicNTU
from vclibpy.components.segmentation import HeatExchangerSegmentation
from vclibpy.components.segmentation import SegmentPressureDropCalculator
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


class MovingBoundaryNTU(BasicNTU, abc.ABC):
    """
    Moving boundary NTU based heat exchanger.
    
    Supports phase segmentation for improved heat transfer calculations.
    Segments can be configured via n_segments_sc, n_segments_lat, n_segments_sh attributes.

    See parent classe for arguments.
    """
    
    def __init__(
        self,
        *args,
        geometry: str | None = None,
        n_segments_sc: int = 3,
        n_segments_lat: int = 5,
        n_segments_sh: int = 3,
        use_segmentation: bool = True,
        pressure_drop_model=None,
        two_phase_pressure_drop=None,
        gas_pressure_drop=None,
        liquid_pressure_drop=None,
        apply_pressure_drops: bool = True,
        **kwargs,
    ):
        """Initialize MovingBoundaryNTU with segmentation and optional geometry.
        
        Args:
            n_segments_sc (int): Number of segments for subcooling phase. Default: 3
            n_segments_lat (int): Number of segments for latent/two-phase. Default: 5
            n_segments_sh (int): Number of segments for superheat phase. Default: 3
            use_segmentation (bool): Enable phase segmentation. Default: True
            pressure_drop_model: Backward-compatible global pressure drop model.
            two_phase_pressure_drop: Optional pressure-drop model for two-phase segments.
            gas_pressure_drop: Optional pressure-drop model for superheated-vapor segments.
            liquid_pressure_drop: Optional pressure-drop model for liquid segments.
            apply_pressure_drops (bool): Enable application of pressure drops. Default: True
        """
        # Ensure a minimal dummy area is available for the HeatExchanger base
        # class; this will be overwritten if geometry is provided.
        if "A" not in kwargs:
            kwargs["A"] = 10.0

        # Initialize base NTU/HX behaviour first
        super().__init__(*args, **kwargs)

        # Geometry-related attributes (can be None if no geometry is provided)
        self.geometry_name: str | None = geometry
        self.geom = None
        self.A_outer: float | None = None
        self.A_inner: float | None = None
        self.length_hx: float | None = None
        self.d_h: float | None = None

        # If a geometry name is provided, load it and configure areas/ratio
        if geometry is not None:
            try:
                from vclibpy.components.heat_exchangers import hx_model

                # geometry is the base name without .txt
                self.geom = hx_model.load_geometry(geometry)

                self.A_outer = float(self.geom.outer_area_m2)
                self.A_inner = float(self.geom.inner_area_m2)
                self.length_hx = float(self.geom.length_m)
                self.d_h = float(self.geom.hydraulic_diameter_m)

                # Use geometric areas if available to define A and the ratio
                if self.A_outer is not None:
                    # A is the outer / secondary side area in the NTU formulation
                    self.A = self.A_outer

                if self.A_outer is not None and self.A_inner not in (None, 0):
                    self.ratio_outer_to_inner_area = self.A_outer / max(self.A_inner, 1e-12)
            except Exception as exc:  # geometry is optional, so fail soft
                logger.warning("Failed to load geometry '%s' for %s: %s", geometry, self.__class__.__name__, exc)
        self.use_segmentation = use_segmentation
        self.pressure_drop_model = pressure_drop_model
        # Optional phase-specific pressure-drop models; fall back to global if not set
        self.two_phase_pressure_drop = two_phase_pressure_drop or pressure_drop_model
        self.gas_pressure_drop = gas_pressure_drop or pressure_drop_model
        self.liquid_pressure_drop = liquid_pressure_drop or pressure_drop_model
        self.apply_pressure_drops = apply_pressure_drops
        self.segmentation = HeatExchangerSegmentation(
            n_segments_sc=n_segments_sc,
            n_segments_lat=n_segments_lat,
            n_segments_sh=n_segments_sh
        )

    @staticmethod
    def _apply_pinch_penalty(error: float, dT_min: float, drives_lower_pressure: bool) -> float:
        """Return an error value that enforces pinch feasibility via the BaseCycle solver.

        Args:
            error: Nominal NTU error.
            dT_min: Minimal temperature difference detected (may be negative).
            drives_lower_pressure: ``True`` if a negative return value should decrease the
                pressure level (evaporator), ``False`` to increase it (condenser).
        """
        if isinstance(dT_min, (int, float)) and dT_min < 0:
            magnitude = abs(error) if isinstance(error, (int, float)) else 1.0
            penalty = -magnitude if drives_lower_pressure else -magnitude
            if magnitude == 0:
                penalty = -1.0
            return penalty
        return error

    @staticmethod
    def _safe_lmtd(dT1: float, dT2: float, eps: float = 1e-6) -> float:
        """
        Compute LMTD robustly for two temperature differences.
        """
        dT1 = float(abs(dT1))
        dT2 = float(abs(dT2))
        if dT1 < eps and dT2 < eps:
            return 0.0
        if abs(dT1 - dT2) < eps:
            return 0.5 * (dT1 + dT2)
        try:
            return (dT1 - dT2) / np.log(max(dT1, eps) / max(dT2, eps))
        except Exception:
            return 0.5 * (dT1 + dT2)

    def _compute_segments_by_area(self,
                                  phase_type: str,
                                  segments,
                                  state_inlet,
                                  state_outlet,
                                  T_sec_in: float,
                                  T_sec_out: float,
                                  k_phase: float,
                                  Q_phase: float):
        """
        Given equal area segments, compute per-segment Q and states via a small internal iteration.
        """
        if not segments:
            return segments

        # If no heat in phase, just set states by linear enthalpy partition and zero Q
        if Q_phase <= 0 or k_phase <= 0 or abs(T_sec_out - T_sec_in) < 1e-9:
            n = len(segments)
            h_in = state_inlet.h
            h_out = state_outlet.h
            p = state_inlet.p
            for i, seg in enumerate(segments):
                frac_in = i / n
                frac_out = (i + 1) / n
                h_seg_in = h_in + frac_in * (h_out - h_in)
                h_seg_out = h_in + frac_out * (h_out - h_in)
                try:
                    seg.state_inlet = self.med_prop.calc_state("PH", p, h_seg_in)
                    seg.state_outlet = self.med_prop.calc_state("PH", p, h_seg_out)
                except Exception:
                    seg.state_inlet = state_inlet
                    seg.state_outlet = state_outlet
                seg.Q = 0.0
                seg.dT_max = abs(seg.state_outlet.T - seg.state_inlet.T)
            return segments

        n = len(segments)
        # Initialize uniform Q split
        Q = np.full(n, Q_phase / n, dtype=float)

        # Initialize secondary temperature boundaries uniformly
        T_sec_bounds = np.linspace(T_sec_in, T_sec_out, n + 1)

        p = state_inlet.p
        max_iter = 10
        tol = 1e-6 * max(abs(Q_phase), 1.0)

        for _ in range(max_iter):
            # Build primary enthalpy bounds from Q and m_flow
            h_bounds = [state_inlet.h]
            sign_dh = np.sign(state_outlet.h - state_inlet.h) if abs(state_outlet.h - state_inlet.h) > 0 else 1.0
            for i in range(n):
                dh_i = sign_dh * (Q[i] / max(self.m_flow, 1e-12))
                h_bounds.append(h_bounds[-1] + dh_i)

            # Correct last bound to match outlet exactly to avoid drift
            h_bounds[-1] = state_outlet.h

            # Compute LMTD and new Q from k*A*LMTD
            Q_new = np.zeros_like(Q)
            for i, seg in enumerate(segments):
                h_in_i = h_bounds[i]
                h_out_i = h_bounds[i + 1]

                # Build primary states to get temperatures
                try:
                    st_in = self.med_prop.calc_state("PH", p, h_in_i)
                    st_out = self.med_prop.calc_state("PH", p, h_out_i)
                except Exception:
                    st_in, st_out = state_inlet, state_outlet

                T_p_in = st_in.T
                T_p_out = st_out.T

                T_s_in_i = T_sec_bounds[i]
                T_s_out_i = T_sec_bounds[i + 1]

                dT1 = T_p_in - T_s_in_i
                dT2 = T_p_out - T_s_out_i
                LMTD_i = self._safe_lmtd(dT1, dT2)

                Q_new[i] = max(0.0, k_phase * segments[i].A * LMTD_i)

                # Stash states temporarily for later write-back
                segments[i].state_inlet = st_in
                segments[i].state_outlet = st_out
                segments[i].dT_max = abs(T_p_out - T_p_in)

            sum_Qn = float(np.sum(Q_new))
            if sum_Qn <= 0:
                # Degenerate; keep uniform
                break

            # Normalize to match phase total
            scale = Q_phase / sum_Qn
            Q_updated = Q_new * scale

            # Update secondary temperature bounds proportional to cumulative Q
            frac = np.concatenate([[0.0], np.cumsum(Q_updated) / Q_phase])
            T_sec_bounds = T_sec_in + (T_sec_out - T_sec_in) * frac

            # Check convergence
            if np.max(np.abs(Q_updated - Q)) < tol:
                Q = Q_updated
                break
            Q = Q_updated

        # Final write-back of Q to segments
        for i, seg in enumerate(segments):
            seg.Q = float(Q[i])

        return segments

    def separate_phases(self, state_max: ThermodynamicState, state_min: ThermodynamicState, p: float):
        """
        Separates a flow with possible phase changes into three parts:
        subcooling (sc), latent phase change (lat), and superheating (sh)
        at the given pressure.

        Args:
            state_max (ThermodynamicState): State with higher enthalpy.
            state_min (ThermodynamicState): State with lower enthalpy.
            p (float): Pressure of phase change.

        Returns:
            Tuple[float, float, float, ThermodynamicState, ThermodynamicState]:
                Q_sc: Heat for subcooling.
                Q_lat: Heat for latent phase change.
                Q_sh: Heat for superheating.
                state_q0: State at vapor quality 0 and the given pressure.
                state_q1: State at vapor quality 1 and the given pressure.
        """
        # Get relevant states:
        state_q0 = self.med_prop.calc_state("PQ", p, 0)
        state_q1 = self.med_prop.calc_state("PQ", p, 1)
        Q_sc = max(0.0,
                   min((state_q0.h - state_min.h),
                       (state_max.h - state_min.h))) * self.m_flow
        Q_lat = max(0.0,
                    (min(state_max.h, state_q1.h) -
                     max(state_min.h, state_q0.h))) * self.m_flow
        Q_sh = max(0.0,
                   min((state_max.h - state_q1.h),
                       (state_max.h - state_min.h))) * self.m_flow
        return Q_sc, Q_lat, Q_sh, state_q0, state_q1
    
    def get_segmented_phases(self, Q_sc: float, Q_lat: float, Q_sh: float,
                            dT_max_sc: float, dT_max_lat: float, dT_max_sh: float):
        """
        Get segmented phases if segmentation is enabled, otherwise return phase totals.
        
        This method provides backward compatibility: if use_segmentation is False,
        it returns the original phase structure. If True, it returns segmented phases.
        
        Args:
            Q_sc (float): Heat for subcooling phase
            Q_lat (float): Heat for latent phase
            Q_sh (float): Heat for superheat phase
            dT_max_sc (float): Max temperature difference for subcooling
            dT_max_lat (float): Max temperature difference for latent phase
            dT_max_sh (float): Max temperature difference for superheat
            
        Returns:
            dict: Dictionary with segmented phases or original phases if segmentation disabled
        """
        if not self.use_segmentation:
            # Backward compatible: return original phase structure
            return {
                'sc': Q_sc,
                'lat': Q_lat,
                'sh': Q_sh,
                'dT_max': {'sc': dT_max_sc, 'lat': dT_max_lat, 'sh': dT_max_sh}
            }
        
        # Return segmented phases. If a geometric length is available, pass it so
        # that segments can report their representative length.
        segments_dict = self.segmentation.segment_all_phases(
            A_sc=Q_sc, A_lat=Q_lat, A_sh=Q_sh,
            dT_max_sc=dT_max_sc, dT_max_lat=dT_max_lat, dT_max_sh=dT_max_sh,
            length_total=getattr(self, "length_hx", None),
            d_h=getattr(self, "d_h", None)
        )
        return segments_dict
    
    def set_segmentation_params(self, n_segments_sc: int = None, n_segments_lat: int = None, 
                                n_segments_sh: int = None):
        """
        Update segmentation parameters at runtime.
        
        Args:
            n_segments_sc (int): Number of segments for subcooling (optional)
            n_segments_lat (int): Number of segments for latent phase (optional)
            n_segments_sh (int): Number of segments for superheat (optional)
        """
        if n_segments_sc is not None:
            self.segmentation.n_segments_sc = max(1, n_segments_sc)
        if n_segments_lat is not None:
            self.segmentation.n_segments_lat = max(1, n_segments_lat)
        if n_segments_sh is not None:
            self.segmentation.n_segments_sh = max(1, n_segments_sh)

    def iterate_area(self, dT_max, alpha_pri, alpha_sec, Q) -> float:
        """
        Iteratively calculates the required area for the heat exchange.

        Args:
            dT_max (float): Maximum temperature differential.
            alpha_pri (float): Heat transfer coefficient for the primary medium.
            alpha_sec (float): Heat transfer coefficient for the secondary medium.
            Q (float): Heat flow rate.

        Returns:
            float: Required area for heat exchange.
        """
        _accuracy = 1e-6  # square mm
        _step = 1.0
        R = self.calc_R()
        k = self.calc_k(alpha_pri, alpha_sec)
        m_flow_cp_min = self.calc_m_flow_cp_min()
        # First check if point is feasible at all
        if dT_max <= 0:
            return self.A
        eps_necessary = Q / (m_flow_cp_min * dT_max)

        # Special cases:
        # ---------------
        # eps is equal or higher than 1, an infinite amount of area would be necessary.
        if eps_necessary >= 1:
            return self.A
        # eps is lower or equal to zero: No Area required (Q<=0)
        if eps_necessary <= 0:
            return 0

        area = 0.0
        while True:
            NTU = self.calc_NTU(area, k, m_flow_cp_min)
            eps = self.calc_eps(R, NTU)
            if eps >= eps_necessary:
                if _step <= _accuracy:
                    break
                else:
                    # Go back
                    area -= _step
                    _step /= 10
                    continue
            if _step < _accuracy and area > self.A:
                break
            area += _step

        return min(area, self.A)

    def apply_segment_pressure_drops(self, segments_dict, m_flow, fs_state, OCR: float = 0.0):
        """
        Apply pressure drops to heat exchanger segments and recalculate outlet states.
        
        This method:
        1. Calculates pressure drop for each segment
        2. Updates segment outlet pressures
        3. Cascades pressure through phases (SH->LAT->SC for Condenser, SC->LAT->SH for Evaporator)
        4. Stores pressure drop information in FlowsheetState
        
        Args:
            segments_dict (dict): Dictionary with phase types as keys and segment lists as values
            m_flow (float): Mass flow rate in kg/s
            fs_state (FlowsheetState): FlowsheetState to store pressure drop information
        
        Returns:
            dict: Total pressure drops per phase, used for convergence checking
        """
        if (not self.pressure_drop_model and not any(
            [self.two_phase_pressure_drop, self.gas_pressure_drop, self.liquid_pressure_drop]
        )) or not segments_dict or not self.apply_pressure_drops:
            return {}
        
        # Define the order of phases for cascading pressure
        # Condenser: SH -> LAT -> SC (high to low pressure)
        # Evaporator: SC -> LAT -> SH (but we process in available order and track pressure)
        if self.__class__.__name__ == "MovingBoundaryNTUCondenser":
            phase_order = ['sh', 'lat', 'sc']
        else:  # Evaporator - process SC first, then LAT, then SH
            phase_order = ['sc', 'lat', 'sh']
        
        # Filter to only phases that exist in segments_dict
        phase_order = [p for p in phase_order if p in segments_dict]
        
        total_dp_per_phase = {}
        prev_phase_outlet_pressure = None
        prev_phase_outlet_state = None
        
        # Process phases in the defined order
        for phase_type in phase_order:
            if phase_type not in segments_dict:
                continue
            
            segments = segments_dict[phase_type]
            if not segments:
                continue
            # Skip pressure drops for phases with zero total segment heat (non-existent physically)
            try:
                phase_Q = sum(getattr(seg, 'Q', 0.0) for seg in segments)
            except Exception:
                phase_Q = None
            if phase_Q is not None and phase_Q <= 0:
                # Enforce continuity for degenerate phase: set all segment pressures
                # to previous phase outlet pressure if available
                if prev_phase_outlet_state is not None:
                    for seg in segments:
                        try:
                            if seg.state_inlet is not None:
                                seg.state_inlet = self.med_prop.calc_state("PH", prev_phase_outlet_state.p, seg.state_inlet.h)
                            if seg.state_outlet is not None:
                                seg.state_outlet = self.med_prop.calc_state("PH", prev_phase_outlet_state.p, seg.state_outlet.h)
                        except Exception:
                            pass
                # Maintain continuity markers even if skipping dp
                if segments and segments[-1].state_outlet is not None:
                    prev_phase_outlet_pressure = segments[-1].state_outlet.p
                    prev_phase_outlet_state = segments[-1].state_outlet
                total_dp_per_phase[phase_type] = 0.0
                continue
            
            total_dp = 0.0

            # Select phase-specific pressure-drop model if available
            if phase_type == 'lat':
                pd_model = self.two_phase_pressure_drop or self.pressure_drop_model
            elif phase_type == 'sh':
                pd_model = self.gas_pressure_drop or self.pressure_drop_model
            elif phase_type == 'sc':
                pd_model = self.liquid_pressure_drop or self.pressure_drop_model
            else:
                pd_model = self.pressure_drop_model
            
            # Cascade pressure drops through segments within a phase
            # AND from the previous phase to this phase
            for i, segment in enumerate(segments):
                # For the first segment of a new phase, enforce continuity with previous phase outlet
                if i == 0 and prev_phase_outlet_state is not None:
                    # Use the previous phase's outlet state directly to guarantee continuity
                    segment.state_inlet = prev_phase_outlet_state
                elif i == 0 and prev_phase_outlet_pressure is not None and segment.state_inlet is not None:
                    # Fallback: update inlet pressure using previous phase's outlet pressure
                    try:
                        h_inlet = segment.state_inlet.h
                        segment.state_inlet = self.med_prop.calc_state("PH", prev_phase_outlet_pressure, h_inlet)
                    except Exception:
                        # Keep original inlet state if recalculation fails
                        pass

                if segment.state_inlet is None or segment.state_outlet is None:
                    continue
                
                # Calculate pressure drop using the model and scale with segment length.
                # Allgemein:
                #   dp_ref = pd_model.calc(...)    → Phasen-Δp bezogen auf eine
                #                                    Referenzlänge L_ref.
                #   L_ref  = length_nominal (falls vorhanden), sonst evtl. length.
                #   L_seg  = Segmentlänge aus der Segmentierung.
                # Für ein Segment mit Länge L_seg gilt dann:
                #   Δp_seg = dp_ref * (L_seg / L_ref)
                dp = 0.0
                try:
                    if pd_model is not None:
                        dp_ref = pd_model.calc(None, m_flow)  # = Δp_ref bezogen auf L_ref
                        # Bevorzugt length_nominal (z.B. für QuadraticMassFlowDependent)
                        L_ref = getattr(pd_model, "length_nominal", None)
                        if L_ref in (None, 0):
                            # Fallback: ggf. length aus einfachen Modellen
                            L_ref = getattr(pd_model, "length", None)
                        L_seg = getattr(segment, "length", None)
                        if L_seg is not None and L_ref not in (None, 0):
                            dp = dp_ref * (L_seg / L_ref)
                        else:
                            # Fallback: keine Segmentlänge bekannt → Referenz-Δp
                            dp = dp_ref

                        # Oil-circulation-Ratio-Korrektur: Druckverlust * (1 + OCR)
                        if OCR:
                            dp *= (1.0 + OCR)
                except Exception:
                    dp = 0.0
                
                total_dp += dp
                
                # Update outlet pressure: apply pressure drop
                p_outlet_new = max(segment.state_inlet.p - dp, 1000)
                
                # Recalculate outlet state with new pressure but keep enthalpy
                h_outlet = segment.state_outlet.h
                try:
                    segment.state_outlet = self.med_prop.calc_state("PH", p_outlet_new, h_outlet)
                except:
                    pass
                
                # Cascade: Update next segment's inlet pressure to this segment's outlet pressure
                if i + 1 < len(segments):
                    next_segment = segments[i + 1]
                    if next_segment.state_inlet is not None:
                        try:
                            h_inlet_next = next_segment.state_inlet.h
                            next_segment.state_inlet = self.med_prop.calc_state("PH", p_outlet_new, h_inlet_next)
                        except:
                            pass
            
            # Store the last segment's outlet pressure for the next phase
            if segments and segments[-1].state_outlet is not None:
                prev_phase_outlet_pressure = segments[-1].state_outlet.p
                prev_phase_outlet_state = segments[-1].state_outlet
            
            total_dp_per_phase[phase_type] = total_dp
        
        # Get the final outlet pressure from the last processed phase
        final_outlet_pressure = None
        for phase_type in reversed(phase_order):
            if phase_type in segments_dict and segments_dict[phase_type]:
                last_segment = segments_dict[phase_type][-1]
                if last_segment.state_outlet is not None:
                    final_outlet_pressure = last_segment.state_outlet.p
                    break
        
        # Update the HX outlet state with the new pressure
        if final_outlet_pressure is not None and self.state_outlet is not None:
            try:
                self.state_outlet = self.med_prop.calc_state("PH", final_outlet_pressure, self.state_outlet.h)
            except:
                pass
        
        # Store pressure drop information in FlowsheetState
        total_dp_all = sum(total_dp_per_phase.values())
        fs_state.set(
            name=f"{self.__class__.__name__}_total_dp",
            value=total_dp_all,
            unit="Pa",
            description=f"Total pressure drop across {self.__class__.__name__} segments"
        )
        
        return total_dp_per_phase


class MovingBoundaryNTUCondenser(MovingBoundaryNTU):
    """
    Condenser class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def __init__(
        self,
        flow_type: str = "counter",
        ratio_outer_to_inner_area: float = 1.0,
        **kwargs,
    ):
        """Initialize condenser with sensible defaults.

        ``ratio_outer_to_inner_area`` will later be overwritten if a
        geometry object is provided via the ``geometry=...`` keyword in the
        base class.
        """

        super().__init__(
            flow_type=flow_type,
            ratio_outer_to_inner_area=ratio_outer_to_inner_area,
            **kwargs,
        )

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> tuple[float, float]:
        """
        Calculate the heat exchanger with the NTU-Method based on the given inputs.

        The flowsheet state can be used to save important variables
        during calculation for later analysis.

        Both return values are used to check if the heat transfer is valid or not.

        Args:
            inputs (Inputs): The inputs for the calculation.
            fs_state (FlowsheetState): The flowsheet state to save important variables.

        Returns:
            Tuple[float, float]:
                error: Error in percentage between the required and calculated heat flow rates.
                dT_min: Minimal temperature difference (can be negative).
        """
        self.m_flow_secondary = inputs.m_flow_con  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_con_in)

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_inlet,
            self.state_outlet,
            self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_con_ntu has to converge to Q_con (m_ref*delta_h), we can safely
        # calculate the output temperature.

        T_mean = inputs.T_con_in + self.calc_secondary_Q_flow(Q) / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # Calculate secondary_medium side temperatures:
        # Assumption loss is the same correlation for each regime
        T_sc = inputs.T_con_in + self.calc_secondary_Q_flow(Q_sc) / self.m_flow_secondary_cp
        T_sh = T_sc + self.calc_secondary_Q_flow(Q_lat) / self.m_flow_secondary_cp
        T_out = T_sh + self.calc_secondary_Q_flow(Q_sh) / self.m_flow_secondary_cp

        OCR = float(getattr(inputs, "OCR", 0.0) or 0.0)

        # 1. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        k_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            self.set_primary_cp((state_q0.h - self.state_outlet.h) / (state_q0.T - self.state_outlet.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            # Only use still available area:
            A_sc = self.iterate_area(dT_max=(state_q0.T - inputs.T_con_in),
                                     alpha_pri=alpha_ref_wall,
                                     alpha_sec=alpha_med_wall,
                                     Q=Q_sc)
            A_sc = min(self.A, A_sc)

            Q_sc_ntu, k_sc = self.calc_Q_ntu(dT_max=(state_q0.T - inputs.T_con_in),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sc)

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        k_lat = 0
        if Q_lat > 0:
            self.set_primary_cp(np.inf)
            # Get transport properties (primary side only: OCR-relevant):
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            A_lat = self.iterate_area(dT_max=(state_q1.T - T_sc),
                                      alpha_pri=alpha_ref_wall,
                                      alpha_sec=alpha_med_wall,
                                      Q=Q_lat)
            # Only use still available area:
            A_lat = min(self.A - A_sc, A_lat)

            Q_lat_ntu, k_lat = self.calc_Q_ntu(dT_max=(state_q1.T - T_sc),
                                               alpha_pri=alpha_ref_wall,
                                               alpha_sec=alpha_med_wall,
                                               A=A_lat)
            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        Q_sh_ntu, A_sh = 0, 0
        k_sh = 0
        if Q_sh and (self.state_inlet.T != state_q1.T):
            self.set_primary_cp((self.state_inlet.h - state_q1.h) / (self.state_inlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            # Only use still available area:
            A_sh = self.A - A_sc - A_lat

            Q_sh_ntu, k_sh = self.calc_Q_ntu(dT_max=(self.state_inlet.T - T_sh),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sh)
            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - inputs.T_con_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2", description="Area for subcooling heat exchange in condenser")

        # Store segmentation data if enabled
        if self.use_segmentation:
            # Build equal-area segments per phase
            segments_dict = self.segmentation.segment_all_phases(
                A_sc=A_sc, A_lat=A_lat, A_sh=A_sh,
                dT_max_sc=(state_q0.T - inputs.T_con_in) if Q_sc > 0 else 0,
                dT_max_lat=(state_q1.T - T_sc) if Q_lat > 0 else 0,
                dT_max_sh=(self.state_inlet.T - T_sh) if Q_sh > 0 else 0,
                length_total=getattr(self, "length_hx", None),
                d_h=getattr(self, "d_h", None)
            )

            # Prune phases with no physical presence (no area or no heat)
            if (A_sc <= 0) or (Q_sc <= 0):
                segments_dict.pop('sc', None)
            if (A_lat <= 0) or (Q_lat <= 0):
                segments_dict.pop('lat', None)
            if (A_sh <= 0) or (Q_sh <= 0):
                segments_dict.pop('sh', None)

            # Compute per-segment Q and states using equal area and local LMTD
            # flow_type aware secondary temperature bounds per phase
            is_counter = str(getattr(self, 'flow_type', 'counter')).lower() == 'counter'

            if 'sc' in segments_dict:
                if is_counter:
                    T_in_sc, T_out_sc = T_sc, inputs.T_con_in
                else:
                    T_in_sc, T_out_sc = inputs.T_con_in, T_sc
                segments_dict['sc'] = self._compute_segments_by_area(
                    'sc', segments_dict['sc'], state_q0, self.state_outlet,
                    T_sec_in=T_in_sc, T_sec_out=T_out_sc, k_phase=k_sc, Q_phase=Q_sc
                )
                if segments_dict['sc']:
                    total_Q_sc = sum(getattr(s, 'Q', 0.0) for s in segments_dict['sc'])
                    if total_Q_sc > 0:
                        fs_state.set(name="segments_con_sc", value=segments_dict['sc'], unit="-",
                                     description="Condenser subcooling phase segments")
                    else:
                        # Remove degenerate phase: zero heat after segmentation
                        segments_dict.pop('sc', None)

            if 'lat' in segments_dict:
                if is_counter:
                    T_in_lat, T_out_lat = T_sh, T_sc
                else:
                    T_in_lat, T_out_lat = T_sc, T_sh
                segments_dict['lat'] = self._compute_segments_by_area(
                    'lat', segments_dict['lat'], state_q1, state_q0,
                    T_sec_in=T_in_lat, T_sec_out=T_out_lat, k_phase=k_lat, Q_phase=Q_lat
                )
                if segments_dict['lat']:
                    total_Q_lat = sum(getattr(s, 'Q', 0.0) for s in segments_dict['lat'])
                    if total_Q_lat > 0:
                        fs_state.set(name="segments_con_lat", value=segments_dict['lat'], unit="-",
                                     description="Condenser latent phase segments")
                    else:
                        segments_dict.pop('lat', None)

            if 'sh' in segments_dict:
                if is_counter:
                    T_in_sh, T_out_sh = T_out, T_sh
                else:
                    T_in_sh, T_out_sh = T_sh, T_out
                segments_dict['sh'] = self._compute_segments_by_area(
                    'sh', segments_dict['sh'], self.state_inlet, state_q1,
                    T_sec_in=T_in_sh, T_sec_out=T_out_sh, k_phase=k_sh, Q_phase=Q_sh
                )
                if segments_dict['sh']:
                    total_Q_sh = sum(getattr(s, 'Q', 0.0) for s in segments_dict['sh'])
                    if total_Q_sh > 0:
                        fs_state.set(name="segments_con_sh", value=segments_dict['sh'], unit="-",
                                     description="Condenser superheat phase segments")
                    else:
                        segments_dict.pop('sh', None)

            # Apply segment pressure drops and write back updated segments
            if self.pressure_drop_model:
                self.apply_segment_pressure_drops(segments_dict, self.m_flow, fs_state)
                # Re-save possibly updated segments to flowsheet state so downstream tools see cascaded pressures
                if 'sc' in segments_dict:
                    fs_state.set(name="segments_con_sc", value=segments_dict['sc'], unit="-",
                                 description="Condenser subcooling phase segments (updated with pressure drops)")
                if 'lat' in segments_dict:
                    fs_state.set(name="segments_con_lat", value=segments_dict['lat'], unit="-",
                                 description="Condenser latent phase segments (updated with pressure drops)")
                if 'sh' in segments_dict:
                    fs_state.set(name="segments_con_sh", value=segments_dict['sh'], unit="-",
                                 description="Condenser superheat phase segments (updated with pressure drops)")

            # Enforce phase-boundary continuity (final safety net)
            try:
                if 'sh' in segments_dict and segments_dict['sh'] and 'lat' in segments_dict and segments_dict['lat']:
                    segments_dict['lat'][0].state_inlet = segments_dict['sh'][-1].state_outlet
                if 'lat' in segments_dict and segments_dict['lat'] and 'sc' in segments_dict and segments_dict['sc']:
                    segments_dict['sc'][0].state_inlet = segments_dict['lat'][-1].state_outlet
                # Save again to ensure continuity is persisted
                if 'sc' in segments_dict:
                    fs_state.set(name="segments_con_sc", value=segments_dict['sc'], unit="-",
                                 description="Condenser subcooling phase segments (boundary-aligned)")
                if 'lat' in segments_dict:
                    fs_state.set(name="segments_con_lat", value=segments_dict['lat'], unit="-",
                                 description="Condenser latent phase segments (boundary-aligned)")
                if 'sh' in segments_dict:
                    fs_state.set(name="segments_con_sh", value=segments_dict['sh'], unit="-",
                                 description="Condenser superheat phase segments (boundary-aligned)")
            except Exception:
                pass

            # Additional check: dT_min within segments (pinchpoint) using the same
            # secondary-side bounds used during segment computation
            dT_min_segments = float('inf')
            for phase_key in ['sh', 'lat', 'sc']:
                segs = segments_dict.get(phase_key, [])
                if not segs:
                    continue
                n = len(segs)
                # Determine secondary inlet/outlet temps for this phase
                if phase_key == 'sh':
                    T_pair = (T_out, T_sh) if is_counter else (T_sh, T_out)
                elif phase_key == 'lat':
                    T_pair = (T_sh, T_sc) if is_counter else (T_sc, T_sh)
                else:  # 'sc'
                    T_pair = (T_sc, inputs.T_con_in) if is_counter else (inputs.T_con_in, T_sc)
                T_s_in_phase, T_s_out_phase = T_pair
                # Bounds along segments as used in _compute_segments_by_area
                T_bounds = np.linspace(T_s_in_phase, T_s_out_phase, n + 1)
                for i, seg in enumerate(segs):
                    if seg.state_inlet and seg.state_outlet:
                        T_s_in_i = T_bounds[i]
                        T_s_out_i = T_bounds[i + 1]
                        dT_seg_in = seg.state_inlet.T - T_s_in_i
                        dT_seg_out = seg.state_outlet.T - T_s_out_i
                        dT_min_segments = min(dT_min_segments, dT_seg_in, dT_seg_out)

            # Include segment-level dT_min in overall check
            # Also include SC↔LAT boundary explicitly
            dT_min_ScLat = state_q0.T - T_sc
            if dT_min_segments != float('inf'):
                dT_min_overall = min(dT_min_in, dT_min_LatSH, dT_min_ScLat, dT_min_out, dT_min_segments)
            else:
                dT_min_overall = min(dT_min_in, dT_min_LatSH, dT_min_ScLat, dT_min_out)
        else:
            # No segmentation: include both internal boundaries
            dT_min_ScLat = state_q0.T - T_sc
            dT_min_overall = min(dT_min_in, dT_min_LatSH, dT_min_ScLat, dT_min_out)

        error = self._apply_pinch_penalty(error, dT_min_overall, drives_lower_pressure=False)
        return error, dT_min_overall


class MovingBoundaryNTUEvaporator(MovingBoundaryNTU):
    """
    Evaporator class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def __init__(
        self,
        flow_type: str = "counter",
        ratio_outer_to_inner_area: float = 1.0,
        **kwargs,
    ):
        """Initialize evaporator with sensible defaults.

        ``ratio_outer_to_inner_area`` will later be overwritten if a
        geometry object is provided via the ``geometry=...`` keyword in the
        base class.
        """

        super().__init__(
            flow_type=flow_type,
            ratio_outer_to_inner_area=ratio_outer_to_inner_area,
            **kwargs,
        )

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> tuple[float, float]:
        """
        Calculate the heat exchanger with the NTU-Method based on the given inputs.

        The flowsheet state can be used to save important variables
        during calculation for later analysis.

        Both return values are used to check if the heat transfer is valid or not.

        Args:
            inputs (Inputs): The inputs for the calculation.
            fs_state (FlowsheetState): The flowsheet state to save important variables.

        Returns:
            Tuple[float, float]:
                error: Error in percentage between the required and calculated heat flow rates.
                dT_min: Minimal temperature difference (can be negative).
        """
        self.m_flow_secondary = inputs.m_flow_eva  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_eva_in)

        # First we separate the flow (using local saturation at evaporator inlet pressure
        # to obtain sharper phase boundaries for subcooling / two-phase / superheat):
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_outlet,
            self.state_inlet,
            self.state_inlet.p
        )

        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_eva_ntu has to converge to Q_eva (m_ref*delta_h), we can safely
        # calculate the output temperature.
        T_mean = inputs.T_eva_in - Q / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # Calculate secondary_medium side temperatures:
        T_sh = inputs.T_eva_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        OCR = float(getattr(inputs, "OCR", 0.0) or 0.0)

        # 1. Regime: Superheating
        Q_sh_ntu, A_sh = 0, 0
        k_sh = 0
        if Q_sh and (self.state_outlet.T != state_q1.T):
            self.set_primary_cp((self.state_outlet.h - state_q1.h) / (self.state_outlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            if Q_lat > 0:
                A_sh = self.iterate_area(dT_max=(inputs.T_eva_in - state_q1.T),
                                         alpha_pri=alpha_ref_wall,
                                         alpha_sec=alpha_med_wall,
                                         Q=Q_sh)
            else:
                # if only sh is present --> full area:
                A_sh = self.A

            # Only use still available area
            A_sh = min(self.A, A_sh)

            Q_sh_ntu, k_sh = self.calc_Q_ntu(dT_max=(inputs.T_eva_in - state_q1.T),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sh)

            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        k_lat = 0
        if Q_lat > 0:
            self.set_primary_cp(np.inf)

            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            if Q_sc > 0:
                A_lat = self.iterate_area(dT_max=(T_sh - self.state_inlet.T),
                                          alpha_pri=alpha_ref_wall,
                                          alpha_sec=alpha_med_wall,
                                          Q=Q_lat)
            else:
                A_lat = self.A - A_sh

            # Only use still available area:
            A_lat = min(self.A - A_sh, A_lat)
            Q_lat_ntu, k_lat = self.calc_Q_ntu(dT_max=(T_sh - self.state_inlet.T),
                                               alpha_pri=alpha_ref_wall,
                                               alpha_sec=alpha_med_wall,
                                               A=A_lat)
            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        k_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            self.set_primary_cp((state_q0.h - self.state_inlet.h) / (state_q0.T - self.state_inlet.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)
            if OCR:
                alpha_ref_wall *= (1.0 - OCR)

            # Only use still available area:
            A_sc = self.A - A_sh - A_lat

            Q_sc_ntu, k_sc = self.calc_Q_ntu(dT_max=(T_sc - self.state_inlet.T),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sc)

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get dT_min
        dT_min_in = inputs.T_eva_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in evaporator")

        # Store segmentation data if enabled
        if self.use_segmentation:
            # Build equal-area segments per phase
            segments_dict = self.segmentation.segment_all_phases(
                A_sc=A_sc, A_lat=A_lat, A_sh=A_sh,
                dT_max_sc=(T_sc - self.state_inlet.T) if Q_sc > 0 else 0,
                dT_max_lat=(T_sh - self.state_inlet.T) if Q_lat > 0 else 0,
                dT_max_sh=(inputs.T_eva_in - state_q1.T) if Q_sh > 0 else 0,
                length_total=getattr(self, "length_hx", None),
                d_h=getattr(self, "d_h", None)
            )

            # Prune phases with no physical presence (no area or no heat)
            if (A_sc <= 0) or (Q_sc <= 0):
                segments_dict.pop('sc', None)
            if (A_lat <= 0) or (Q_lat <= 0):
                segments_dict.pop('lat', None)
            if (A_sh <= 0) or (Q_sh <= 0):
                segments_dict.pop('sh', None)

            # Calculate per-segment Q and states using equal area and local LMTD
            is_counter = str(getattr(self, 'flow_type', 'counter')).lower() == 'counter'

            if 'sh' in segments_dict:
                if is_counter:
                    T_in_sh, T_out_sh = T_sh, inputs.T_eva_in
                else:
                    T_in_sh, T_out_sh = inputs.T_eva_in, T_sh
                segments_dict['sh'] = self._compute_segments_by_area(
                    'sh', segments_dict['sh'], state_q1, self.state_outlet,
                    T_sec_in=T_in_sh, T_sec_out=T_out_sh, k_phase=k_sh, Q_phase=Q_sh
                )
                if segments_dict['sh']:
                    total_Q_sh = sum(getattr(s, 'Q', 0.0) for s in segments_dict['sh'])
                    if total_Q_sh > 0:
                        fs_state.set(name="segments_eva_sh", value=segments_dict['sh'], unit="-",
                                     description="Evaporator superheat phase segments")
                    else:
                        segments_dict.pop('sh', None)

            if 'lat' in segments_dict:
                eva_lat_inlet = self.state_inlet  # Expansion valve outlet
                if is_counter:
                    T_in_lat, T_out_lat = T_sc, T_sh
                else:
                    T_in_lat, T_out_lat = T_sh, T_sc
                segments_dict['lat'] = self._compute_segments_by_area(
                    'lat', segments_dict['lat'], eva_lat_inlet, state_q1,
                    T_sec_in=T_in_lat, T_sec_out=T_out_lat, k_phase=k_lat, Q_phase=Q_lat
                )
                if segments_dict['lat']:
                    total_Q_lat = sum(getattr(s, 'Q', 0.0) for s in segments_dict['lat'])
                    if total_Q_lat > 0:
                        fs_state.set(name="segments_eva_lat", value=segments_dict['lat'], unit="-",
                                     description="Evaporator latent phase segments")
                    else:
                        segments_dict.pop('lat', None)

            if 'sc' in segments_dict:
                if is_counter:
                    T_in_sc, T_out_sc = T_out, T_sc
                else:
                    T_in_sc, T_out_sc = T_sc, T_out
                segments_dict['sc'] = self._compute_segments_by_area(
                    'sc', segments_dict['sc'], self.state_inlet, state_q0,
                    T_sec_in=T_in_sc, T_sec_out=T_out_sc, k_phase=k_sc, Q_phase=Q_sc
                )
                if segments_dict['sc']:
                    total_Q_sc = sum(getattr(s, 'Q', 0.0) for s in segments_dict['sc'])
                    if total_Q_sc > 0:
                        fs_state.set(name="segments_eva_sc", value=segments_dict['sc'], unit="-",
                                     description="Evaporator subcooling phase segments")
                    else:
                        segments_dict.pop('sc', None)

            # Apply segment pressure drops and write back updated segments
            if self.pressure_drop_model:
                self.apply_segment_pressure_drops(segments_dict, self.m_flow, fs_state)
                if 'sc' in segments_dict:
                    fs_state.set(name="segments_eva_sc", value=segments_dict['sc'], unit="-",
                                 description="Evaporator subcooling phase segments (updated with pressure drops)")
                if 'lat' in segments_dict:
                    fs_state.set(name="segments_eva_lat", value=segments_dict['lat'], unit="-",
                                 description="Evaporator latent phase segments (updated with pressure drops)")
                if 'sh' in segments_dict:
                    fs_state.set(name="segments_eva_sh", value=segments_dict['sh'], unit="-",
                                 description="Evaporator superheat phase segments (updated with pressure drops)")

            # Enforce phase-boundary continuity (final safety net)
            try:
                if 'sc' in segments_dict and segments_dict['sc'] and 'lat' in segments_dict and segments_dict['lat']:
                    segments_dict['lat'][0].state_inlet = segments_dict['sc'][-1].state_outlet
                if 'lat' in segments_dict and segments_dict['lat'] and 'sh' in segments_dict and segments_dict['sh']:
                    segments_dict['sh'][0].state_inlet = segments_dict['lat'][-1].state_outlet
                if 'sc' in segments_dict:
                    fs_state.set(name="segments_eva_sc", value=segments_dict['sc'], unit="-",
                                 description="Evaporator subcooling phase segments (boundary-aligned)")
                if 'lat' in segments_dict:
                    fs_state.set(name="segments_eva_lat", value=segments_dict['lat'], unit="-",
                                 description="Evaporator latent phase segments (boundary-aligned)")
                if 'sh' in segments_dict:
                    fs_state.set(name="segments_eva_sh", value=segments_dict['sh'], unit="-",
                                 description="Evaporator superheat phase segments (boundary-aligned)")
            except Exception:
                pass

            # Additional check: dT_min within segments (pinchpoint) using the same
            # secondary-side bounds used during segment computation
            dT_min_segments = float('inf')
            for phase_key in ['sc', 'lat', 'sh']:
                segs = segments_dict.get(phase_key, [])
                if not segs:
                    continue
                n = len(segs)
                # Determine secondary inlet/outlet temps for this phase
                if phase_key == 'sh':
                    T_pair = (T_sh, inputs.T_eva_in) if is_counter else (inputs.T_eva_in, T_sh)
                elif phase_key == 'lat':
                    T_pair = (T_sc, T_sh) if is_counter else (T_sh, T_sc)
                else:  # 'sc'
                    T_pair = (T_out, T_sc) if is_counter else (T_sc, T_out)
                T_s_in_phase, T_s_out_phase = T_pair
                # Bounds along segments as used in _compute_segments_by_area
                T_bounds = np.linspace(T_s_in_phase, T_s_out_phase, n + 1)
                for i, seg in enumerate(segs):
                    if seg.state_inlet and seg.state_outlet:
                        T_s_in_i = T_bounds[i]
                        T_s_out_i = T_bounds[i + 1]
                        # Evaporator feasibility: T_secondary - T_refrigerant must be > 0
                        dT_seg_in = T_s_in_i - seg.state_inlet.T
                        dT_seg_out = T_s_out_i - seg.state_outlet.T
                        dT_min_segments = min(dT_min_segments, dT_seg_in, dT_seg_out)

            # Include segment-level dT_min in overall check
            # Also include both internal boundaries explicitly
            dT_min_ScLat = T_sc - state_q0.T
            dT_min_LatSh = T_sh - state_q1.T
            if dT_min_segments != float('inf'):
                dT_min_overall = min(dT_min_out, dT_min_in, dT_min_ScLat, dT_min_LatSh, dT_min_segments)
            else:
                dT_min_overall = min(dT_min_out, dT_min_in, dT_min_ScLat, dT_min_LatSh)
        else:
            # No segmentation: include internal boundaries as well
            dT_min_ScLat = T_sc - state_q0.T
            dT_min_LatSh = T_sh - state_q1.T
            dT_min_overall = min(dT_min_out, dT_min_in, dT_min_ScLat, dT_min_LatSh)

        error = self._apply_pinch_penalty(error, dT_min_overall, drives_lower_pressure=True)
        return error, dT_min_overall

