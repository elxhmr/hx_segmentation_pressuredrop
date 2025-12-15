import abc
import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.ntu import BasicNTU
from vclibpy.components.heat_exchangers.segmentation import SegmentResult
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


class MovingBoundaryNTU(BasicNTU, abc.ABC):
    """
    Moving boundary NTU based heat exchanger.

    See parent classe for arguments.
    """

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

    def _calc_segment_dp(self, segment_area: float, state_in, state_out, dp_phase: str | None) -> float:
        """Calculate pressure drop for a single segment.

        The base pressure-drop model returns a total Δp for the full geometry.
        We scale the contribution by the segment's share of the total area so
        that the sum of all segments reproduces the configured model when
        ``A_phase`` equals the heat-exchanger area.
        """

        model = None
        if dp_phase == "liquid":
            model = self.dp_model_liquid or self.dp_model
        elif dp_phase == "gas":
            model = self.dp_model_gas or self.dp_model
        elif dp_phase == "two_phase":
            model = self.dp_model_two_phase or self.dp_model
        else:
            model = self.dp_model

        if not model or self.A <= 0:
            return 0.0

        transport_properties = self.med_prop.calc_mean_transport_properties(state_in, state_out)
        base_dp = model.calc(transport_properties=transport_properties, m_flow=self.m_flow)
        if base_dp <= 0:
            return 0.0

        return base_dp * max(segment_area, 0) / self.A

    def _segment_phase(
            self,
            phase: str,
            n_segments: int,
            h_start: float,
            h_end: float,
            p_start: float,
            T_secondary_in: float,
            alpha_pri: float,
            alpha_sec: float,
            A_phase: float,
            dp_phase: str | None,
    ):
        """Forward-integrate one phase into equal subsegments."""

        if n_segments <= 0 or A_phase <= 0:
            return [], h_start, p_start, T_secondary_in, 0.0, None, 0.0

        segment_area = A_phase / n_segments
        h_current = h_start
        p_current = p_start
        T_sec_current = T_secondary_in
        segments = []
        total_Q_phase = 0.0
        min_dT_phase = None
        dp_phase_total = 0.0

        for seg_index in range(n_segments):
            # Linear predictor towards the end-of-phase enthalpy
            h_target = h_start + (h_end - h_start) * ((seg_index + 1) / n_segments)
            h_out_guess = h_target
            dT_max = 0.0
            dT_min_local = 0.0
            Q_ntu = 0.0

            for _ in range(self.segmentation.max_iter):
                state_in = self.med_prop.calc_state("PH", p_current, h_current)
                state_out = self.med_prop.calc_state("PH", p_current, h_out_guess)
                Q_guess = self.m_flow * (h_out_guess - h_current)
                T_secondary_out_guess = T_sec_current + self.calc_secondary_Q_flow(Q_guess) / self.m_flow_secondary_cp
                dT_max = max(
                    state_in.T - T_sec_current,
                    state_out.T - T_secondary_out_guess,
                )
                if dT_max < 0:
                    dT_max = abs(dT_max)

                Q_ntu, _ = self.calc_Q_ntu(
                    dT_max=dT_max,
                    alpha_pri=alpha_pri,
                    alpha_sec=alpha_sec,
                    A=segment_area,
                )

                h_out_new = h_current + Q_ntu / self.m_flow
                if h_end >= h_start:
                    h_out_new = min(max(h_out_new, h_current), h_end)
                else:
                    h_out_new = max(min(h_out_new, h_current), h_end)
                if abs(h_out_new - h_out_guess) < self.segmentation.tol:
                    h_out_guess = h_out_new
                    state_out = self.med_prop.calc_state("PH", p_current, h_out_guess)
                    T_secondary_out_guess = (
                        T_sec_current + self.calc_secondary_Q_flow(Q_ntu) / self.m_flow_secondary_cp
                    )
                    dT_min_local = min(
                        state_in.T - T_sec_current,
                        state_out.T - T_secondary_out_guess,
                    )
                    break

                h_out_guess = (
                    self.segmentation.relaxation * h_out_new
                    + (1 - self.segmentation.relaxation) * h_out_guess
                )

            # Finalize segment with the last guess
            state_out = self.med_prop.calc_state("PH", p_current, h_out_guess)
            Q_ntu = self.m_flow * (h_out_guess - h_current)
            T_secondary_out = T_sec_current + self.calc_secondary_Q_flow(Q_ntu) / self.m_flow_secondary_cp
            dp_segment = self._calc_segment_dp(segment_area, state_in, state_out, dp_phase)
            p_out = max(p_current - dp_segment, 1e-9)
            state_out = self.med_prop.calc_state("PH", p_out, h_out_guess)
            dT_min_local = min(
                state_in.T - T_sec_current,
                state_out.T - T_secondary_out,
            )
            segment = SegmentResult(
                phase=phase,
                segment_index=seg_index,
                total_segments=n_segments,
                Q=Q_ntu,
                dT_max=dT_max,
                dT_min=dT_min_local,
                state_inlet=state_in,
                state_outlet=state_out,
                p_inlet=p_current,
                p_outlet=p_out,
            )
            segments.append(segment)
            h_current = h_out_guess
            p_current = p_out
            T_sec_current = T_secondary_out
            total_Q_phase += Q_ntu
            dp_phase_total += dp_segment
            if min_dT_phase is None or dT_min_local < min_dT_phase:
                min_dT_phase = dT_min_local

        for segment in segments:
            segment.total_phase_heat = total_Q_phase

        return segments, h_current, p_current, T_sec_current, total_Q_phase, min_dT_phase, dp_phase_total

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


class MovingBoundaryNTUCondenser(MovingBoundaryNTU):
    """
    Condenser class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
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
        if self.uses_segmented_mode():
            return self._calc_segmented_condenser(inputs=inputs, fs_state=fs_state)

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

        # 1. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            self.set_primary_cp((state_q0.h - self.state_outlet.h) / (state_q0.T - self.state_outlet.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

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
        if Q_lat > 0:
            self.set_primary_cp(np.inf)
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

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
        if Q_sh and (self.state_inlet.T != state_q1.T):
            self.set_primary_cp((self.state_inlet.h - state_q1.h) / (self.state_inlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

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

        return error, min(dT_min_in,
                          dT_min_LatSH,
                          dT_min_out)

    def _calc_segmented_condenser(self, inputs: Inputs, fs_state: FlowsheetState):
        self.m_flow_secondary = inputs.m_flow_con  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_con_in)

        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_inlet,
            self.state_outlet,
            self.state_inlet.p
        )
        Q_total = Q_sc + Q_lat + Q_sh

        T_mean = inputs.T_con_in + self.calc_secondary_Q_flow(Q_total) / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        segments_sc = []
        segments_lat = []
        segments_sh = []

        A_sc = A_lat = A_sh = 0.0
        min_dT_global = None
        dp_total = 0.0
        p_current = self.state_inlet.p

        # 1) Subcooling
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            self.set_primary_cp((state_q0.h - self.state_outlet.h) / (state_q0.T - self.state_outlet.T))
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall_sc = self.calc_alpha_liquid(tra_prop_ref_con)

            A_sc = self.iterate_area(
                dT_max=(state_q0.T - inputs.T_con_in),
                alpha_pri=alpha_ref_wall_sc,
                alpha_sec=alpha_med_wall,
                Q=Q_sc,
            )
            A_sc = min(self.A, A_sc)

            segments_sc, h_current, p_current, T_secondary, Q_sc_seg, min_dT_sc, dp_sc = self._segment_phase(
                phase="sc",
                n_segments=self.segmentation.N_sc,
                h_start=self.state_outlet.h,
                h_end=state_q0.h,
                p_start=p_current,
                T_secondary_in=inputs.T_con_in,
                alpha_pri=alpha_ref_wall_sc,
                alpha_sec=alpha_med_wall,
                A_phase=A_sc,
                dp_phase="liquid",
            )
            min_dT_global = min_dT_sc
            dp_total += dp_sc
        else:
            h_current = self.state_outlet.h
            p_current = self.state_inlet.p
            T_secondary = inputs.T_con_in
            Q_sc_seg = 0.0

        # 2) Latent
        if Q_lat > 0:
            self.set_primary_cp(np.inf)
            alpha_ref_wall_lat = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs,
            )

            A_lat = self.iterate_area(
                dT_max=(state_q1.T - (T_secondary if Q_sc > 0 else inputs.T_con_in)),
                alpha_pri=alpha_ref_wall_lat,
                alpha_sec=alpha_med_wall,
                Q=Q_lat,
            )
            A_lat = min(self.A - A_sc, A_lat)

            segments_lat, h_current, p_current, T_secondary, Q_lat_seg, min_dT_lat, dp_lat = self._segment_phase(
                phase="lat",
                n_segments=self.segmentation.N_lat,
                h_start=h_current,
                h_end=state_q1.h,
                p_start=p_current,
                T_secondary_in=T_secondary,
                alpha_pri=alpha_ref_wall_lat,
                alpha_sec=alpha_med_wall,
                A_phase=A_lat,
                dp_phase="two_phase",
            )
            if min_dT_lat is not None:
                min_dT_global = min(min_dT_global, min_dT_lat) if min_dT_global is not None else min_dT_lat
            dp_total += dp_lat
            p_current = segments_lat[-1].p_outlet if segments_lat else p_current
        else:
            Q_lat_seg = 0.0

        # 3) Superheat
        if Q_sh and (self.state_inlet.T != state_q1.T):
            self.set_primary_cp((self.state_inlet.h - state_q1.h) / (self.state_inlet.T - state_q1.T))
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall_sh = self.calc_alpha_gas(tra_prop_ref_con)

            A_sh = self.A - A_sc - A_lat

            segments_sh, h_current, p_current, T_secondary, Q_sh_seg, min_dT_sh, dp_sh = self._segment_phase(
                phase="sh",
                n_segments=self.segmentation.N_sh,
                h_start=h_current,
                h_end=self.state_inlet.h,
                p_start=p_current,
                T_secondary_in=T_secondary,
                alpha_pri=alpha_ref_wall_sh,
                alpha_sec=alpha_med_wall,
                A_phase=A_sh,
                dp_phase="gas",
            )
            if min_dT_sh is not None:
                min_dT_global = min(min_dT_global, min_dT_sh) if min_dT_global is not None else min_dT_sh
            dp_total += dp_sh
        else:
            Q_sh_seg = 0.0

        Q_segments = Q_sc_seg + Q_lat_seg + Q_sh_seg
        error = (Q_segments / Q_total - 1) * 100 if Q_total else 0.0

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2", description="Area for subcooling heat exchange in condenser")
        fs_state.set(name="segments_con_sc", value=segments_sc)
        fs_state.set(name="segments_con_lat", value=segments_lat)
        fs_state.set(name="segments_con_sh", value=segments_sh)
        fs_state.set(name="dp_con_total", value=dp_total, unit="Pa", description="Total condenser pressure drop")

        return error, min_dT_global or 0.0


class MovingBoundaryNTUEvaporator(MovingBoundaryNTU):
    """
    Evaporator class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
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
        if self.uses_segmented_mode():
            return self._calc_segmented_evaporator(inputs=inputs, fs_state=fs_state)

        self.m_flow_secondary = inputs.m_flow_eva  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_eva_in)

        # First we separate the flow:
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

        # 1. Regime: Superheating
        Q_sh_ntu, A_sh = 0, 0
        if Q_sh and (self.state_outlet.T != state_q1.T):
            self.set_primary_cp((self.state_outlet.h - state_q1.h) / (self.state_outlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

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
        if Q_lat > 0:
            self.set_primary_cp(np.inf)

            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

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
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            self.set_primary_cp((state_q0.h - self.state_inlet.h) / (state_q0.T - self.state_inlet.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

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

        return error, min(dT_min_out, dT_min_in)

    def _calc_segmented_evaporator(self, inputs: Inputs, fs_state: FlowsheetState):
        self.m_flow_secondary = inputs.m_flow_eva  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_eva_in)

        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_outlet,
            self.state_inlet,
            self.state_inlet.p
        )
        Q_total = Q_sc + Q_lat + Q_sh

        T_mean = inputs.T_eva_in - Q_total / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        segments_sc = []
        segments_lat = []
        segments_sh = []
        A_sc = A_lat = A_sh = 0.0
        min_dT_global = None
        dp_total = 0.0

        # The segmented evaporator integrates from the inlet (liquid) to the outlet (superheated)
        h_current = self.state_inlet.h
        p_current = self.state_inlet.p
        T_secondary = inputs.T_eva_in

        # 1) Subcooling portion
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            self.set_primary_cp((state_q0.h - self.state_inlet.h) / (state_q0.T - self.state_inlet.T))
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall_sc = self.calc_alpha_liquid(tra_prop_ref_eva)

            A_sc = self.A  # remaining area will be restricted by later phases

            segments_sc, h_current, p_current, T_secondary, Q_sc_seg, min_dT_sc, dp_sc = self._segment_phase(
                phase="sc",
                n_segments=self.segmentation.N_sc,
                h_start=h_current,
                h_end=state_q0.h,
                p_start=p_current,
                T_secondary_in=T_secondary,
                alpha_pri=alpha_ref_wall_sc,
                alpha_sec=alpha_med_wall,
                A_phase=A_sc,
                dp_phase="liquid",
            )
            min_dT_global = min_dT_sc
            dp_total += dp_sc
            p_current = segments_sc[-1].p_outlet if segments_sc else p_current
        else:
            Q_sc_seg = 0.0

        # 2) Latent portion
        if Q_lat > 0:
            self.set_primary_cp(np.inf)
            alpha_ref_wall_lat = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs,
            )

            A_lat = max(self.A - A_sc, 0)

            segments_lat, h_current, p_current, T_secondary, Q_lat_seg, min_dT_lat, dp_lat = self._segment_phase(
                phase="lat",
                n_segments=self.segmentation.N_lat,
                h_start=h_current,
                h_end=state_q1.h,
                p_start=p_current,
                T_secondary_in=T_secondary,
                alpha_pri=alpha_ref_wall_lat,
                alpha_sec=alpha_med_wall,
                A_phase=A_lat if A_lat > 0 else self.A,
                dp_phase="two_phase",
            )
            if min_dT_lat is not None:
                min_dT_global = min(min_dT_global, min_dT_lat) if min_dT_global is not None else min_dT_lat
            dp_total += dp_lat
            p_current = segments_lat[-1].p_outlet if segments_lat else p_current
        else:
            Q_lat_seg = 0.0

        # 3) Superheat
        if Q_sh and (self.state_outlet.T != state_q1.T):
            self.set_primary_cp((self.state_outlet.h - state_q1.h) / (self.state_outlet.T - state_q1.T))
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall_sh = self.calc_alpha_gas(tra_prop_ref_eva)

            A_sh = max(self.A - A_sc - A_lat, 0)

            segments_sh, h_current, p_current, T_secondary, Q_sh_seg, min_dT_sh, dp_sh = self._segment_phase(
                phase="sh",
                n_segments=self.segmentation.N_sh,
                h_start=h_current,
                h_end=self.state_outlet.h,
                p_start=p_current,
                T_secondary_in=T_secondary,
                alpha_pri=alpha_ref_wall_sh,
                alpha_sec=alpha_med_wall,
                A_phase=A_sh if A_sh > 0 else self.A,
                dp_phase="gas",
            )
            if min_dT_sh is not None:
                min_dT_global = min(min_dT_global, min_dT_sh) if min_dT_global is not None else min_dT_sh
            dp_total += dp_sh
        else:
            Q_sh_seg = 0.0

        Q_segments = Q_sc_seg + Q_lat_seg + Q_sh_seg
        error = (Q_segments / Q_total - 1) * 100 if Q_total else 0.0

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in evaporator")
        fs_state.set(name="A_eva_sc", value=A_sc, unit="m2", description="Area for subcooling heat exchange in evaporator")
        fs_state.set(name="segments_eva_sc", value=segments_sc)
        fs_state.set(name="segments_eva_lat", value=segments_lat)
        fs_state.set(name="segments_eva_sh", value=segments_sh)
        fs_state.set(name="dp_eva_total", value=dp_total, unit="Pa", description="Total evaporator pressure drop")

        return error, min_dT_global or 0.0
