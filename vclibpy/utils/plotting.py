import pathlib
from typing import List, Optional, Dict

import numpy as np
import matplotlib.pyplot as plt
import sdf

from vclibpy.media import ThermodynamicState, MedProp


def _dedup_consecutive_states(states: List[ThermodynamicState], tol_h: float = 50.0, tol_p: float = 100.0) -> List[ThermodynamicState]:
    """
    Remove consecutive near-identical states to avoid duplicate points in plots.

    Args:
        states: Ordered list of states.
        tol_h: Enthalpy tolerance in J/kg for considering two consecutive states identical.
        tol_p: Pressure tolerance in Pa for considering two consecutive states identical.

    Returns:
        Filtered list with consecutive near-duplicates removed.
    """
    if not states:
        return states

    filtered = [states[0]]
    for s in states[1:]:
        prev = filtered[-1]
        if abs((s.h or 0) - (prev.h or 0)) < tol_h and abs((s.p or 0) - (prev.p or 0)) < tol_p:
            continue
        filtered.append(s)
    return filtered


def _collect_segments_from_fs(fs_state) -> Dict[str, List]:
    """
    Collect condenser and evaporator segments from FlowsheetState into a dict.

    Expected keys (if present):
      - segments_con_sc, segments_con_lat, segments_con_sh
      - segments_eva_sc, segments_eva_lat, segments_eva_sh
    """
    segments: Dict[str, List] = {}
    if not fs_state or not hasattr(fs_state, 'get_variable_names'):
        return segments

    names = fs_state.get_variable_names()
    for key in (
        "segments_con_sc", "segments_con_lat", "segments_con_sh",
        "segments_eva_sc", "segments_eva_lat", "segments_eva_sh",
    ):
        if key in names:
            var_obj = fs_state.get(key)
            segments[key] = getattr(var_obj, 'value', var_obj)
    return segments


def build_states_for_plot_from_flowsheet(
    flowsheet,
    fs_state=None,
    include_segment_inlets: bool = False,
    dedup: bool = True,
    tol_h: float = 50.0,
    tol_p: float = 100.0,
) -> List[ThermodynamicState]:
    """
    Build the ordered state list for plotting directly from a flowsheet and its FlowsheetState.

    The sequence includes segment transitions automatically so the red path passes through all segments.

    Args:
        flowsheet: The flowsheet object (e.g., StandardCycle) containing component states.
        fs_state: FlowsheetState holding segment data (segments_* variables).
        include_segment_inlets: If True, include segment inlet points as well (denser path).
        dedup: If True, drop consecutive near-identical points using tolerances.
        tol_h: J/kg tolerance for enthalpy when deduplicating.
        tol_p: Pa tolerance for pressure when deduplicating.

    Returns:
        Ordered list of ThermodynamicState for plotting.
    """
    segs = _collect_segments_from_fs(fs_state)

    states_for_plot: List[ThermodynamicState] = []

    # Start at evaporator inlet (expansion valve outlet)
    if hasattr(flowsheet, 'evaporator') and hasattr(flowsheet.evaporator, 'state_inlet'):
        states_for_plot.append(flowsheet.evaporator.state_inlet)

    # Evaporator phases: SC -> LAT -> SH
    for phase in ('sc', 'lat', 'sh'):
        key = f'segments_eva_{phase}'
        if key in segs and segs[key]:
            for seg in segs[key]:
                if include_segment_inlets and hasattr(seg, 'state_inlet') and seg.state_inlet:
                    states_for_plot.append(seg.state_inlet)
                if hasattr(seg, 'state_outlet') and seg.state_outlet:
                    states_for_plot.append(seg.state_outlet)

    # Compressor outlet (condenser inlet is identical; skip duplicate)
    if hasattr(flowsheet, 'compressor') and hasattr(flowsheet.compressor, 'state_outlet'):
        states_for_plot.append(flowsheet.compressor.state_outlet)

    # Condenser phases: SH -> LAT -> SC
    for phase in ('sh', 'lat', 'sc'):
        key = f'segments_con_{phase}'
        if key in segs and segs[key]:
            for seg in segs[key]:
                if include_segment_inlets and hasattr(seg, 'state_inlet') and seg.state_inlet:
                    states_for_plot.append(seg.state_inlet)
                if hasattr(seg, 'state_outlet') and seg.state_outlet:
                    states_for_plot.append(seg.state_outlet)

    # Expansion valve outlet (condenser outlet equals expansion valve inlet; avoid duplicate)
    if hasattr(flowsheet, 'expansion_valve') and hasattr(flowsheet.expansion_valve, 'state_outlet'):
        states_for_plot.append(flowsheet.expansion_valve.state_outlet)

    if dedup:
        states_for_plot = _dedup_consecutive_states(states_for_plot, tol_h=tol_h, tol_p=tol_p)

    return states_for_plot


def plot_cycle(
    med_prop: MedProp,
    states: List[ThermodynamicState],
    save_path: pathlib.Path = None,
    show: bool = False,
):
    """
    Creates T-h and p-h diagrams for a thermodynamic cycle.

    Args:
        med_prop (MedProp):
            Object containing the medium properties and two-phase limits.
        states (List[ThermodynamicState]):
            List of thermodynamic states defining the cycle points. Each state
            should contain T, p, and h properties.
        save_path (pathlib.Path, optional):
            Path where the plot should be saved. If None, returns the figure
            and axes objects instead. Defaults to None.
        show (bool):
            If True, plots are displayed. Default is False.

    Returns:
        tuple(matplotlib.figure.Figure, numpy.ndarray) or None:
            If save_path is provided, saves the plot and returns None.
            If show is True, shows the plot.
    """
    states.append(states[0])  # Plot full cycle
    # Unpack state var:
    h_T = np.array([state.h for state in states]) / 1000
    T = [state.T - 273.15 for state in states]
    p = np.array([state.p / 1e5 for state in states])
    h_p = h_T

    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].set_ylabel("$T$ in °C")
    ax[1].set_xlabel("$h$ in kJ/kgK")
    # Two phase limits
    ax[0].plot(
        med_prop.get_two_phase_limits("h") / 1000,
        med_prop.get_two_phase_limits("T") - 273.15, color="black"
    )

    ax[0].plot(h_T, T, color="r", marker="s", label="Main cycle")
    ax[1].set_yscale('log')  # Set y-axis to logarithmic scale
    ax[1].plot(h_p, p, marker="s", color="r", label="Main cycle")
    
    
    # Two phase limits
    ax[1].plot(
        med_prop.get_two_phase_limits("h") / 1000,
        med_prop.get_two_phase_limits("p") / 1e5,
        color="black"
    )
    ax[1].set_ylabel("$log(p)$ in bar")
    ax[1].set_ylim([np.min(p) * 0.9, np.max(p) * 1.1])
    ax[0].set_ylim([np.min(T) - 5, np.max(T) + 5])
    ax[1].set_xlim([np.min(h_T) * 0.9, np.max(h_T) * 1.1])
    ax[0].set_xlim([np.min(h_T) * 0.9, np.max(h_T) * 1.1])
    
    # Legend for main cycle
    ax[0].legend(loc="best")
    ax[1].legend(loc="best")
    
    if show:
        plt.show()
    if save_path is not None:
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)
        return
    return fig, ax


def plot_cycle_for_flowsheet(
    flowsheet,
    fs_state=None,
    save_path: Optional[pathlib.Path] = None,
    show: bool = False,
    include_segment_inlets: bool = False,
    show_segments_overlay: bool = False,
    dedup: bool = True,
    tol_h: float = 50.0,
    tol_p: float = 100.0,
):
    """
    Convenience: Build the plotting states (including segment transitions) automatically and plot.

    Args:
        flowsheet: Flowsheet object providing components and med_prop.
        fs_state: FlowsheetState containing segments_* variables.
        save_path: Optional path to save figure; if None, returns (fig, ax).
        show: If True, display the figure.
        include_segment_inlets: If True, include segment inlet points along the red path.
        show_segments_overlay: If True, draw segment inlet->outlet lines in orange as overlay.
        dedup: Drop consecutive near-identical states on the red path.
        tol_h: J/kg tolerance for enthalpy deduplication.
        tol_p: Pa tolerance for pressure deduplication.

    Returns:
        Same as plot_cycle: (fig, ax) if not saving, else None.
    """
    states = build_states_for_plot_from_flowsheet(
        flowsheet=flowsheet,
        fs_state=fs_state,
        include_segment_inlets=include_segment_inlets,
        dedup=dedup,
        tol_h=tol_h,
        tol_p=tol_p,
    )

    fig_ax = plot_cycle(med_prop=flowsheet.med_prop, states=states, save_path=None, show=False)
    if fig_ax is None:
        return None
    fig, ax = fig_ax

    if show_segments_overlay and fs_state is not None:
        segs = _collect_segments_from_fs(fs_state)

        # Helper to draw a segment line
        def _draw_segment(seg_obj):
            if not hasattr(seg_obj, 'state_inlet') or not hasattr(seg_obj, 'state_outlet'):
                return
            s_in = seg_obj.state_inlet
            s_out = seg_obj.state_outlet
            if not s_in or not s_out:
                return
            # T-h
            ax[0].plot(
                [s_in.h / 1000.0, s_out.h / 1000.0],
                [s_in.T - 273.15, s_out.T - 273.15],
                color='orange', alpha=0.5, linewidth=1.2
            )
            # log(p)-h
            ax[1].plot(
                [s_in.h / 1000.0, s_out.h / 1000.0],
                [s_in.p / 1e5, s_out.p / 1e5],
                color='orange', alpha=0.5, linewidth=1.2
            )

        # Order: evaporator SC -> LAT -> SH, then condenser SH -> LAT -> SC
        for phase in ('sc', 'lat', 'sh'):
            key = f'segments_eva_{phase}'
            for seg_list in ([segs[key]] if key in segs else []):
                for seg in seg_list:
                    _draw_segment(seg)
        for phase in ('sh', 'lat', 'sc'):
            key = f'segments_con_{phase}'
            for seg_list in ([segs[key]] if key in segs else []):
                for seg in seg_list:
                    _draw_segment(seg)

    if show:
        plt.show()
    if save_path is not None:
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)
        return
    return fig, ax


def _safe_lmtd_plot(dT1: float, dT2: float, eps: float = 1e-9) -> float:
    """Robuste LMTD-Berechnung für die Plot-Hilfsfunktionen."""
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


def show_in_hx(
    flowsheet,
    fs_state=None,
    inputs=None,
    save_path: Optional[pathlib.Path] = None,
    show: bool = True,
):
    """
    Visualisiere beide Wärmeübertrager (Verdampfer & Kondensator) übereinander über die Fläche.

    Für jeden HX werden 5 Diagramme vs. Fläche A gezeigt:
      - k_eff (aus Q_i, A_i, LMTD_i je Segment)
      - p absolut (aus Segment-States, stückweise linear)
      - Q_i je Segment (Stufen)
      - h des Kältemittels (stückweise linear über Segmente)
      - NTU je Segment (falls bestimmbar – sonst NaN)

    Hinweise:
      - Sekundärtemperaturen je Segment werden aus Q-Kumulierung und m_flow_secondary_cp rekonstruiert.
      - Für NTU wird m_flow_cp_min = min(m_ref*cp_pri, m_sec*cp_sec) verwendet; cp_pri wird in 1-Phasen-Segmenten als Δh/ΔT geschätzt, in LAT als ∞.

    Args:
        flowsheet: Flowsheet-Objekt mit `evaporator` und `condenser` Komponenten.
        fs_state: FlowsheetState mit `segments_*` Variablen.
        inputs: Inputs-Objekt (für T_con_in, T_eva_in). Optional – ohne Inputs sind k/NTU nur begrenzt sinnvoll.
        save_path: Optionaler Pfad zum Speichern.
        show: Ob die Figur angezeigt werden soll.
    """
    # Sammle Segmente
    segs = _collect_segments_from_fs(fs_state)

    def _linearize_hx(kind: str):
        # kind in {"con", "eva"}
        if kind == 'con':
            phase_order = ('sh', 'lat', 'sc')
            comp = getattr(flowsheet, 'condenser', None)
            T_sec_in = getattr(inputs, 'T_con_in', None) if inputs is not None else None
        else:
            phase_order = ('sc', 'lat', 'sh')
            comp = getattr(flowsheet, 'evaporator', None)
            T_sec_in = getattr(inputs, 'T_eva_in', None) if inputs is not None else None
        flow_type = getattr(comp, 'flow_type', 'counter') if comp is not None else 'counter'
        # Signum abhängig von Strömungsart (Entlang Primär-Flussrichtung):
        # Kondensator: counter → Sekundärtemperatur sinkt (−1), parallel → steigt (+1)
        # Verdampfer: counter → Sekundärtemperatur steigt (+1), parallel → sinkt (−1)
        if kind == 'con':
            sign_sec = -1.0 if str(flow_type).lower() == 'counter' else +1.0
        else:
            sign_sec = +1.0 if str(flow_type).lower() == 'counter' else -1.0

        if comp is None:
            return [], 0.0, None, None

        seg_list = []
        # Keep original computed order; continuity must be ensured by physics cascade,
        # not by reordering here (reversing breaks adjacency of segment endpoints).
        for ph in phase_order:
            key = f'segments_{kind}_{ph}'
            if key in segs and segs[key]:
                seg_list.extend(segs[key])

        A_total = float(sum(getattr(s, 'A', 0.0) or 0.0 for s in seg_list))
        m_ref = getattr(comp, 'm_flow', None) or 0.0
        m_sec_cp = getattr(comp, 'm_flow_secondary_cp', None)

        return seg_list, A_total, T_sec_in, (m_ref, m_sec_cp, sign_sec)

    seg_con, A_con, T_con_in, con_params = _linearize_hx('con')
    seg_eva, A_eva, T_eva_in, eva_params = _linearize_hx('eva')

    # Figure: 2 Zeilen (eva oben, con unten), 5 Spalten
    n_cols = 5
    fig, axes = plt.subplots(2, n_cols, figsize=(4*n_cols, 7), sharex=False)

    def _plot_one_hx(ax_row, segments, A_total, T_sec_in, params, title_prefix: str):
        k_ax, p_ax, q_ax, h_ax, ntu_ax = ax_row

        # Vorab: X-Bereiche je Segment (Flächenkoordinate)
        x0 = 0.0
        xs = []  # [(x0, x1)] je Segment
        for s in segments:
            A_i = float(getattr(s, 'A', 0.0) or 0.0)
            x1 = x0 + A_i
            xs.append((x0, x1))
            x0 = x1

        # Sekundär-T Verlauf rekonstruiert aus Q-Kumulierung
        m_ref, m_sec_cp, sign_sec = params if params is not None else (0.0, None, +1.0)
        T_s = None
        if T_sec_in is not None and m_sec_cp not in (None, 0):
            T_s = [float(T_sec_in)]
            for s in segments:
                Q_i = float(getattr(s, 'Q', 0.0) or 0.0)
                T_s.append(T_s[-1] + sign_sec * Q_i / float(m_sec_cp))

        # k_eff & NTU (Stufen), p & h (stückweise), Q (Stufen)
        cum_Q = 0.0
        prev_p_out = None
        for seg_idx, ((x0, x1), s) in enumerate(zip(xs, segments)):
            st_in = getattr(s, 'state_inlet', None)
            st_out = getattr(s, 'state_outlet', None)
            A_i = float(getattr(s, 'A', 0.0) or 0.0)
            Q_i = float(getattr(s, 'Q', 0.0) or 0.0)

            # Druck (Pa) & Enthalpie (J/kg): stückweise linear
            if st_in and st_out:
                # Enforce continuity at phase boundaries for visualization: if a segment starts
                # at a pressure different from the previous segment's outlet, draw from the
                # previous outlet to avoid visual jumps (data remains unchanged).
                p_in_plot = st_in.p
                if prev_p_out is not None and abs(p_in_plot - prev_p_out) > 50.0:  # 50 Pa tolerance
                    p_in_plot = prev_p_out
                p_ax.plot([x0, x1], [p_in_plot, st_out.p], color='tab:blue')
                prev_p_out = st_out.p
                h_ax.plot([x0, x1], [st_in.h/1000.0, st_out.h/1000.0], color='tab:green')

            # Q (W): Stufe
            q_ax.plot([x0, x1], [Q_i, Q_i], color='tab:red')

            # k_eff aus Q = k*A*LMTD (wenn T_s bekannt und st_in/out vorhanden)
            k_val = np.nan
            if T_s is not None and st_in and st_out and A_i > 0:
                # Segmentindex für T_s: len(T_s) == n_seg+1
                T_s_in = T_s[seg_idx]
                T_s_out = T_s[seg_idx+1]
                dT1 = (st_in.T - T_s_in)
                dT2 = (st_out.T - T_s_out)
                L = _safe_lmtd_plot(dT1, dT2)
                if L > 0:
                    k_val = Q_i / (A_i * L)
            k_ax.plot([x0, x1], [k_val, k_val], color='tab:orange')

            # NTU pro Segment: k*A_i / m_flow_cp_min, cp_pri ≈ Δh/ΔT für 1-Phasen, ∞ für LAT
            ntu_val = np.nan
            if not np.isnan(k_val) and m_sec_cp not in (None, 0):
                # cp_pri-Schätzung
                cp_pri = np.inf
                if st_in and st_out:
                    dT = (st_out.T - st_in.T)
                    dh = (st_out.h - st_in.h)
                    if abs(dT) > 1e-9:
                        cp_pri = max(abs(dh/dT), 1e-6)
                mcp_pri = (m_ref or 0.0) * (cp_pri if np.isfinite(cp_pri) else 1e99)
                mcp_min = min(mcp_pri, float(m_sec_cp)) if m_sec_cp is not None else None
                if mcp_min not in (None, 0):
                    ntu_val = k_val * A_i / mcp_min
            ntu_ax.plot([x0, x1], [ntu_val, ntu_val], color='tab:purple')

            cum_Q += Q_i

        # Styling
        for ax in (k_ax, p_ax, q_ax, h_ax, ntu_ax):
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0.0, max(1e-9, A_total))

        k_ax.set_title(f"{title_prefix} k_eff")
        k_ax.set_ylabel("k in W/m²K")

        p_ax.set_title(f"{title_prefix} p")
        p_ax.set_ylabel("p in Pa")

        q_ax.set_title(f"{title_prefix} Q Segment")
        q_ax.set_ylabel("Q in W")

        h_ax.set_title(f"{title_prefix} h (primär)")
        h_ax.set_ylabel("h in kJ/kg")

        ntu_ax.set_title(f"{title_prefix} NTU (Segment)")
        ntu_ax.set_ylabel("-")

        # Gemeinsame X-Label nur unten
        for ax in (k_ax, p_ax, q_ax, h_ax, ntu_ax):
            ax.set_xlabel("Fläche A in m²")

    # Obere Zeile: Verdampfer, Untere Zeile: Kondensator
    _plot_one_hx(axes[0, :], seg_eva, A_eva, T_eva_in, eva_params, "Verdampfer:")
    _plot_one_hx(axes[1, :], seg_con, A_con, T_con_in, con_params, "Kondensator:")

    fig.tight_layout()
    if show:
        plt.show()
    if save_path is not None:
        fig.savefig(save_path)
        plt.close(fig)
        return
    return fig, axes


def show_in_hx2(
    flowsheet,
    fs_state=None,
    inputs=None,
    save_path: Optional[pathlib.Path] = None,
    show: bool = True,
):
    """
    Visualisiere beide Wärmeübertrager (Verdampfer & Kondensator) übereinander über die Fläche.
    Diese Variante erzeugt smoothe, interpolierte Linien ohne Sprünge zwischen Segmenten.

    Für jeden HX werden 5 Diagramme vs. Fläche A gezeigt:
      - k_eff (interpoliert über Segmentmitten)
      - p absolut (smooth über alle Segment-States)
      - Q_i je Segment (interpoliert über Segmentmitten)
      - h des Kältemittels (smooth über alle Segment-States)
      - NTU je Segment (interpoliert über Segmentmitten)

    Hinweise:
      - Alle Linien werden interpoliert für glatte Darstellung
      - Konstante Werte (k, Q, NTU) werden über Segmentmitten verbunden
      - Zustandsgrößen (p, h) werden über Segment-Ein/Auslässe interpoliert

    Args:
        flowsheet: Flowsheet-Objekt mit `evaporator` und `condenser` Komponenten.
        fs_state: FlowsheetState mit `segments_*` Variablen.
        inputs: Inputs-Objekt (für T_con_in, T_eva_in). Optional.
        save_path: Optionaler Pfad zum Speichern.
        show: Ob die Figur angezeigt werden soll.
    """
    from scipy.interpolate import interp1d
    
    # Sammle Segmente
    segs = _collect_segments_from_fs(fs_state)

    def _linearize_hx(kind: str):
        if kind == 'con':
            phase_order = ('sh', 'lat', 'sc')
            comp = getattr(flowsheet, 'condenser', None)
            T_sec_in = getattr(inputs, 'T_con_in', None) if inputs is not None else None
        else:
            phase_order = ('sc', 'lat', 'sh')
            comp = getattr(flowsheet, 'evaporator', None)
            T_sec_in = getattr(inputs, 'T_eva_in', None) if inputs is not None else None
        flow_type = getattr(comp, 'flow_type', 'counter') if comp is not None else 'counter'
        # Richtung der Sekundärtemperatur entlang Primär-Flussrichtung
        if kind == 'con':
            sign_sec = -1.0 if str(flow_type).lower() == 'counter' else +1.0
        else:
            sign_sec = +1.0 if str(flow_type).lower() == 'counter' else -1.0

        if comp is None:
            return [], 0.0, None, None

        seg_list = []
        for ph in phase_order:
            key = f'segments_{kind}_{ph}'
            if key in segs and segs[key]:
                seg_list.extend(segs[key])

        A_total = float(sum(getattr(s, 'A', 0.0) or 0.0 for s in seg_list))
        m_ref = getattr(comp, 'm_flow', None) or 0.0
        m_sec_cp = getattr(comp, 'm_flow_secondary_cp', None)

        return seg_list, A_total, T_sec_in, (m_ref, m_sec_cp, sign_sec)

    seg_con, A_con, T_con_in, con_params = _linearize_hx('con')
    seg_eva, A_eva, T_eva_in, eva_params = _linearize_hx('eva')

    # Figure: 2 Zeilen (eva oben, con unten), 5 Spalten
    n_cols = 5
    fig, axes = plt.subplots(2, n_cols, figsize=(4*n_cols, 7), sharex=False)

    def _plot_one_hx(ax_row, segments, A_total, T_sec_in, params, title_prefix: str):
        k_ax, p_ax, q_ax, h_ax, ntu_ax = ax_row

        if not segments:
            return

        # Vorab: X-Bereiche je Segment (Flächenkoordinate)
        x0 = 0.0
        xs = []  # [(x0, x1)] je Segment
        x_mids = []  # Segmentmitten
        for s in segments:
            A_i = float(getattr(s, 'A', 0.0) or 0.0)
            x1 = x0 + A_i
            xs.append((x0, x1))
            x_mids.append(0.5 * (x0 + x1))
            x0 = x1

        # Sekundär-T Verlauf rekonstruiert aus Q-Kumulierung
        m_ref, m_sec_cp, sign_sec = params if params is not None else (0.0, None, +1.0)
        T_s = None
        if T_sec_in is not None and m_sec_cp not in (None, 0):
            T_s = [float(T_sec_in)]
            for s in segments:
                Q_i = float(getattr(s, 'Q', 0.0) or 0.0)
                T_s.append(T_s[-1] + sign_sec * Q_i / float(m_sec_cp))

        # Sammle Daten für smooth interpolation
        # Für k, Q, NTU: Werte an Segmentmitten
        k_vals = []
        q_vals = []
        ntu_vals = []
        
        # Für p, h: Werte an Segment-Grenzen (inlet/outlet)
        x_states = [0.0]  # Startposition
        p_vals = []
        h_vals = []
        
        cum_Q = 0.0
        for seg_idx, ((x0, x1), s) in enumerate(zip(xs, segments)):
            st_in = getattr(s, 'state_inlet', None)
            st_out = getattr(s, 'state_outlet', None)
            A_i = float(getattr(s, 'A', 0.0) or 0.0)
            Q_i = float(getattr(s, 'Q', 0.0) or 0.0)

            # States für p und h sammeln
            if st_in and seg_idx == 0:
                p_vals.append(st_in.p)
                h_vals.append(st_in.h / 1000.0)
            
            if st_out:
                x_states.append(x1)
                p_vals.append(st_out.p)
                h_vals.append(st_out.h / 1000.0)

            # Q
            q_vals.append(Q_i)

            # k_eff aus Q = k*A*LMTD
            k_val = np.nan
            if T_s is not None and st_in and st_out and A_i > 0:
                T_s_in = T_s[seg_idx]
                T_s_out = T_s[seg_idx+1]
                dT1 = (st_in.T - T_s_in)
                dT2 = (st_out.T - T_s_out)
                L = _safe_lmtd_plot(dT1, dT2)
                if L > 0:
                    k_val = Q_i / (A_i * L)
            k_vals.append(k_val)

            # NTU pro Segment
            ntu_val = np.nan
            if not np.isnan(k_val) and m_sec_cp not in (None, 0):
                cp_pri = np.inf
                if st_in and st_out:
                    dT = (st_out.T - st_in.T)
                    dh = (st_out.h - st_in.h)
                    if abs(dT) > 1e-9:
                        cp_pri = max(abs(dh/dT), 1e-6)
                mcp_pri = (m_ref or 0.0) * (cp_pri if np.isfinite(cp_pri) else 1e99)
                mcp_min = min(mcp_pri, float(m_sec_cp)) if m_sec_cp is not None else None
                if mcp_min not in (None, 0):
                    ntu_val = k_val * A_i / mcp_min
            ntu_vals.append(ntu_val)

            cum_Q += Q_i

        # Smooth interpolation für konstante Werte (k, Q, NTU) über Segmentmitten
        if len(x_mids) > 1:
            x_smooth = np.linspace(0, A_total, 200)
            
            # k_eff: nur finite Werte interpolieren
            k_finite_idx = [i for i, v in enumerate(k_vals) if np.isfinite(v)]
            if len(k_finite_idx) > 1:
                k_interp = interp1d([x_mids[i] for i in k_finite_idx], 
                                   [k_vals[i] for i in k_finite_idx],
                                   kind='linear', bounds_error=False, fill_value='extrapolate')
                k_ax.plot(x_smooth, k_interp(x_smooth), color='tab:orange', linewidth=2)
            elif len(k_finite_idx) == 1:
                k_ax.axhline(k_vals[k_finite_idx[0]], color='tab:orange', linewidth=2)
            
            # Q
            q_finite_idx = [i for i, v in enumerate(q_vals) if np.isfinite(v)]
            if len(q_finite_idx) > 1:
                q_interp = interp1d([x_mids[i] for i in q_finite_idx], 
                                   [q_vals[i] for i in q_finite_idx],
                                   kind='linear', bounds_error=False, fill_value='extrapolate')
                q_ax.plot(x_smooth, q_interp(x_smooth), color='tab:red', linewidth=2)
            elif len(q_finite_idx) == 1:
                q_ax.axhline(q_vals[q_finite_idx[0]], color='tab:red', linewidth=2)
            
            # NTU
            ntu_finite_idx = [i for i, v in enumerate(ntu_vals) if np.isfinite(v)]
            if len(ntu_finite_idx) > 1:
                ntu_interp = interp1d([x_mids[i] for i in ntu_finite_idx], 
                                     [ntu_vals[i] for i in ntu_finite_idx],
                                     kind='linear', bounds_error=False, fill_value='extrapolate')
                ntu_ax.plot(x_smooth, ntu_interp(x_smooth), color='tab:purple', linewidth=2)
            elif len(ntu_finite_idx) == 1:
                ntu_ax.axhline(ntu_vals[ntu_finite_idx[0]], color='tab:purple', linewidth=2)
        
        # Smooth interpolation für Zustandsgrößen (p, h) über Segment-Grenzen
        if len(x_states) > 1 and len(p_vals) == len(x_states):
            x_smooth = np.linspace(0, A_total, 200)
            
            # Druck: monotone interpolation to prevent oscillations
            p_finite_idx = [i for i, v in enumerate(p_vals) if v is not None and np.isfinite(v)]
            if len(p_finite_idx) > 1:
                from scipy.interpolate import PchipInterpolator
                # PCHIP (Piecewise Cubic Hermite Interpolating Polynomial) preserves monotonicity
                p_interp = PchipInterpolator([x_states[i] for i in p_finite_idx], 
                                             [p_vals[i] for i in p_finite_idx])
                p_ax.plot(x_smooth, p_interp(x_smooth), color='tab:blue', linewidth=2)
            
            # Enthalpie: PCHIP for smooth monotone curve
            h_finite_idx = [i for i, v in enumerate(h_vals) if v is not None and np.isfinite(v)]
            if len(h_finite_idx) > 1:
                from scipy.interpolate import PchipInterpolator
                h_interp = PchipInterpolator([x_states[i] for i in h_finite_idx], 
                                             [h_vals[i] for i in h_finite_idx])
                h_ax.plot(x_smooth, h_interp(x_smooth), color='tab:green', linewidth=2)

        # Styling
        for ax in (k_ax, p_ax, q_ax, h_ax, ntu_ax):
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0.0, max(1e-9, A_total))

        k_ax.set_title(f"{title_prefix} k_eff (smooth)")
        k_ax.set_ylabel("k in W/m²K")

        p_ax.set_title(f"{title_prefix} p (smooth)")
        p_ax.set_ylabel("p in Pa")

        q_ax.set_title(f"{title_prefix} Q Segment (smooth)")
        q_ax.set_ylabel("Q in W")

        h_ax.set_title(f"{title_prefix} h (primär, smooth)")
        h_ax.set_ylabel("h in kJ/kg")

        ntu_ax.set_title(f"{title_prefix} NTU (Segment, smooth)")
        ntu_ax.set_ylabel("-")

        # Gemeinsame X-Label nur unten
        for ax in (k_ax, p_ax, q_ax, h_ax, ntu_ax):
            ax.set_xlabel("Fläche A in m²")

    # Obere Zeile: Verdampfer, Untere Zeile: Kondensator
    _plot_one_hx(axes[0, :], seg_eva, A_eva, T_eva_in, eva_params, "Verdampfer:")
    _plot_one_hx(axes[1, :], seg_con, A_con, T_con_in, con_params, "Kondensator:")

    fig.tight_layout()
    if show:
        plt.show()
    if save_path is not None:
        fig.savefig(save_path)
        plt.close(fig)
        return
    return fig, axes


def show_hx_temperature_profiles(
    flowsheet,
    fs_state=None,
    inputs=None,
    save_path: Optional[pathlib.Path] = None,
    show: bool = True,
    show_flow_arrows: bool = True,
):
    """
    Visualisiere Temperaturverläufe von Kältemittel und Sekundärmedium über die Fläche.
    
    Zeigt zwei Diagramme:
      - Verdampfer: T_refrigerant und T_secondary vs. Fläche
      - Kondensator: T_refrigerant und T_secondary vs. Fläche
    
    Args:
        flowsheet: Flowsheet-Objekt mit `evaporator` und `condenser` Komponenten.
        fs_state: FlowsheetState mit `segments_*` Variablen.
        inputs: Inputs-Objekt (für T_con_in, T_eva_in).
        save_path: Optionaler Pfad zum Speichern.
        show: Ob die Figur angezeigt werden soll.
    """
    from scipy.interpolate import PchipInterpolator
    
    # Sammle Segmente
    segs = _collect_segments_from_fs(fs_state)

    def _linearize_hx(kind: str):
        if kind == 'con':
            phase_order = ('sh', 'lat', 'sc')
            comp = getattr(flowsheet, 'condenser', None)
            T_sec_in = getattr(inputs, 'T_con_in', None) if inputs is not None else None
            sign_sec = +1.0
        else:
            phase_order = ('sc', 'lat', 'sh')
            comp = getattr(flowsheet, 'evaporator', None)
            T_sec_in = getattr(inputs, 'T_eva_in', None) if inputs is not None else None
            sign_sec = -1.0

        if comp is None:
            return [], 0.0, None, None

        seg_list = []
        for ph in phase_order:
            key = f'segments_{kind}_{ph}'
            if key in segs and segs[key]:
                seg_list.extend(segs[key])

        A_total = float(sum(getattr(s, 'A', 0.0) or 0.0 for s in seg_list))
        m_sec_cp = getattr(comp, 'm_flow_secondary_cp', None)
        flow_type = getattr(comp, 'flow_type', 'counter')

        return seg_list, A_total, T_sec_in, (m_sec_cp, flow_type)

    seg_con, A_con, T_con_in, con_params = _linearize_hx('con')
    seg_eva, A_eva, T_eva_in, eva_params = _linearize_hx('eva')

    # Figure: 1 Zeile, 2 Spalten (Verdampfer links, Kondensator rechts)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    def _plot_one_hx(ax, segments, A_total, T_sec_in, params, title: str):
        if not segments:
            return

        m_sec_cp, flow_type = params if params is not None else (None, 'counter')

        # X-Positionen und Temperaturdaten sammeln
        x_states = [0.0]
        T_ref = []  # Kältemittel-Temperaturen
        T_sec = []  # Sekundärmedium-Temperaturen

        x0 = 0.0

        # Gruppiere Segmente nach Phase (in vorhandener Reihenfolge)
        phase_groups: Dict[str, List] = {'sc': [], 'lat': [], 'sh': []}
        for s in segments:
            ph = getattr(s, 'phase_type', None)
            if ph in phase_groups:
                phase_groups[ph].append(s)

        # Phasenreihenfolge je HX festlegen
        is_condenser = 'Kondensator' in title or 'con' in title.lower()
        if is_condenser:
            ordered_phases = ['sh', 'lat', 'sc']
        else:
            ordered_phases = ['sc', 'lat', 'sh']

        # Q je Phase (W)
        Q_phase: Dict[str, float] = {
            ph: float(sum((getattr(s, 'Q', 0.0) or 0.0) for s in phase_groups.get(ph, [])))
            for ph in ['sc', 'lat', 'sh']
        }

        # Berechne Sekundär-Temperaturgrenzen je Phase abhängig vom flow_type
        T_bounds_per_phase: Dict[str, tuple] = {}
        if m_sec_cp not in (None, 0) and T_sec_in is not None:
            mcp = float(m_sec_cp)
            if is_condenser:
                # Sekundär heizt sich auf; Gesamtbilanz
                T_sc = T_sec_in + Q_phase['sc'] / mcp
                T_sh = T_sc + Q_phase['lat'] / mcp
                T_out = T_sh + Q_phase['sh'] / mcp
                if str(flow_type).lower() == 'parallel':
                    # Entlang Primärfluss: steigt von T_in → T_out
                    T_bounds_per_phase['sh'] = (T_sec_in, T_sec_in + Q_phase['sh'] / mcp)
                    T_bounds_per_phase['lat'] = (T_bounds_per_phase['sh'][1], T_bounds_per_phase['sh'][1] + Q_phase['lat'] / mcp)
                    T_bounds_per_phase['sc'] = (T_bounds_per_phase['lat'][1], T_out)
                else:
                    # Gegenstrom: entlang Primärfluss sinkt von T_out → T_in
                    T_bounds_per_phase['sh'] = (T_out, T_sh)
                    T_bounds_per_phase['lat'] = (T_sh, T_sc)
                    T_bounds_per_phase['sc'] = (T_sc, T_sec_in)
            else:
                # Verdampfer: Sekundär kühlt sich ab
                T_sh = T_sec_in - Q_phase['sh'] / mcp
                T_sc = T_sh - Q_phase['lat'] / mcp
                T_out = T_sc - Q_phase['sc'] / mcp
                if str(flow_type).lower() == 'parallel':
                    # Entlang Primärfluss: sinkt von T_in → T_out
                    T_bounds_per_phase['sc'] = (T_sec_in, T_sec_in - Q_phase['sc'] / mcp)
                    T_bounds_per_phase['lat'] = (T_bounds_per_phase['sc'][1], T_bounds_per_phase['sc'][1] - Q_phase['lat'] / mcp)
                    T_bounds_per_phase['sh'] = (T_bounds_per_phase['lat'][1], T_out)
                else:
                    # Gegenstrom: entlang Primärfluss steigt von T_out → T_in
                    T_bounds_per_phase['sc'] = (T_out, T_sc)
                    T_bounds_per_phase['lat'] = (T_sc, T_sh)
                    T_bounds_per_phase['sh'] = (T_sh, T_sec_in)

        # Sammle Kältemittel- und Sekundär-Temperaturen an Segment-Grenzen
        idx_in_phase: Dict[str, int] = {'sc': 0, 'lat': 0, 'sh': 0}
        bounds_cache: Dict[str, np.ndarray] = {}
        for ph in ['sc', 'lat', 'sh']:
            if ph in phase_groups and phase_groups[ph] and ph in T_bounds_per_phase:
                n = len(phase_groups[ph])
                T_in_ph, T_out_ph = T_bounds_per_phase[ph]
                bounds_cache[ph] = np.linspace(T_in_ph, T_out_ph, n + 1)
            else:
                bounds_cache[ph] = np.array([])

        for seg_idx, s in enumerate(segments):
            A_i = float(getattr(s, 'A', 0.0) or 0.0)
            st_in = getattr(s, 'state_inlet', None)
            st_out = getattr(s, 'state_outlet', None)
            ph = getattr(s, 'phase_type', None)

            # Erster Eintrag: Inlet-Temperatur des ersten Segments
            if seg_idx == 0 and st_in:
                T_ref.append(st_in.T - 273.15)  # Konvertiere zu °C
                if ph in bounds_cache and bounds_cache[ph].size > 0:
                    # inlet des ersten segments in dieser phase
                    T_sec.append(bounds_cache[ph][0] - 273.15)

            # Outlet-Temperatur am Ende des Segments
            if st_out:
                x1 = x0 + A_i
                x_states.append(x1)
                T_ref.append(st_out.T - 273.15)
                if ph in bounds_cache and bounds_cache[ph].size > 0:
                    i_in_phase = idx_in_phase.get(ph, 0)
                    # outlet-Grenze für dieses Segment
                    T_sec.append(bounds_cache[ph][i_in_phase + 1] - 273.15)
                    idx_in_phase[ph] = i_in_phase + 1

                x0 = x1

        # Smooth interpolation mit PCHIP (monoton)
        if len(x_states) > 1 and len(T_ref) == len(x_states):
            x_smooth = np.linspace(0, A_total, 200)

            # Kältemittel-Temperatur (immer Vorwärtsrichtung entlang Fläche)
            T_ref_finite_idx = [i for i, v in enumerate(T_ref) if v is not None and np.isfinite(v)]
            T_ref_curve = None
            if len(T_ref_finite_idx) > 1:
                T_ref_interp = PchipInterpolator([x_states[i] for i in T_ref_finite_idx],
                                                 [T_ref[i] for i in T_ref_finite_idx])
                T_ref_curve = T_ref_interp(x_smooth)
                ax.plot(x_smooth, T_ref_curve, color='tab:red', linewidth=2, label='Kältemittel')

            # Sekundärmedium-Temperatur (Richtung abhängig von flow_type)
            T_sec_curve = None
            if len(T_sec) == len(x_states):
                T_sec_finite_idx = [i for i, v in enumerate(T_sec) if v is not None and np.isfinite(v)]
                if len(T_sec_finite_idx) > 1:
                    T_sec_interp = PchipInterpolator([x_states[i] for i in T_sec_finite_idx],
                                                     [T_sec[i] for i in T_sec_finite_idx])
                    T_sec_curve = T_sec_interp(x_smooth)
                    ax.plot(x_smooth, T_sec_curve, color='tab:blue', linewidth=2, label='Sekundärmedium')

            # Flussrichtungspfeile einzeichnen (Marker statt Text)
            if show_flow_arrows and x_smooth.size > 0:
                # Positionen für Pfeile (in Prozent der Gesamtfläche)
                frac_positions = [0.15, 0.45, 0.75]
                arrow_x = [f * A_total for f in frac_positions if 0.0 <= f <= 1.0]
                # Primärfluss: immer vorwärts (→) entlang steigender Fläche
                if T_ref_curve is not None:
                    # Wähle y-Werte durch Interpolation der Kurve
                    y_ref_sel = np.interp(arrow_x, x_smooth, T_ref_curve)
                    ax.plot(arrow_x, y_ref_sel, linestyle='None', marker='>', color='tab:red', markersize=8, label=None)
                # Sekundärfluss: Richtung abhängig vom flow_type
                if T_sec_curve is not None:
                    sec_direction = 'forward'
                    if str(flow_type).lower() == 'counter':
                        sec_direction = 'backward'
                    y_sec_sel = np.interp(arrow_x, x_smooth, T_sec_curve)
                    marker_style = '>' if sec_direction == 'forward' else '<'
                    ax.plot(arrow_x, y_sec_sel, linestyle='None', marker=marker_style, color='tab:blue', markersize=8, label=None)

        # Styling
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0.0, max(1e-9, A_total))
        ax.set_xlabel("Fläche A in m²")
        ax.set_ylabel("Temperatur in °C")
        ax.set_title(title)
        ax.legend(loc='best')

    # Linkes Diagramm: Verdampfer
    _plot_one_hx(axes[0], seg_eva, A_eva, T_eva_in, eva_params, "Verdampfer: Temperaturverläufe")

    # Rechtes Diagramm: Kondensator
    _plot_one_hx(axes[1], seg_con, A_con, T_con_in, con_params, "Kondensator: Temperaturverläufe")

    fig.tight_layout()
    if show:
        plt.show()
    if save_path is not None:
        fig.savefig(save_path)
        plt.close(fig)
        return
    return fig, axes


def plot_sdf_map(
        filepath_sdf: pathlib.Path,
        nd_data: str,
        first_dimension: str,
        second_dimension: str,
        fluids: List[str] = None,
        flowsheets: List[str] = None,
        violin_plot_variable: str = None,
        third_dimension: str = None
):
    """
    Generate and display visualizations based on data from an SDF (Structured Data File) dataset.
    This function generates various types of visualizations based on the provided parameters,
    including 3D scatter plots, 3D surface plots, and violin plots, and displays them using Matplotlib.

    Args:
        filepath_sdf (pathlib.Path):
            The path to the SDF dataset file.
        nd_data (str):
            The name of the primary data to be plotted.
        first_dimension (str):
            The name of the first dimension for the visualization.
        second_dimension (str):
            The name of the second dimension for the visualization.
        fluids (List[str], optional):
            List of specific fluids to include in the visualization.
            Default is None, which includes all fluids.
        flowsheets (List[str], optional):
            List of specific flowsheets to include in the visualization.
            Default is None, which includes all flowsheets.
        violin_plot_variable (str, optional):
            The variable to be used for creating violin plots.
            Default is None, which disables violin plots.
        third_dimension (str, optional):
            The name of the third dimension for 4D visualizations.
            Default is None, which disables 4D plotting.

    Raises:
        KeyError: If the specified data or dimensions are not found in the dataset.

    Examples:
    >>> FILEPATH_SDF = r"HeatPumpMaps.sdf"
    >>> plot_sdf_map(
    >>>     filepath_sdf=FILEPATH_SDF,
    >>>     nd_data="COP",
    >>>     first_dimension="T_eva_in",
    >>>     second_dimension="n",
    >>>     fluids=["R410A"],
    >>>     flowsheets=["OptiHorn"],
    >>> )

    """
    if fluids is None:
        fluids = []
    if flowsheets is None:
        flowsheets = []
    if "T_" in second_dimension:
        offset_sec = -273.15
    else:
        offset_sec = 0
    if "T_" in first_dimension:
        offset_pri = -273.15
    else:
        offset_pri = 0
    offset_thi = 0
    plot_4D = False
    if third_dimension is not None:
        plot_4D = True
        if "T_" in third_dimension:
            offset_thi = -273.15

    dataset = sdf.load(str(filepath_sdf))
    plot_violin = True
    if violin_plot_variable is None:
        plot_violin = False
        violin_plot_variable = ""
    if plot_violin:
        if flowsheets:
            n_rows = len(flowsheets)
        else:
            n_rows = len(dataset.groups)
        fig_v, ax_v = plt.subplots(nrows=n_rows, ncols=1, sharex=True,
                                   squeeze=False)
        fig_v.suptitle(violin_plot_variable)
    i_fs = 0
    nd_str_plot = nd_data
    fac = 1
    if nd_data == "dT_eva_min":
        nd_data = "T_1"
        sub_str = "T_eva_in"
        fac = - 1
    elif nd_data == "dT_con":
        nd_data = "T_3"
        sub_str = "T_con_in"
    elif nd_data == "dT_sh":
        nd_data = "T_1"
        sub_str = "T_4"
    else:
        sub_str = ""

    if nd_str_plot.startswith("T_"):
        offset_nd = -273.15
    else:
        offset_nd = 0

    for flowsheet in dataset.groups:
        violin_data = {}
        if flowsheet.name not in flowsheets and len(flowsheets) > 0:
            continue
        for fluid in flowsheet.groups:
            if fluid.name not in fluids and len(fluids) > 0:
                continue
            nd, fd, sd, sub_data, td = None, None, None, None, None
            _other_scale = {}
            for ds in fluid.datasets:
                if ds.name == nd_data:
                    nd = ds
                elif ds.name == first_dimension:
                    fd = ds.data
                elif ds.name == second_dimension:
                    sd = ds.data
                if ds.name == sub_str:
                    sub_data = ds.data
                if ds.name == violin_plot_variable:
                    data = ds.data.flatten()
                    violin_data[fluid.name] = data[~np.isnan(data)]
                if plot_4D and ds.name == third_dimension:
                    td = ds.data

            if nd is None:
                raise KeyError("nd-String not found in dataset")

            if sub_data is None:
                sub_data = np.zeros(nd.data.shape)

            # Check other scales:
            for i, scale in enumerate(nd.scales):
                if scale.name not in [first_dimension, second_dimension]:
                    _other_scale[i] = scale

            if fd is None or sd is None or not _other_scale or (plot_4D and td is None):
                raise KeyError("One of the given strings was not found in dataset")

            if plot_4D:
                fig = plt.figure()
                figtitle = f"{flowsheet.name}_{fluid.name}_{nd_str_plot}"
                fig.suptitle(figtitle)
                ax = fig.add_subplot(111, projection='3d')
                ax.set_xlabel(first_dimension)
                ax.set_ylabel(second_dimension)
                ax.set_zlabel(third_dimension)
                fourth_dim = (nd.data - sub_data) * fac + offset_nd
                # Scale values for better sizes of circles:
                bounds = [fourth_dim.min(), fourth_dim.max()]
                _max_circle_size = 30
                fourth_dim_scaled = (fourth_dim - bounds[0]) / (bounds[1] - bounds[0]) * _max_circle_size
                inp = [fd + offset_pri, sd + offset_sec, td + offset_thi]
                import itertools
                scattergrid = np.array([c for c in itertools.product(*inp)])
                ax.scatter(scattergrid[:, 0],
                           scattergrid[:, 1],
                           scattergrid[:, 2],
                           c=fourth_dim_scaled,
                           s=fourth_dim_scaled)
            else:
                for index, scale in _other_scale.items():
                    for idx_data, value in enumerate(scale.data):
                        if index==0:
                            Z = nd.data[idx_data, :, :]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[idx_data, :, :]
                            else:
                                sub_data_use = sub_data
                        elif index == 1:
                            Z = nd.data[:, idx_data, :]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[:, idx_data, :]
                            else:
                                sub_data_use = sub_data
                        else:
                            Z = nd.data[:, :, idx_data]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[:, :, idx_data]
                            else:
                                sub_data_use = sub_data
                        if not plot_violin:
                            fig = plt.figure()
                            figtitle = f"{flowsheet.name}_{fluid.name}_{nd_str_plot}_{scale.name}={round(value, 3)}"
                            fig.suptitle(figtitle)
                            ax = fig.add_subplot(111, projection='3d')
                            ax.set_xlabel(first_dimension)
                            ax.set_ylabel(second_dimension)
                            X, Y = np.meshgrid(fd, sd)
                            ax.plot_surface(X + offset_pri, Y + offset_sec, (Z - sub_data_use)*fac + offset_nd)

        if plot_violin:
            for key, value in violin_data.items():
                print(f"{violin_plot_variable}: {flowsheet.name}_{key}")
                print(f"Min: {np.min(value)}")
                print(f"Max: {np.max(value)}")
                print(f"Mean: {np.mean(value)}")
                print(f"Median: {np.median(value)}\n")
            ax_v[i_fs][0].violinplot(
                                     list(violin_data.values()),
                                     showextrema=True,
                                     showmeans=True,
                                     showmedians=True
                                     )
            set_axis_style(ax_v[i_fs][0], list(violin_data.keys()))
            ax_v[i_fs][0].set_ylabel(flowsheet.name.replace("Flowsheet", ""))
        i_fs += 1
    plt.show()


def set_axis_style(ax, labels):
    """
    From: https://matplotlib.org/3.1.1/gallery/statistics/customized_violin.html#sphx-glr-gallery-statistics-customized-violin-py
    """
    ax.get_xaxis().set_tick_params(direction='out')
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xticks(np.arange(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_xlim(0.25, len(labels) + 0.75)
