# Heat exchanger modes and legacy entry points

This repository currently relies on the moving-boundary NTU implementation for
all vapor-compression flowsheets. Two concrete heat exchanger classes drive the
calculations:

- `MovingBoundaryNTUEvaporator` in
  `vclibpy/components/heat_exchangers/moving_boundary_ntu.py`, which separates
  subcooling/latent/superheat regions, determines heat transfer via existing NTU
  helpers, and returns the iteration signals `error` and `dT_min` to the cycle
  solver.【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L12-L120】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L122-L224】
- `MovingBoundaryNTUCondenser` in the same module, following the identical
  contract and using the condenser-specific branch ordering while exposing the
  same `calc` signature.【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L33-L120】

Cycle classes delegate heat-exchanger iterations to these `calc` methods during
steady-state solving. The `BaseCycle.calc_steady_state` loop calls
`evaporator.calc` and `condenser.calc` after the refrigerant states are
assembled, and it uses the returned `error` and `dT_min` values to adjust the
pressures until convergence.【F:vclibpy/flowsheets/base.py†L97-L198】

## Pressure-level convention

`StandardCycleWithDP` keeps the steady-state solver unchanged: `p1` and `p2`
remain evaporation and condensation **level pressures**. Port pressures are
derived from accumulated pressure drops, which are stored in the flowsheet
state by the condenser and evaporator as `dp_con_total` and `dp_eva_total`.
When no pressure-drop model is present, the port and level pressures are
identical, preserving regression behaviour.【F:vclibpy/flowsheets/standardWITHdp.py†L1-L86】

## Mode selection placeholder

A dedicated configuration object, `SegmentationConfig`, is available for callers
to signal when a segmented path should be used in the future. Providing this
configuration attaches the intent to the heat exchanger instance while the
legacy calculations remain active: the segmented counts default to `(N_sc,
N_lat, N_sh) = (1, 1, 1)` and the existing moving-boundary logic runs
unchanged.【F:vclibpy/components/heat_exchangers/segmentation.py†L12-L61】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L122-L224】

### Segmentation parameters

The configuration exposes iteration controls (`tol`, `max_iter`, `relaxation`)
and per-zone segment counts (`N_sc`, `N_lat`, `N_sh`). When any segmentation is
requested but counts are omitted, the defaults resolve to `(1, 1, 1)` so legacy
results are reproduced while still emitting per-segment profiles in the
flowsheet state.【F:vclibpy/components/heat_exchangers/segmentation.py†L12-L61】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L320-L520】

### Pressure-drop model hook

Heat exchangers accept an optional `dp_model` that scales pressure drop for each
subsegment proportionally to its share of the surface area. With `dp_model=None`
the drops are zero; when provided, total pressure drops are accumulated and
stored under `dp_con_total`/`dp_eva_total`, and per-segment inlet/outlet
pressures are recorded for diagnostics.【F:vclibpy/components/heat_exchangers/heat_exchanger.py†L30-L77】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L320-L520】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L600-L758】

### Segment-wise pinch tracking

Each subsegment computes its local minimum temperature difference and the global
minimum pinch value is retained alongside the zone and segment index where it
occurs. This information is available through the segment profiles stored in the
flowsheet state so violations can be traced back to a specific region of the
heat exchanger.【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L320-L520】【F:vclibpy/components/heat_exchangers/moving_boundary_ntu.py†L600-L758】

## Release checklist

- [x] `base.py` unchanged; steady-state solver still iterates on two level
  pressures.
- [x] `StandardCycleWithDP` derives port pressures from segment pressure drops
  while keeping `p1`/`p2` as level pressures.
- [x] Segmented heat-exchanger mode performs moving-boundary zoning with
  subsegments and iterative NTU solves, persisting p/h profiles.
- [x] Segment-wise pinch detection and pressure-drop hook with `dp_model=None`
  defaulting to zero.
- [x] Segmentation defaults `(1,1,1)` preserve legacy regression; legacy mode
  remains available.
- [x] Example e14 exercises the segmentation-aware API and runs unchanged.
- [x] Tests cover regression, segmentation, pressure-drop profiles, integration,
  and the e14 example.

