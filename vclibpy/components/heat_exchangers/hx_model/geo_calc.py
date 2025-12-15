"""Geometry calculation helpers for HX models.

This module reads simple TXT-based geometry definition files (see
``fin_tube_demo.txt`` and ``plate_demo.txt``) and computes the most
important geometric quantities needed for pressure-drop and heat-transfer
correlations.

Current implementation uses placeholder formulas for demonstration. The
actual correlations will be filled in later.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any


BASE_DIR = Path(__file__).resolve().parent


def _parse_geometry_txt(filename: str | Path) -> Dict[str, Any]:
	"""Parse a simple key=value TXT geometry file.

	The parser is intentionally very lightweight:
	- Ignores empty lines and lines starting with '#'
	- Expects ``key=value,`` per line (trailing comma optional)
	- Converts values with ``float()`` when possible, otherwise keeps strings
	"""

	path = Path(filename)
	if not path.is_absolute():
		path = BASE_DIR / path

	if not path.exists():
		raise FileNotFoundError(f"Geometry file not found: {path}")

	data: Dict[str, Any] = {}
	with path.open("r", encoding="utf-8") as f:
		for raw_line in f:
			line = raw_line.strip()
			if not line or line.startswith("#"):
				continue

			# Remove optional trailing comma
			if line.endswith(","):
				line = line[:-1]

			if "=" not in line:
				continue

			key, value_str = line.split("=", 1)
			key = key.strip()
			value_str = value_str.strip()

			# Try to convert to float, otherwise keep as string
			try:
				value: Any = float(value_str)
			except ValueError:
				value = value_str

			data[key] = value

	return data


@dataclass
class GeometryResult:
	"""Container for key geometric quantities.

	All formulas are placeholders for now and will be updated later.
	"""

	type: str
	outer_area_m2: float
	inner_area_m2: float
	length_m: float
	hydraulic_diameter_m: float
	wall_thickness_m: float | None
	raw_data: Dict[str, Any]


def _compute_geometry_placeholders(params: Dict[str, Any]) -> GeometryResult:
	"""Compute geometry using placeholder formulas.

	The logic branches on ``type`` (e.g. "fin_tube", "plate") but only uses
	very simple surrogate expressions for now. This is intentional so that the
	structure of the code is ready while the exact equations are added later.
	"""

	geom_type = str(params.get("type", "unknown"))

	# --- Generic length: depends on geometry type ---
	if geom_type == "plate":
		length = float(params.get("length_m", 1.0))
	else:
		length = float(params.get("finnedTubeLength_m", params.get("length_m", 1.0)))

	if geom_type == "fin_tube":
		# Fin-and-tube geometry based on FinAndTubeGeometry record
		import math

		finned_tube_length = float(params.get("finnedTubeLength_m", length))
		n_serial = float(params.get("nSerialTubes", 1.0))
		serial_tube_distance = float(params.get("serialTubeDistance_m", 1e-3))
		n_parallel = float(params.get("nParallelTubes", 1.0))
		parallel_tube_distance = float(params.get("parallelTubeDistance_m", 1e-3))
		fin_thickness = float(params.get("finThickness_m", 1e-4))
		fin_pitch = float(params.get("finPitch_m", 1e-3))
		tube_inner_diameter = float(params.get("tubeInnerDiameter_m", 1e-3))
		tube_wall_thickness = float(params.get("tubeWallThickness_m", 1e-3))
		n_tube_side_parallel_flows = float(
			params.get("nTubeSideParallelHydraulicFlows", 1.0)
		)

		# Derived quantities from Modelica record
		total_hx_height = n_parallel * parallel_tube_distance
		total_hx_width = finned_tube_length
		total_hx_depth = n_serial * serial_tube_distance
		total_n_tubes = n_parallel * n_serial
		tube_outer_diameter = tube_inner_diameter + 2.0 * tube_wall_thickness
		n_fins = int(finned_tube_length / fin_pitch) + 1
		total_finned_tube_length = total_n_tubes * finned_tube_length

		# Single tube quantities
		single_tube_fin_surface_area = 2.0 * (
			serial_tube_distance * parallel_tube_distance
			- math.pi * tube_outer_diameter * tube_outer_diameter / 4.0
		) * n_fins
		single_tube_inner_surface_area = (
			math.pi * tube_inner_diameter * finned_tube_length
		)
		single_tube_outer_surface_area = (
			math.pi * tube_outer_diameter * finned_tube_length
			- math.pi * tube_outer_diameter * fin_thickness * n_fins
		)

		# Totals
		total_fin_side_heat_transfer_area = (
			(single_tube_fin_surface_area + single_tube_outer_surface_area)
			* total_n_tubes
		)
		total_tube_side_heat_transfer_area = (
			single_tube_inner_surface_area * total_n_tubes
		)

		# Representative hydraulic diameter on tube side: use tubeInnerDiameter
		# (could be refined later with an effective diameter on fin side)
		inner_area = total_tube_side_heat_transfer_area
		outer_area = total_fin_side_heat_transfer_area
		dh = tube_inner_diameter
		length = total_finned_tube_length / n_tube_side_parallel_flows

	elif geom_type == "plate":
		# Map Modelica record "PlateGeometry" logic to Python
		number_of_plates = float(params.get("numberOfPlates", 3.0))
		width = float(params.get("width_m", 1.0))
		wall_thickness = float(params.get("wallThickness_m", 1e-3))
		pattern_amplitude = float(params.get("patternAmplitude_m", 1e-3))
		pattern_wave_length = float(params.get("patternWaveLength_m", 1e-2))

		# height = (numberOfPlates - 1)*2*patternAmplitude + numberOfPlates*wallThickness
		# (not yet exposed, but kept for completeness of logic)
		_ = (number_of_plates - 1.0) * 2.0 * pattern_amplitude + number_of_plates * wall_thickness

		# nParallelHydraulicFlows = floor(numberOfPlates/2)
		n_parallel_flows = int(number_of_plates // 2)

		# meanAngle not required for now

		# waveNumber = (2*pi*patternAmplitude)/patternWaveLength
		import math
		wave_number = (2.0 * math.pi * pattern_amplitude) / pattern_wave_length

		# areaExpansionFactor = (1/6)*(1+sqrt(1+waveNumber^2)+4*sqrt(1+(waveNumber^2)/2))
		area_expansion_factor = (
			1.0
			/ 6.0
			* (
				1.0
				+ math.sqrt(1.0 + wave_number**2)
				+ 4.0 * math.sqrt(1.0 + 0.5 * wave_number**2)
			)
		)

		# heatTransferArea = (numberOfPlates - 2)*width*length*areaExpansionFactor
		outer_area = (number_of_plates - 2.0) * width * length * area_expansion_factor
		inner_area = outer_area

		# crossSectionalArea = width*2*patternAmplitude*nParallelHydraulicFlows
		cross_sectional_area = width * 2.0 * pattern_amplitude * n_parallel_flows
		_ = cross_sectional_area  # channel volume etc. not yet exposed

		# hydraulicDiameter = 4*patternAmplitude/areaExpansionFactor
		dh = 4.0 * pattern_amplitude / area_expansion_factor

	else:
		# Fallback: make everything proportional to length
		outer_area = length
		inner_area = length
		dh = 1.0

	# Try to extract a meaningful wall thickness for use in WallTransfer
	if geom_type == "plate":
		wall_thickness_val = float(params.get("wallThickness_m", 0.0))
	elif geom_type == "fin_tube":
		wall_thickness_val = float(params.get("tubeWallThickness_m", 0.0))
	else:
		wall_thickness_val = 0.0

	wall_thickness = wall_thickness_val if wall_thickness_val > 0 else None

	return GeometryResult(
		type=geom_type,
		outer_area_m2=outer_area,
		inner_area_m2=inner_area,
		length_m=length,
		hydraulic_diameter_m=dh,
		wall_thickness_m=wall_thickness,
		raw_data=params,
	)


def load_geometry(filename: str | Path) -> GeometryResult:
	"""Load a geometry TXT file and compute basic quantities.

	Parameters
	----------
	filename:
		Name of the TXT file. Relative paths are resolved relative to the
		``hx_model`` directory.
	"""

	# Accept both base names (e.g. "plate_demo") and explicit TXT paths
	name = str(filename)
	if not name.lower().endswith(".txt"):
		name = f"{name}.txt"

	params = _parse_geometry_txt(name)
	return _compute_geometry_placeholders(params)


def main() -> None:
	"""Small helper to quickly inspect a geometry file.

	Kann mit oder ohne Dateinamen als Argument aufgerufen werden.

	Beispiele (vom Projekt-Root)::

		python -m vclibpy.components.heat_exchangers.hx_model.geo_calc
		python -m vclibpy.components.heat_exchangers.hx_model.geo_calc plate_demo.txt
	"""

	import argparse

	parser = argparse.ArgumentParser(
		description="Compute basic HX geometry sizes from TXT file.",
	)
	parser.add_argument(
		"filename",
		nargs="?",
		help="Geometry TXT filename (relative to hx_model or absolute path)",
	)

	args = parser.parse_args()

	name = args.filename or input(
		"Geometrie (Basisname, z.B. plate_demo oder fin_tube_demo): "
	).strip()
	filename = name if name.endswith(".txt") else f"{name}.txt"

	geom = load_geometry(filename)

	print(f"Geometry type          : {geom.type}")
	print(f"Outer area      [m^2]  : {geom.outer_area_m2:.6g}")
	print(f"Inner area      [m^2]  : {geom.inner_area_m2:.6g}")
	print(f"Length          [m]    : {geom.length_m:.6g}")
	print(f"Hydraulic d     [m]    : {geom.hydraulic_diameter_m:.6g}")


if __name__ == "__main__":  # pragma: no cover - convenience entry point
	main()
