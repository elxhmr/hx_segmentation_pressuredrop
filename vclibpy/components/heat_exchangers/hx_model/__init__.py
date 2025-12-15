"""HX geometry models and helpers.

This package currently provides utilities to load simple TXT-based
geometry definitions (see ``fin_tube_demo.txt`` and ``plate_demo.txt``)
and compute key geometric quantities for pressure-drop and heat-transfer
correlations.
"""

from .geo_calc import GeometryResult, load_geometry

# Backwards-compatible alias used by older examples
loadgeometry = load_geometry

__all__ = [
        "GeometryResult",
        "load_geometry",
        "loadgeometry",
]
