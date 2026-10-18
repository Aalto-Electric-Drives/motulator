"""
Gradient-based neural network maps.

The magnetic maps (e.g., `CurrentMap`) evaluate trained gradient networks (GradNets)
with NumPy, see `load_gradnet`. The networks are trained with the related package
motulator-gradnet, https://github.com/Aalto-Electric-Drives/motulator-gradnet.

"""

from motulator.drive.gradnet._maps import (
    ACTIVATIONS,
    CurrentMap,
    CurrentMapWithHarmonics,
    FluxMap,
    FluxMapWithHarmonics,
    GradNetFunction,
    load_gradnet,
)

__all__ = [
    "ACTIVATIONS",
    "CurrentMap",
    "CurrentMapWithHarmonics",
    "FluxMap",
    "FluxMapWithHarmonics",
    "GradNetFunction",
    "load_gradnet",
]
