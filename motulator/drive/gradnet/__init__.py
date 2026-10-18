"""
Gradient-based neural network maps.

The trained networks (see `load_gradnet`) and the maps based on them (e.g.,
`CurrentMap`) are evaluated with NumPy. Training the networks (see `training`)
requires PyTorch.

"""

from motulator.drive.gradnet._maps import (
    CurrentMap,
    CurrentMapWithHarmonics,
    FluxMap,
    FluxMapWithHarmonics,
    GradNetFunction,
    load_gradnet,
)
from motulator.drive.gradnet._plot import (
    PlotOptions,
    plot_output_vs_angle,
    plot_surface_vs_current_and_angle,
)
from motulator.drive.gradnet._stats import (
    print_current_map_errors_fem,
    print_current_map_errors_meas,
    print_flux_map_errors_fem,
    print_flux_map_errors_meas,
)

__all__ = [
    "CurrentMap",
    "CurrentMapWithHarmonics",
    "FluxMap",
    "FluxMapWithHarmonics",
    "GradNetFunction",
    "PlotOptions",
    "plot_output_vs_angle",
    "plot_surface_vs_current_and_angle",
    "print_current_map_errors_fem",
    "print_current_map_errors_meas",
    "print_flux_map_errors_fem",
    "print_flux_map_errors_meas",
    "load_gradnet",
]
