"""Gradient-based neural network maps."""

from motulator.drive.gradnet._dataset import get_training_data
from motulator.drive.gradnet._gn import (
    AlgebraicSigmoid,
    CurrentMap,
    CurrentMapWithHarmonics,
    FluxMap,
    FluxMapWithHarmonics,
    PNormGradient,
    Softmax,
    Squareplus,
    load_gradnet,
)
from motulator.drive.gradnet._plot import (
    PlotOptions,
    plot_maps,
    plot_output_vs_angle,
    plot_surface_vs_current_and_angle,
    sample_map_on_grid,
)
from motulator.drive.gradnet._stats import (
    print_current_map_errors_fem,
    print_current_map_errors_meas,
    print_flux_map_errors_fem,
    print_flux_map_errors_meas,
)
from motulator.drive.gradnet._train import train_gradnet

__all__ = [
    "AlgebraicSigmoid",
    "Squareplus",
    "PNormGradient",
    "Softmax",
    "CurrentMap",
    "CurrentMapWithHarmonics",
    "FluxMap",
    "FluxMapWithHarmonics",
    "get_training_data",
    "PlotOptions",
    "plot_maps",
    "plot_output_vs_angle",
    "plot_surface_vs_current_and_angle",
    "sample_map_on_grid",
    "print_current_map_errors_fem",
    "print_current_map_errors_meas",
    "print_flux_map_errors_fem",
    "print_flux_map_errors_meas",
    "train_gradnet",
    "load_gradnet",
    "get_training_data",
]
