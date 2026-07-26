"""Utility functions for machine drives."""

import motulator.drive.utils._gn as gn
from motulator.common.utils._utils import (
    BaseValues,
    NominalValues,
    SequenceGenerator,
    Step,
)
from motulator.drive.utils._gn import (
    CurrentMap,
    CurrentMapWithHarmonics,
    FluxMap,
    FluxMapWithHarmonics,
)
from motulator.drive.utils._gn_dataset import get_training_data
from motulator.drive.utils._gn_plot import (
    PlotOptions,
    plot_gn_map,
    plot_output_vs_angle,
    plot_surface_vs_current_and_angle,
    sample_map_on_grid,
)
from motulator.drive.utils._gn_statistical import (
    print_meas_current_map_error_metrics,
    print_meas_flux_map_error_metrics,
    stat_fem,
    stat_fem_curr,
)
from motulator.drive.utils._gn_train import train_gradnet
from motulator.drive.utils._plots import (
    plot,
    plot_dc_bus_waveforms,
    plot_stator_waveforms,
)
from motulator.drive.utils._sm_control_loci import ControlLoci
from motulator.drive.utils._sm_flux_maps import (
    MagneticModel,
    SaturationModelBase,
    SaturationModelPMSyRM,
    SaturationModelSyRM,
    import_syre_data,
)
from motulator.drive.utils._sm_plot_control_loci import MachineCharacteristics
from motulator.drive.utils._sm_plot_flux_maps import plot_flux_vs_current, plot_map

__all__ = [
    "BaseValues",
    "ControlLoci",
    "import_syre_data",
    "MachineCharacteristics",
    "MagneticModel",
    "NominalValues",
    "plot",
    "plot_stator_waveforms",
    "plot_dc_bus_waveforms",
    "plot_flux_vs_current",
    "plot_gn_map",
    "SaturationModelPMSyRM",
    "SaturationModelSyRM",
    "SaturationModelBase",
    "SequenceGenerator",
    "Step",
    "gn",
    "get_training_data",
    "train_gradnet",
    "plot_map",
    "sample_map_on_grid",
    "print_meas_current_map_error_metrics",
    "stat_fem_curr",
    "plot_surface_vs_current_and_angle",
    "PlotOptions",
    "plot_output_vs_angle",
    "print_meas_flux_map_error_metrics",
    "stat_fem",
    "CurrentMap",
    "FluxMap",
    "CurrentMapWithHarmonics",
    "FluxMapWithHarmonics",
]
