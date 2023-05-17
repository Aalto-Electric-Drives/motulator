"""This package contains continuous-time system models."""

from motulator.simulation import Simulation

# Import system models
from motulator.model.mechanics import Mechanics, MechanicsTwoMass
from motulator.model.converter import Inverter  # , FrequencyConverter

from motulator.model.sm.drive import (
    Drive,
    DriveTwoMassMechanics,
    SynchronousMachine,
    SynchronousMachineSaturated,
)

from motulator.model.sm.flux_maps import (
    import_syre_data,
    plot_flux_map,
    plot_flux_vs_current,
)
