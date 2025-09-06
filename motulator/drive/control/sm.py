"""Controls for synchronous machine drives."""

from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._common import PIController, SpeedController, SpeedObserver
from motulator.drive.control._sm_current_vector import (
    CurrentController,
    CurrentVectorController,
    CurrentVectorControllerCfg,
)
from motulator.drive.control._sm_flux_vector import (
    FluxVectorController,
    FluxVectorControllerCfg,
    ObserverBasedVHzController,
    ObserverBasedVHzControllerCfg,
)
from motulator.drive.control._sm_observers import (
    FluxObserver,
    ObserverOutputs,
    SpeedFluxObserver,
)
from motulator.drive.control._sm_reference_gen import ReferenceGenerator
from motulator.drive.control._sm_signal_inj import SignalInjectionController
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

__all__ = [
    "PIController",
    "CurrentVectorControllerCfg",
    "VHzControlSystem",
    "VectorControlSystem",
    "FluxObserver",
    "SpeedFluxObserver",
    "CurrentController",
    "CurrentVectorController",
    "FluxVectorController",
    "FluxVectorControllerCfg",
    "ObserverBasedVHzController",
    "ObserverBasedVHzControllerCfg",
    "ObserverOutputs",
    "ReferenceGenerator",
    "SaturatedSynchronousMachinePars",
    "SignalInjectionController",
    "SpeedController",
    "SpeedObserver",
    "SynchronousMachinePars",
]
