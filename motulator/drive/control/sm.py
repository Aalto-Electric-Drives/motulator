"""Controls for synchronous machine drives."""

from motulator.common.control._pwm import PWM, InverterNonlinearity
from motulator.drive.control._base import (
    DriveControlSystem,
    VectorControlSystem,
    VHzControlSystem,
)
from motulator.drive.control._common import (
    PIController,
    References,
    SpeedController,
    SpeedObserver,
)
from motulator.drive.control._sm_current_vector import (
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
from motulator.drive.control._sm_reference_gen import (
    ReferenceGenerator,
    ReferenceGeneratorOnline,
)
from motulator.drive.control._sm_signal_inj import SignalInjectionController
from motulator.drive.utils._parameters import (
    BaseSynchronousMachineControlPars,
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

__all__ = [
    "BaseSynchronousMachineControlPars",
    "DriveControlSystem",
    "PWM",
    "InverterNonlinearity",
    "PIController",
    "CurrentVectorControllerCfg",
    "VHzControlSystem",
    "VectorControlSystem",
    "FluxObserver",
    "SpeedFluxObserver",
    "CurrentVectorController",
    "FluxVectorController",
    "FluxVectorControllerCfg",
    "ObserverBasedVHzController",
    "ObserverBasedVHzControllerCfg",
    "ObserverOutputs",
    "ReferenceGenerator",
    "References",
    "ReferenceGeneratorOnline",
    "SaturatedSynchronousMachinePars",
    "SignalInjectionController",
    "SpeedController",
    "SpeedObserver",
    "SynchronousMachinePars",
]
