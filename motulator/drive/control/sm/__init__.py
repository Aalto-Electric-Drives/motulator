"""Controls for synchronous machines."""

from motulator.drive.control.sm._common import Observer, ObserverCfg
from motulator.drive.control.sm._flux_vector import (
    FluxTorqueReference,
    FluxTorqueReferenceCfg,
    FluxVectorControl,
)
from motulator.drive.control.sm._current_vector import (
    CurrentController,
    CurrentReference,
    CurrentReferenceCfg,
    CurrentVectorControl,
)
from motulator.drive.control.sm._obs_vhz import (
    ObserverBasedVHzControl,
    ObserverBasedVHzControlCfg,
)
from motulator.drive.control.sm._signal_inj import (
    SignalInjection,
    SignalInjectionControl,
)
from motulator.drive.control.sm._torque import TorqueCharacteristics
from motulator.drive.control._common import SpeedController

__all__ = [
    "Observer",
    "ObserverCfg",
    "FluxTorqueReference",
    "FluxTorqueReferenceCfg",
    "FluxVectorControl",
    "CurrentController",
    "CurrentReference",
    "CurrentReferenceCfg",
    "CurrentVectorControl",
    "ObserverBasedVHzControl",
    "ObserverBasedVHzControlCfg",
    "SignalInjection",
    "SignalInjectionControl",
    "TorqueCharacteristics",
    "SpeedController",
]
