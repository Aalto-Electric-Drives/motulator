"""Controls for synchronous machines."""
from motulator.drive.control.sm._common import Observer, ObserverCfg
from motulator.drive.control.sm._flux_vector import (
    FluxTorqueReference, FluxTorqueReferenceCfg, FluxVectorCtrl)
from motulator.drive.control.sm._current_vector import (
    CurrentCtrl, CurrentReference, CurrentReferenceCfg, CurrentVectorCtrl)
from motulator.drive.control.sm._obs_vhz import (
    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlCfg)
from motulator.drive.control.sm._signal_inj import (
    SignalInjection, SignalInjectionCtrl)
from motulator.drive.control.sm._torque import TorqueCharacteristics
from motulator.drive.control._common import SpeedCtrl

__all__ = [
    "Observer",
    "ObserverCfg",
    "FluxTorqueReference",
    "FluxTorqueReferenceCfg",
    "FluxVectorCtrl",
    "CurrentCtrl",
    "CurrentReference",
    "CurrentReferenceCfg",
    "CurrentVectorCtrl",
    "ObserverBasedVHzCtrl",
    "ObserverBasedVHzCtrlCfg",
    "SignalInjection",
    "SignalInjectionCtrl",
    "TorqueCharacteristics",
    "SpeedCtrl",
]
