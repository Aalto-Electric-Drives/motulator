"""This package contains example controllers for synchronous machines."""

from motulator.control.sm._common import ModelPars, Observer, ObserverCfg

from motulator.control.sm._torque import TorqueCharacteristics

from motulator.control.sm._current_vector import (
    CurrentReferenceCfg,
    CurrentReference,
    CurrentCtrl,
    CurrentVectorCtrl,
)

from motulator.control.sm._flux_vector import (
    FluxVectorCtrl, FluxTorqueReference, FluxTorqueReferenceCfg)
from motulator.control.sm._obs_vhz import (
    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlCfg)
from motulator.control.sm._signal_inj import (
    SignalInjectionCtrl, SignalInjection)

__all__ = [
    "CurrentReferenceCfg",
    "CurrentReference",
    "CurrentCtrl",
    "ModelPars",
    "Observer",
    "ObserverCfg",
    "CurrentVectorCtrl",
    "FluxVectorCtrl",
    "FluxTorqueReference",
    "FluxTorqueReferenceCfg",
    "TorqueCharacteristics",
    "SignalInjectionCtrl",
    "SignalInjection",
    "ObserverBasedVHzCtrl",
    "ObserverBasedVHzCtrlCfg",
]
