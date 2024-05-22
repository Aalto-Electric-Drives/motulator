"""This package contains example controllers for synchronous machines."""

from motulator.control.sm._current_vector import (
    CurrentReferencePars,
    CurrentReference,
    CurrentCtrl,
    CurrentVectorCtrl,
)

from motulator.control.sm._common import ModelPars, Observer, ObserverPars

from motulator.control.sm._flux_vector import (
    FluxVectorCtrl, FluxTorqueReference, FluxTorqueReferencePars)
#from motulator.control.sm._obs_vhz import (
#    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlPars)
from motulator.control.sm._torque import TorqueCharacteristics
from motulator.control.sm._signal_inj import (
    SignalInjectionCtrl, SignalInjection)

__all__ = [
    "CurrentReferencePars", "CurrentReference", "CurrentCtrl", "ModelPars",
    "Observer", "ObserverPars", "CurrentVectorCtrl", "FluxVectorCtrl", "FluxTorqueReference",
    "FluxTorqueReferencePars", "TorqueCharacteristics", "SignalInjectionCtrl",
    "SignalInjection"
]
