"""This package contains example controllers for synchronous machines."""

from motulator.control.sm._vector import (
    CurrentReferencePars,
    CurrentReference,
    CurrentCtrl,
    ModelPars,
    VectorCtrl,
)

from motulator.control.sm._observers import (Observer, FluxObserver)

from motulator.control.sm._flux_vector import (
    FluxVectorCtrl, FluxTorqueReference, FluxTorqueReferencePars)
from motulator.control.sm._obs_vhz import (
    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlPars)
from motulator.control.sm._torque import TorqueCharacteristics
from motulator.control.sm._signal_inj import (
    SignalInjectionCtrl, SignalInjection)

__all__ = [
    "CurrentReferencePars", "CurrentReference", "CurrentCtrl", "ModelPars",
    "Observer", "VectorCtrl", "Observer", "FluxObserver", "FluxVectorCtrl",
    "FluxTorqueReference", "FluxTorqueReferencePars", "ObserverBasedVHzCtrl",
    "ObserverBasedVHzCtrlPars", "FluxObserver", "TorqueCharacteristics",
    "SignalInjectionCtrl", "SignalInjection"
]
