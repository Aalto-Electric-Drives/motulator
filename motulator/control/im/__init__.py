"""This package contains example controllers for induction machines."""

from motulator.control.im._vector import (
    CurrentCtrl,
    CurrentReferencePars,
    CurrentReference,
    ModelPars,
    VectorCtrl,
)

from motulator.control.im._observers import (Observer, FluxObserver)

from motulator.control.im._obs_vhz import (
    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlPars)

from motulator.control.im._vhz import VHzCtrl

__all__ = [
    "CurrentCtrl", "CurrentReferencePars", "CurrentReference", "ModelPars",
    "VectorCtrl", "Observer", "FluxObserver", "ObserverBasedVHzCtrl",
    "ObserverBasedVHzCtrlPars", "VHzCtrl"
]
