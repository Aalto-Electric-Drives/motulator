"""This package contains example controllers for induction machines."""

from motulator.control.im._current_vector import (
    CurrentCtrl,
    CurrentReferencePars,
    CurrentReference,
    ModelPars,
    CurrentVectorCtrl,
)

from motulator.control.im._common import Observer, ObserverPars
from motulator.control.im._common import FullOrderObserver, FullOrderObserverPars

from motulator.control.im._vhz import VHzCtrl, VHzCtrlPars

__all__ = [
    "CurrentCtrl",
    "CurrentReferencePars",
    "CurrentReference",
    "ModelPars",
    "CurrentVectorCtrl",
    "Observer",
    "ObserverPars",
    "FullOrderObserver",
    "FullOrderObserverPars",
    "VHzCtrl",
    "VHzCtrlPars",
]
