"""This package contains example controllers for induction machines."""

from motulator.control.im._vector import (
    CurrentCtrl,
    CurrentReferencePars,
    CurrentReference,
    ModelPars,
    Observer,
    VectorCtrl,
)

from motulator.control.im._obs_vhz import (
    FluxObserver, ObserverBasedVHzCtrl, ObserverBasedVHzCtrlPars)

from motulator.control.im._vhz import VHzCtrl

# from motulator.control._common import RateLimiter, SpeedCtrl
