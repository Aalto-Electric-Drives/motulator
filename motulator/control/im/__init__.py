"""This package contains example controllers for induction machines."""

from motulator.control.im.vector import (
    ModelPars,
    CurrentReferencePars,
    VectorCtrl,
    CurrentReference,
    CurrentCtrl,
    Observer,
)

from motulator.control.im.obs_vhz import (
    ObserverBasedVHzCtrl, ObserverBasedVHzCtrlPars, SensorlessObserverExtCoord)

from motulator.control.im.vhz import VHzCtrl

from motulator.control.common import SpeedCtrl, RateLimiter
