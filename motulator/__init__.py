"""
motulator: Motor Drive Simulator in Python

"""
import sys
import os

sys.path.append(
    os.path.dirname(os.path.abspath(__file__)))

# Import simulation environment
# pylint: disable=wrong-import-position
from motulator.simulation import Simulation

# Import system models
from motulator.model.mech import Mechanics
from motulator.model.converter import (
    Inverter,
    FrequencyConverter)
from motulator.model.im import (
    InductionMotor,
    InductionMotorSaturated,
    SaturableStatorInductance,
    InductionMotorInvGamma)
from motulator.model.sm import SynchronousMotor
from motulator.model.im_drive import (
    InductionMotorDrive,
    InductionMotorDriveDiode)
from motulator.model.sm_drive import SynchronousMotorDrive

# Import controllers
from motulator.control.im_vhz import (
    InductionMotorVHzCtrl,
    InductionMotorVHzCtrlPars)
from motulator.control.im_vector import (
    InductionMotorVectorCtrl,
    InductionMotorVectorCtrlPars)
from motulator.control.sm_vector import (
    SynchronousMotorVectorCtrl,
    SynchronousMotorVectorCtrlPars)

# Import other useful stuff
from motulator.helpers import (
    BaseValues,
    abc2complex,
    complex2abc,
    Sequence,
    Step)

# Import some default plotting functions
from motulator.plots import (
    plot,
    plot_pu,
    plot_extra_pu)

# Delete imported modules
del sys, os
