# pylint: disable=wrong-import-position
# pylint: disable=import-error
"""
motulator: Motor Drive Simulator in Python

"""
import sys
import os

PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(PATH)

# Import simulation environment
from simulation import Simulation

# Import system models
from model.mech import Mechanics
from model.converter import (
    Inverter,
    PWMInverter,
    FrequencyConverter)
from model.im import (
    InductionMotor,
    InductionMotorSaturated,
    SaturableStatorInductance)
from model.sm import SynchronousMotor
from model.im_drive import InductionMotorDrive, InductionMotorDriveDiode
from model.sm_drive import SynchronousMotorDrive

# Import controllers
from control.im_vhz import (
    InductionMotorVHzCtrl,
    InductionMotorVHzCtrlPars)
from control.im_vector import (
    InductionMotorVectorCtrl,
    InductionMotorVectorCtrlPars)
from control.sm_vector import (
    SynchronousMotorVectorCtrl,
    SynchronousMotorVectorCtrlPars)

# Import other useful stuff
from helpers import (
    BaseValues,
    abc2complex,
    complex2abc,
    Sequence,
    Step)

# Import some default plotting functions
from plots import (
    plot,
    plot_pu,
    plot_extra_pu)

# Delete imported modules
del sys, os, PATH
