# pylint: disable=C0103
"""
This script configures sensored vector control for a 2.2-kW induction motor
drive.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from control.common import SpeedCtrl
from control.im.vector import CurrentModelEstimator
from control.im.vector import CurrentRef, CurrentCtrl2DOFPI
from control.im.vector import VectorCtrl, Datalogger
from helpers import Sequence  # , Step
from config.mdl_im_2kW import mdl


# %%
@dataclass
class BaseValues:
    """
    This data class contains the base values computed from the rated values.
    These are used for plotting the results.

    """
    # pylint: disable=too-many-instance-attributes
    w: float = 2*np.pi*50
    i: float = np.sqrt(2)*5
    u: float = np.sqrt(2/3)*400
    p: int = 2
    psi: float = u/w
    P: float = 1.5*u*i
    Z: float = u/i
    L: float = Z/w
    T: float = p*P/w


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the control system.

    """
    # pylint: disable=too-many-instance-attributes
    # Sampling period
    T_s: float = 250e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*100
    alpha_s: float = 2*np.pi*4
    # Maximum values
    T_M_max: float = 1.5*14.6
    i_s_max: float = 1.5*np.sqrt(2)*5
    # Nominal values
    psi_R_nom: float = .9
    u_dc_nom: float = 540
    # Motor parameter estimates
    R_s: float = 3.7
    R_R: float = 2.1
    L_sgm: float = .021
    L_M: float = .224
    p: int = 2
    J: float = .015


# %% Choose controller
base = BaseValues()
pars = CtrlParameters()
speed_ctrl = SpeedCtrl(pars)
current_ref = CurrentRef(pars)
current_ctrl = CurrentCtrl2DOFPI(pars)
observer = CurrentModelEstimator(pars)
datalog = Datalogger()
ctrl = VectorCtrl(pars, speed_ctrl, current_ref, current_ctrl,
                  observer, datalog)

# %% Profiles
# Speed reference
times = np.array([0, .5, 1, 1.5, 2, 2.5,  3, 3.5, 4])
values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*2*np.pi*50
mdl.speed_ref = Sequence(times, values)
# External load torque
times = np.array([0, .5, .5, 3.5, 3.5, 4])
values = np.array([0, 0, 1, 1, 0, 0])*14.6
mdl.mech.T_L_ext = Sequence(times, values)  # T_L_ext = Step(1, 14.6)
# Stop time of the simulation
mdl.t_stop = mdl.speed_ref.times[-1]

# %% Print the control system data
print('\nSensored vector control')
print('-----------------------')
print('Sampling period:')
print('    T_s={}'.format(pars.T_s))
print('Motor parameter estimates:')
print(('    p={}  R_s={}  R_R={}  L_sgm={}  L_M={}'
       ).format(pars.p, pars.R_s, pars.R_R, pars.L_sgm, pars.L_M))
print(current_ref)
print(current_ctrl)
print(observer)
print(speed_ctrl)

# %% Print the profiles
print('\nProfiles')
print('--------')
print('Speed reference:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.speed_ref))
print('External load torque:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.mech.T_L_ext))
