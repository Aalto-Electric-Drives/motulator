# pylint: disable=C0103
"""
This script configures V/Hz control for an induction motor. The default values
correspond to a 45-kW induction motor.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from helpers import Step  # , Sequence
from control.im.vhz import VHzCtrl, Datalogger
from config.mdl_im_45kW import mdl


# %%
@dataclass
class BaseValues:
    """
    This data class contains the base values computed from the rated values.
    These are used for plotting the results.

    """
    # pylint: disable=too-many-instance-attributes
    w: float = 2*np.pi*50
    i: float = np.sqrt(2)*81
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
    This data class contains parameters for the V/Hz control system.

    """
    T_s: float = 250e-6
    psi_s_nom: float = BaseValues.psi
    rate_limit: float = 2*np.pi*120
    R_s: float = .057
    R_R: float = .029
    L_sgm: float = 2.2e-3
    L_M: float = 24.5e-3
    w_rb: float = R_R*(L_M + L_sgm)/(L_sgm*L_M)
    alpha_f: float = .1*w_rb
    alpha_i: float = .1*w_rb
    k_u: float = .4
    k_w: float = 3.


# %%
base = BaseValues()
pars = CtrlParameters()
# Open-loop V/Hz control can be obtained by choosing:
# pars.k_u, pars.k_w, pars.R_s, pars.R_R = 0, 0, 0, 0
datalog = Datalogger()
ctrl = VHzCtrl(pars, datalog)

# %% Profile: Acceleration and load torque step
# Speed reference
mdl.speed_ref = Step(2, .8*base.w)
# External load torque
mdl.mech.T_L_ext = Step(5, 291)
# Stop time of the simulation
mdl.t_stop = 7
# %% Profile: Speed reversals with constant load torque
# Speed reference
# times = 5*np.array([0, .5, 1, 1.5, 2, 2.5,  3, 3.5, 4])
# values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*2*np.pi*50
# mdl.speed_ref = Sequence(times, values)
# External load torque
# times = np.array([0, .5, .5, 3.5, 3.5, 4])*5
# values = np.array([0, 0, 1, 1, 0, 0])*291*.3
# mdl.mech.T_L_ext = Sequence(times, values)
# Stop time of the simulation
# mdl.t_stop = mdl.freq_ref.times[-1]

# %% Print the control system data
print('\nV/Hz control')
print('------------')
print('Sampling period:')
print('    T_s={}'.format(pars.T_s))
print(ctrl)

# %% Print the profiles
print('\nProfiles')
print('--------')
print('Speed reference:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.speed_ref))
print('External load torque:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.mech.T_L_ext))
