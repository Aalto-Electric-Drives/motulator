# pylint: disable=C0103
"""
This script configures V/Hz control for an induction motor. The default values
correspond to a 45-kW induction motor.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from helpers import Step
from control.im.scalar import ScalarCtrl, Datalogger
from config.mdl_im_45kW import mdl


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the V/Hz control system.

    """
    T_s: float = 250e-6*2
    psi_s_nom: float = np.sqrt(2/3)*400/(2*np.pi*50)
    k_u: float = .4
    k_w: float = 4
    rate_limit: float = 2*np.pi*120
    R_s: float = .057
    R_R: float = .029
    L_sgm: float = 2.2e-3
    L_M: float = 24.5e-3


# %%
pars = CtrlParameters()
# Open-loop V/Hz control can be obtained by choosing:
# pars.k_u, pars.k_w, pars.R_s, pars.R_R = 0, 0, 0, 0
datalog = Datalogger()
ctrl = ScalarCtrl(pars, datalog)

# %% Profiles
# Speed reference
mdl.speed_ref = Step(2, 2*np.pi*50)
# External load torque
mdl.mech.T_L_ext = Step(5, 291)
# Stop time of the simulation
mdl.t_stop = 7

# %% Print the control system data
print('\nScalar control')
print('--------------')
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
