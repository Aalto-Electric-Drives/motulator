# pylint: disable=C0103
"""
This script configures V/Hz control for an induction motor. The default values
correspond to a 45-kW induction motor.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from control.im.vhz import VHzCtrl
from config.mdl_im_45kW import mdl
# pylint: disable=unused-import
from helpers import ref_step, plot


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
    tau: float = p*P/w


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the V/Hz control system.

    """
    # pylint: disable=too-many-instance-attributes
    sensorless: bool = True
    T_s: float = 2*250e-6
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

# %% Configure controller
#  Open-loop V/Hz control can be obtained by choosing:
# pars.k_u, pars.k_w, pars.R_s, pars.R_R = 0, 0, 0, 0
ctrl = VHzCtrl(pars)
print(ctrl)

# %% Speed refrerence and load torque profiles
ref_step(mdl, w_max=.8*base.w, tau_max=291, t_max=5)
