# pylint: disable=C0103
"""
This script configures sensorless vector control for a 2.2-kW induction motor
drive.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from control.common import SpeedCtrl
from control.im.vector import SensorlessObserver, CurrentModelEstimator
from control.im.vector import CurrentRef, CurrentCtrl, VectorCtrl
from config.mdl_im_2kW import mdl
# pylint: disable=unused-import
from helpers import plot, ref_ramp, ref_step


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
    tau: float = p*P/w


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the control system.

    """
    # pylint: disable=too-many-instance-attributes
    sensorless: bool = True
    # Sampling period
    T_s: float = 250e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*200
    alpha_o: float = 2*np.pi*40  # Used only in the sensorless mode
    alpha_s: float = 2*np.pi*4
    # Maximum values
    tau_M_max: float = 1.5*14.6
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


# %%
base = BaseValues()
pars = CtrlParameters()

# %% Configure controller
speed_ctrl = SpeedCtrl(pars)
current_ref = CurrentRef(pars)
current_ctrl = CurrentCtrl(pars)

if pars.sensorless:
    observer = SensorlessObserver(pars)
else:
    observer = CurrentModelEstimator(pars)

ctrl = VectorCtrl(pars, speed_ctrl, current_ref, current_ctrl, observer)

# %% Speed refrerence and load torque profiles
ref_ramp(mdl, w_max=base.w, tau_max=14.6, t_max=4)
