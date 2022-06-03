# pylint: disable=C0103
"""
This script configures sensorless vector control for a 2.2-kW permanent-magnet
synchronous motor drive.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from control.common import SpeedCtrl
from control.sm.vector import CurrentRef, CurrentCtrl
from control.sm.vector import VectorCtrl, SensorlessObserver
from control.sm.torque import TorqueCharacteristics
from config.mdl_pmsm_2kW import mdl
# pylint: disable=unused-import
from helpers import plot, ref_ramp


# %%
@dataclass
class BaseValues:
    """
    This data class contains the base values computed from the rated values.
    These are used for plotting the results.

    """
    # pylint: disable=too-many-instance-attributes
    w: float = 2*np.pi*75
    i: float = np.sqrt(2)*4.3
    u: float = np.sqrt(2/3)*370
    p: int = 3
    psi: float = u/w
    P: float = 1.5*u*i
    Z: float = u/i
    L: float = Z/w
    tau: float = p*P/w


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the control system of a 2.2-kW
    permanent-magnet synchronous motor.

    """
    # pylint: disable=too-many-instance-attributes
    sensorless: bool = True
    # Sampling period
    T_s: float = 250e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*200
    alpha_fw: float = 2*np.pi*20
    alpha_s: float = 2*np.pi*4
    # Maximum values
    tau_M_max: float = 2*14.6
    i_s_max: float = 1.5*np.sqrt(2)*5
    # Voltage margin
    k_u: float = .95
    # Nominal values
    u_dc_nom: float = 540
    w_nom: float = 2*np.pi*75
    # Motor parameter estimates
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    p: int = 3
    J: float = .015
    # Sensorless observer
    w_o: float = 2*np.pi*40  # Used only in the sensorless mode
    # Look-up tables to be generated subsequently
    i_sd_mtpa = None
    i_sd_lim = None
    tau_M_lim = None


# %%
base = BaseValues()
pars = CtrlParameters()

# %% Generate LUTs
tq = TorqueCharacteristics(pars)
mtpa = tq.mtpa_locus(i_s_max=pars.i_s_max)
lims = tq.mtpv_and_current_limits(i_s_max=pars.i_s_max)
# MTPA locus
pars.i_sd_mtpa = mtpa.i_sd_vs_tau_M
# Merged MTPV and current limits
pars.tau_M_lim = lims.tau_M_vs_abs_psi_s
pars.i_sd_lim = lims.i_sd_vs_tau_M

# Plot loci and characteristics
if __name__ == '__main__':
    tq.plot_current_loci(pars.i_s_max, base)
    # tq.plot_flux_loci(pars.i_s_max, base)
    tq.plot_torque_flux(pars.i_s_max, base)
    tq.plot_torque_current(pars.i_s_max, base)
    # tq.plot_angle_torque(base.psi, base)

# %% Configure controller
speed_ctrl = SpeedCtrl(pars)
current_ref = CurrentRef(pars)
current_ctrl = CurrentCtrl(pars)
observer = None
if pars.sensorless:
    observer = SensorlessObserver(pars)

ctrl = VectorCtrl(pars, speed_ctrl, current_ref, current_ctrl, observer)

# %% Speed refrerence and load torque profiles
ref_ramp(mdl, w_max=base.w, tau_max=14.6, t_max=4)
