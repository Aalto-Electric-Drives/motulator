# pylint: disable=C0103
"""
This script configures sensorless vector control for a 2.2-kW permanent-magnet
synchronous motor drive.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.interpolate import interp1d
from control.common import SpeedCtrl
from control.sm.vector import CurrentRef, CurrentCtrl, SensorlessObserver
from control.sm.vector import SensorlessVectorCtrl, Datalogger
from control.sm.torque import TorqueCharacteristics
from helpers import Sequence  # , Step
from config.mdl_pmsm_2kW import mdl


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
    # Sampling period
    T_s: float = 250e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*100
    alpha_fw: float = 2*np.pi*20
    alpha_s: float = 2*np.pi*4
    # Observer
    w_o: float = 2*np.pi*40
    # Maximum values
    tau_M_max: float = 1.5*14.0
    i_s_max: float = 2*np.sqrt(2)*4.3
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
    # LUTs to be dedined subsequently
    i_sd_mtpa: float = None
    i_sq_mtpv: float = None


# %%
base = BaseValues()
pars = CtrlParameters()
tq = TorqueCharacteristics(pars)

# %% Generate LUTs
i_s_max_lut = 2*pars.i_s_max  # LUTs are computed up to this current
i_s_mtpa = tq.mtpa_locus(i_s_max_lut)
psi_s_mtpa = tq.flux(i_s_mtpa)
tau_M_mtpa = tq.torque(psi_s_mtpa)
pars.i_sd_mtpa = interp1d(tau_M_mtpa, i_s_mtpa.real)
psi_s_max_lut = tq.flux(tq.mtpv_current(i_s_max_lut))
# psi_s_max_lut = 2*base.psi  # Simpler version
psi_s_mtpv = tq.mtpv_locus(np.abs(psi_s_max_lut))
i_s_mtpv = tq.current(psi_s_mtpv)
pars.i_sq_mtpv = interp1d(i_s_mtpv.real, i_s_mtpv.imag, bounds_error=False)

# Plot loci and characteristics
tq.plot_current_loci(i_s_max_lut, base)
tq.plot_flux_loci(i_s_max_lut, base)
tq.plot_torque_current(i_s_max_lut, base)
# tq.plot_angle_torque(base.psi, base)

# %% Choose controller
speed_ctrl = SpeedCtrl(pars)
current_ref = CurrentRef(pars)
current_ctrl = CurrentCtrl(pars)
observer = SensorlessObserver(pars)
datalog = Datalogger()
ctrl = SensorlessVectorCtrl(pars, speed_ctrl, current_ref, current_ctrl,
                            observer, datalog)

# %% Profiles
# Speed reference
times = np.array([0, .5, 1, 1.5, 2, 2.5,  3, 3.5, 4])
values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*base.w
mdl.speed_ref = Sequence(times, values)
# External load torque
times = np.array([0, .5, .5, 3.5, 3.5, 4])
values = np.array([0, 0, 1, 1, 0, 0])*14.0  # *.5
mdl.mech.tau_L_ext = Sequence(times, values)  # tau_L_ext = Step(1, 14.6)
# Stop time of the simulation
mdl.t_stop = mdl.speed_ref.times[-1]

# %% Print the control system data
print('\nSensorless vector control')
print('-------------------------')
print('Sampling period:')
print('    T_s={}'.format(pars.T_s))
print('Motor parameter estimates:')
print(('    p={}  R_s={}  L_d={}  L_q={}  psi_f={}'
       ).format(pars.p, pars.R_s, pars.L_d, pars.L_q, pars.psi_f))
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
    print('    {}'.format(mdl.mech.tau_L_ext))
