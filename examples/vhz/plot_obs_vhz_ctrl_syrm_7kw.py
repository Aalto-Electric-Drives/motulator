"""
Observer-based V/Hz control: 6.7-kW SyRM
========================================

This example simulates observer-based V/Hz control of a saturated 6.7-kW
synchronous reluctance motor drive. The saturation is not taken into account
in the control method (only in the system model).

"""

# %%
# Import the package.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, n_p=2)

# %%
# A saturation model is created based on [1]_, [2]_. For simplicity, the
# saturation model parameters are hard-coded in function below, but the same
# model structure can also be used for other synchronous motors. For PM motors,
# the magnetomotive force (MMF) of the PMs can be modeled using constant current
# source `i_f` on the d-axis [2], [3]_. Correspondingly, this approach assumes
# that the MMFs of the d-axis current and of the PMs are in series. This model
# cannot capture the desaturation phenomenon of thin iron ribs, see [4]_ for
# details. For such motors, look-up tables can be used.


def i_s(psi_s):
    """
    Magnetic model for a 6.7-kW synchronous reluctance motor.

    Parameters
    ----------
    psi_s : complex
        Stator flux linkage (Vs).

    Returns
    -------
    complex
        Stator current (A).

    Notes
    -----
    For nonzero `i_f`, the initial value of the stator flux linkage `psi_s0` 
    needs to be solved, e.g., as follows::

    from scipy.optimize import minimize_scalar
    res = minimize_scalar(
        lambda psi_d: np.abs(
                    (a_d0 + a_dd*np.abs(psi_d)**S)*psi_d - i_f))
    psi_s0 = complex(res.x)

    """
    # Parameters
    a_d0, a_dd, S = 17.4, 373., 5  # d-axis self-saturation
    a_q0, a_qq, T = 52.1, 658., 1  # q-axis self-saturation
    a_dq, U, V = 1120., 1, 0  # Cross-saturation
    i_f = 0  # MMF of PMs
    # Inverse inductance functions
    G_d = a_d0 + a_dd*np.abs(psi_s.real)**S + (
        a_dq/(V + 2)*np.abs(psi_s.real)**U*np.abs(psi_s.imag)**(V + 2))
    G_q = a_q0 + a_qq*np.abs(psi_s.imag)**T + (
        a_dq/(U + 2)*np.abs(psi_s.real)**(U + 2)*np.abs(psi_s.imag)**V)
    # Stator current
    return G_d*psi_s.real - i_f + 1j*G_q*psi_s.imag


# %%
# Configure the system model.

motor = mt.SynchronousMotorSaturated(n_p=2, R_s=.54, current=i_s)

# Magnetically linear SyRM model
# motor = mt.SynchronousMotor(p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter(u_dc=540)
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVHzObsCtrlPars(
    n_p=2,
    R_s=.54,
    L_d=37e-3,
    L_q=6.2e-3,
    psi_f=0,
    psi_s_max=base.psi,
    psi_s_min=base.psi,
    i_s_max=2*base.i,
)
ctrl = mt.SynchronousMotorVHzObsCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = mt.Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mech.tau_L_t = mt.Sequence(times, values)

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=8)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

mt.plot(sim, base=base)

# %%
# References
# ----------
# .. [1] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi,
#    “Sensorless self-commissioning of synchronous reluctance motors at
#    standstill without rotor locking, ”IEEE Trans. Ind. Appl., 2017,
#    https://doi.org/10.1109/TIA.2016.2644624
#
# .. [2] Awan, Song, Saarakkala, Hinkkanen, "Optimal torque control of
#    saturated synchronous motors: plug-and-play method," IEEE Trans. Ind.
#    Appl., 2018, https://doi.org/10.1109/TIA.2018.2862410
#
# .. [3] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous
#    motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986,
#    https://doi.org/10.1109/TIA.1986.4504786
#
# .. [4] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate
#    modeling and performance analysis of IPM-PMASR motors," IEEE Trans. Ind.
#    Appl., 2009, https://doi.org/10.1109/TIA.2008.2009493
