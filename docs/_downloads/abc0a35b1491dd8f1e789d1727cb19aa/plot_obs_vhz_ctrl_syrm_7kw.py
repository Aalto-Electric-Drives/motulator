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
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, p=2)

# %%
# Configure the system model.

# Saturated SyRM model
motor = mt.SynchronousMotorSaturated()
# Magnetically linear SyRM model
# motor = mt.SynchronousMotor(p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter()
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVHzObsCtrlPars(
    p=2,
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
