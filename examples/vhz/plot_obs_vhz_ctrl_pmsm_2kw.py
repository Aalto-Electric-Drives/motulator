"""
Observer-based V/Hz control: 2.2-kW PMSM
========================================

This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.

"""

# %%
# Import the package.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

mdl = mt.SynchronousMotorDrive()
mdl.motor = mt.SynchronousMotor(n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mdl.mech = mt.Mechanics(J=.015)
mdl.conv = mt.Inverter(u_dc=540)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVHzObsCtrlPars()
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
