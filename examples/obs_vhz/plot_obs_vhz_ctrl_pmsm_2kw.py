"""
2.2-kW PMSM
===========

This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.

"""

# %%
# Imports.

import numpy as np
from motulator import model, control
from motulator import BaseValues, Sequence, plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

machine = model.sm.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.sm.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

par = control.sm.ModelPars(n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
ctrl_par = control.sm.ObserverBasedVHzCtrlPars(par, i_s_max=1.5*base.i)
ctrl = control.sm.ObserverBasedVHzCtrl(par, ctrl_par, T_s=250e-6)
#ctrl.rate_limiter = control.RateLimiter(2*np.pi*120)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mechanics.tau_L_t = Sequence(times, values)

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = model.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=8)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
