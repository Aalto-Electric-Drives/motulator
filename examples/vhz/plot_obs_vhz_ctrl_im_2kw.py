"""
Observer-based V/Hz control: 2.2-kW induction motor
===================================================

This example simulates observer-based V/Hz control of a 2.2-kW induction motor
drive.

"""

# %%
# Import the packages.

import numpy as np
import motulator.model.im as model
import motulator.control.im as control
from motulator.helpers import BaseValues, Sequence
from motulator.plots import plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# Configure the system model.

# Configure the induction machine using its inverse-Γ parameters
machine = model.InductionMachineInvGamma(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

# Inverse-Γ model parameter estimates
par = control.ModelPars(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
ctrl_par = control.ObserverBasedVHzCtrlPars(
    psi_s_nom=base.psi, i_s_max=1.5*base.i)
ctrl = control.ObserverBasedVHzCtrl(par, ctrl_par, T_s=250e-6)

# %%
# Set the speed reference.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = Sequence(times, values)

# %%
# Set the load torque reference

# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mechanics.tau_L_t = Sequence(times, values)

# Quadratic load torque profile, e.g. pumps and fans (uncomment to enable)
# k = 1.1*base.tau_nom/(base.w/base.p)**2
# mdl.mech.tau_L_w = lambda w_M: np.sign(w_M)*k*w_M**2

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = model.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
