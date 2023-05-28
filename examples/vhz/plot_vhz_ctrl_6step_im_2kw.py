"""
2.2-kW induction motor, 6-step mode
===================================

This example simulates V/Hz of a 2.2-kW induction motor drive. The six-step
overmodulation is enabled, which increases the fundamental voltage as well as
the harmonics. Since the PWM is not synchronized with the stator frequency, the
harmonic content also depends on the ratio between the stator frequency and the 
sampling frequency.

"""
# %%
# Imports.

import numpy as np
from motulator import model, control
from motulator import BaseValues, Sequence, plot, plot_extra

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# Create the system model.

# Configure the induction machine using its inverse-Γ parameters
machine = model.im.InductionMachineInvGamma(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.im.Drive(machine, mechanics, converter)

# %%
# Control system (parametrized as open-loop V/Hz control).

par = control.im.ModelPars(R_s=0*3.7, R_R=0*2.1, L_sgm=.021, L_M=.224)
ctrl = control.im.VHzCtrl(
    250e-6, par, psi_s_nom=base.psi, k_u=0, k_w=0, six_step=True)
ctrl.rate_limiter = control.RateLimiter(2*np.pi*120)

# %%
# Set the speed reference and the external load torque. More complicated
# signals could be defined as functions.

# Speed reference
times = np.array([0, .1, .3, 1])*2
values = np.array([0, 0, 1, 1])*base.w*2
ctrl.w_m_ref = Sequence(times, values)

# Quadratic load torque profile (corresponding to pumps and fans)
k = .2*base.tau_nom/(base.w/base.n_p)**2
mdl.mechanics.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)
# External load torque could be set here, now zero
mdl.mechanics.tau_L_t = lambda t: (t > 1.)*base.tau_nom*0

# %%
# Create the simulation object and simulate it. The option ``pwm=True`` enables
# the model for the carrier comparison.

sim = model.Simulation(mdl, ctrl, pwm=True)
sim.simulate(t_stop=2)

# %%
# Plot results in per-unit values.

# sphinx_gallery_thumbnail_number = 2
plot(sim, base)
plot_extra(sim, base, t_span=(0.58, 0.7))
