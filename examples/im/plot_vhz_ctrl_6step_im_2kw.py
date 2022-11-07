"""
V/Hz control up to the six-step mode: 2.2-kW induction motor
============================================================

This example simulates V/Hz of a 2.2-kW induction motor drive. The six-step
overmodulation is enabled, which increases the fundamental voltage as well as
the harmonics. Since the PWM is not synchronized with the stator freuqency, the
harmonic content also depends on the ratio between the stator frequency and
the sampling frequency.

"""
# %%
# Import the package.

import numpy as np
import motulator as mt
from time import time

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, p=2)

# %%
# Create the system model.

# Configure the induction motor using its inverse-Γ parameters
motor = mt.InductionMotorInvGamma(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, p=2)

mech = mt.Mechanics(J=.016)  # Mechanics model
conv = mt.Inverter(u_dc=540)  # Inverter model
mdl = mt.InductionMotorDrive(motor, mech, conv)  # System model

# %%
# Control system (parametrized as open-loop V/Hz control).

ctrl = mt.InductionMotorVHzCtrl(
    mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0, six_step=True,
                                 T_s=250e-6))

# %%
# Set the speed reference and the external load torque. More complicated
# signals could be defined as functions.

# Speed reference
times = np.array([0, .1, .3, 1])*2
values = np.array([0, 0, 1, 1])*base.w*2
ctrl.w_m_ref = mt.Sequence(times, values)

# Quadratic load torque profile (corresponding to pumps and fans)
k = .2*base.tau_nom/(base.w/base.p)**2
mdl.mech.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)
# External load torque could be set here, now zero
mdl.mech.tau_L_t = lambda t: (t > 1.)*base.tau_nom*0


# %%
# Create the simulation object and simulate it. The option `pwm=True` enables
# the model for the carrier comparison.

sim = mt.Simulation(mdl, ctrl, pwm=True)
t_start = time()  # Start the timer
sim.simulate(t_stop=2)
print('\nExecution time: {:.2f} s'.format((time() - t_start)))

# %%
# Plot results in per-unit values.

# sphinx_gallery_thumbnail_number = 2
mt.plot(sim, base=base)
mt.plot_extra(sim, t_span=(0.58, 0.7), base=base)
