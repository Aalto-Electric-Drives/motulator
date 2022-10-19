"""
Observer-based V/Hz control of 2.2-kW induction motor drive
==============================================

This example simulates observer-based V/Hz control of a 2.2-kW induction motor
drive.

"""

# %%
# Import the packages.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=400,  # Line-line rms voltage
    I_nom=5,  # Rms current
    f_nom=50,  # Frequency
    tau_nom=14.6,  # Torque
    P_nom=2.2e3,  # Power
    p=2)  # Number of pole pairs

# %%
# Configure the system model.

# Configure the induction motor using its inverse-Γ parameters
motor = mt.InductionMotorInvGamma(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, p=2)

mech = mt.Mechanics(J=.015)  # Mechanics model
conv = mt.Inverter(u_dc=540)  # Inverter model
mdl = mt.InductionMotorDrive(motor, mech, conv)  # System model

# %%
# Configure the control system.
ctrl = mt.InductionMotorVHzObsCtrl(
    mt.InductionMotorObsVHzCtrlPars(slip_compensation=False))

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = mt.Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mech.tau_L_ext = mt.Sequence(times, values)
# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

mt.plot(sim, base=base)
