"""
Vector-controlled 2.2-kW induction motor drive
==============================================

This example simulates sensorless vector control of a 2.2-kW induction motor
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
# Alternatively configure the induction motor using its Γ parameters
# motor = mt.InductionMotor(R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, p=2)

mech = mt.Mechanics(J=.015)  # Mechanics model
conv = mt.Inverter(u_dc=540)  # Inverter model
mdl = mt.InductionMotorDrive(motor, mech, conv)  # System model

# %%
# Configure the control system. You may also try to change the parameters.
# Notice that the drive may become unstable if you for example have too large
# parameter errors. This may happen in real drives as well.

ctrl = mt.InductionMotorVectorCtrl(
    mt.InductionMotorVectorCtrlPars(
        sensorless=True,  # Enable sensorless mode
        T_s=250e-6,  # Sampling period
        alpha_c=2*np.pi*200,  # Current-control bandwidth
        alpha_o=2*np.pi*40,  # Observer bandwidth
        alpha_s=2*np.pi*4,  # Speed-control bandwidth
        psi_R_nom=.9,  # Nominal rotor flux
        i_s_max=1.5*base.i,  # Current limit
        tau_M_max=1.5*base.tau_nom,  # Torque limit (for the speed ctrl)
        J=.015,  # Inertia estimate (for the speed ctrl)
        p=2,  # Number of pole pairs
        # Inverse-Γ model parameter estimates
        R_s=3.7,
        R_R=2.1,
        L_sgm=.021,
        L_M=.224))

# %%
# Set the speed reference and the external load torque. You may also try to
# uncomment the field-weakening sequence. More complicated sequences could be
# created as functions.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(.5*base.w)
mdl.mech.tau_L_t = lambda t: (t > .75)*base.tau_nom

# No load, field-weakening (uncomment to try)
# mdl.mech.tau_L_t = lambda t: 0
# ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=1.5)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

mt.plot(sim, base=base)
