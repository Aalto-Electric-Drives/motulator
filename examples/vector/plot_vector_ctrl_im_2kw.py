"""
Vector control: 2.2-kW induction motor
======================================

This example simulates sensorless vector control of a 2.2-kW induction motor
drive.

"""

# %%
# Import the packages.

import motulator.model.im as model
import motulator.control.im as control
from motulator.helpers import BaseValues
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
# Alternatively configure the induction motor using its Γ parameters
# mdl.machine = model.InductionMachine(
#     R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, n_p=2)

# %%
# Configure the control system.
# Create the control system

# Machine model parameters
par = control.ModelPars(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
# Set nominal values and limits for reference generation
ref = control.CurrentReferencePars(
    par, i_s_max=1.5*base.i, u_s_nom=base.u, w_s_nom=base.w)
# Create the control system
ctrl = control.VectorCtrl(par, ref, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque. You may also try to
# uncomment the field-weakening sequence.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(.5*base.w)
mdl.mechanics.tau_L_t = lambda t: (t > .75)*base.tau_nom

# No load, field-weakening (uncomment to try)
ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)
mdl.mechanics.tau_L_t = lambda t: 0

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = model.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=1.5)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
