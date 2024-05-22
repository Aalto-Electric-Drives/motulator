"""
2.2-kW PMSM, diode bridge
=========================

This example simulates sensorless current-vector control of a 2.2-kW PMSM 
drive, equipped with a diode bridge rectifier. 

"""

# %%

from motulator import model, control
from motulator import BaseValues, plot, plot_extra

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

machine = model.sm.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.FrequencyConverter(L=2e-3, C=235e-6, U_g=400, f_g=50)
mdl = model.sm.DriveWithDiodeBridge(machine, mechanics, converter)
mdl.pwm = model.CarrierComparison()  # Enable the PWM model

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, J=.015)
ref = control.sm.CurrentReferencePars(par, w_m_nom=base.w, i_s_max=1.5*base.i)
ctrl = control.sm.CurrentVectorCtrl(par, ref, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
ctrl.ref.w_m = lambda t: (t > .2)*base.w

# External load torque
mdl.mechanics.tau_L_t = lambda t: (t > .6)*base.tau_nom

# %%
# Create the simulation object and simulate it.

# Simulate the system
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1)

# Plot results in per-unit values
plot(sim, base)
plot_extra(sim, base, t_span=(.8, .825))
