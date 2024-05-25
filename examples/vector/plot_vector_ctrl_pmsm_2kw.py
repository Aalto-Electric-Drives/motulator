"""
2.2-kW PMSM
===========

This example simulates sensorless current-vector control of a 2.2-kW PMSM 
drive. 

"""

# %%

import time
from motulator import model, control
from motulator import base_values, plot, NominalValues

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = base_values(nom, n_p=3)

# %%
# Configure the system model.

machine = model.sm.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.sm.Drive(machine, mechanics, converter)
# mdl.pwm = model.CarrierComparison()  # Enable the PWM model

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, J=.015)
cfg = control.sm.CurrentReferenceCfg(par, nom_w_m=base.w, max_i_s=1.5*base.i)
ctrl = control.sm.CurrentVectorCtrl(par, cfg, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference in mechanical rad/s
ctrl.ref.w_m = lambda t: (t > .2)*2*base.w

# External load torque
mdl.mechanics.tau_L_t = lambda t: (t > .8)*.7*nom.tau

# %%
# Create the simulation object and simulate it.

# Simulate the system and plot results in per-unit values
start_time = time.time()
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.4)
stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")
plot(sim, base)
