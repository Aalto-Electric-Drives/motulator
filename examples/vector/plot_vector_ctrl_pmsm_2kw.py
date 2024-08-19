"""
2.2-kW PMSM
===========

This example simulates sensorless current-vector control of a 2.2-kW PMSM 
drive. 

"""
# %%
from motulator.common.model import Simulation, VoltageSourceConverter
from motulator.common.utils import BaseValues, NominalValues
from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import plot, SynchronousMachinePars

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

mdl_par = SynchronousMachinePars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=.015)
converter = VoltageSourceConverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

par = mdl_par  # Assume accurate machine model parameter estimates
cfg = control.CurrentReferenceCfg(par, nom_w_m=base.w, max_i_s=1.5*base.i)
ctrl = control.CurrentVectorControl(
    par, cfg, J=.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference in mechanical rad/s
ctrl.ref.w_m = lambda t: (t > .2)*2*base.w

# External load torque
mdl.mechanics.tau_L = lambda t: (t > .8)*.7*nom.tau

# %%
# Create the simulation object and simulate it.

# Simulate the system and plot results in per-unit values
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=1.4)

plot(sim, base)
