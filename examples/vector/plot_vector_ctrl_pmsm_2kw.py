"""
2.2-kW PMSM
===========

This example simulates sensorless current-vector control of a 2.2-kW PMSM
drive.

"""

# %%

import time

from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import (
    BaseValues,
    NominalValues,
    plot,
    SynchronousMachinePars,
)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

mdl_par = SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

par = mdl_par  # Assume accurate machine model parameter estimates
cfg = control.CurrentReferenceCfg(par, nom_w_m=base.w, max_i_s=1.5 * base.i)
ctrl = control.CurrentVectorControl(par, cfg, J=0.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference in mechanical rad/s
ctrl.ref.w_m = lambda t: (t > 0.2) * 2 * base.w

# External load torque
mdl.mechanics.tau_L = lambda t: (t > 0.8) * 0.7 * nom.tau

# %%
# Create the simulation object and simulate it.

# Simulate the system and plot results in per-unit values
start_time = time.time()
sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.4)
stop_time = time.time()
print(f"Simulation time: {stop_time-start_time:.2f} s")
plot(sim, base)
