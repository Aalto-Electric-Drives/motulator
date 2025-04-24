"""
2.2-kW induction motor, torque-control mode
===========================================

This example simulates current-vector control of a 2.2-kW induction motor drive in
torque-control mode.

"""

# %%
from math import pi, sin

import motulator.drive.control.im as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

# Parametrize the machine model using its inverse-Γ parameters
par = model.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
machine = model.InductionMachine(par)
mechanics = model.ExternalRotorSpeed()
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.


est_par = control.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
cfg = control.CurrentVectorControllerCfg(psi_s_nom=base.psi, i_s_max=1.5 * base.i)
vector_ctrl = control.CurrentVectorController(est_par, cfg, sensorless=True)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl=None)

# %%
# Set the torque reference and the actual speed.

# Torque reference steps
ctrl.set_torque_ref(lambda t: (t > 0.25) * nom.tau - (t > 1.25) * 2 * nom.tau)
# Actual speed varies sinusoidally
mdl.mechanics.set_external_rotor_speed(lambda t: 0.5 * base.w_M * sin(2 * pi * 1 * t))

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=2)
utils.plot(res, base)
