"""
2.2-kW PMSM, diode bridge
=========================

This example simulates sensorless current-vector control of a 2.2-kW PMSM drive,
equipped with a diode bridge rectifier.

"""

# %%
from math import pi

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.FrequencyConverter(C_dc=235e-6, L_dc=2e-3, U_g=400, f_g=50)
mdl = model.Drive(machine, mechanics, converter, pwm=True)

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.CurrentVectorControllerCfg(i_s_max=1.5 * base.i)
vector_ctrl = control.CurrentVectorController(est_par, cfg, T_s=250e-6)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * nom.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1)
# sphinx_gallery_thumbnail_number = 3
utils.plot(res, base)

# %%
# Plot also the stator voltage and currents as well as the DC-bus and grid-side
# quantities.

utils.plot_extra(res, base, t_span=(0.8, 0.825))
