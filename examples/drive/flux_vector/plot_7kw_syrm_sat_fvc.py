"""
6.7-kW saturated SyRM, FVC
==========================

This example simulates sensorless flux-vector control (FVC) of a saturated 6.7-kW
synchronous reluctance machine (SyRM) drive.

"""

# %%
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=15.5, f=105.8, P=6.7e3, tau=20.1)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

curr_map = utils.SaturationModelSyRM(
    a_d0=17.4, a_dd=373, S=5, a_q0=52.1, a_qq=658, T=1, a_dq=1120, U=1, V=0
)
par = model.SaturatedSynchronousMachinePars(n_p=2, R_s=0.54, i_s_dq_fcn=curr_map)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system, including the saturation model.

# Compute a rectilinear flux map
psi_d_range = np.linspace(-1.5 * base.psi, 1.5 * base.psi, 256)
psi_q_range = np.linspace(-0.5 * base.psi, 0.5 * base.psi, 256)
flux_map = curr_map.as_magnetic_model(psi_d_range, psi_q_range).invert()

# Plot the flux maps
utils.plot_map(flux_map, "d", base)
utils.plot_map(flux_map, "q", base)

# Parameter estimates
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.54, psi_s_dq_fcn=flux_map
)

# Configure the controller
cfg = control.FluxVectorControllerCfg(i_s_max=2 * base.i, psi_s_min=0.5 * base.psi)
vector_ctrl = control.FluxVectorController(est_par, cfg, sensorless=True)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * np.pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Plot control characteristics.

# sphinx_gallery_thumbnail_number = 1
i_s_vals = [1, 1.5, 2]  # Current values for the plots
mc = utils.MachineCharacteristics(est_par)
mc.plot_flux_vs_torque(i_s_vals, base)
mc.plot_current_vs_torque(i_s_vals, base)
mc.plot_current_loci(i_s_vals, base)
mc.plot_flux_loci(i_s_vals, base)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 1) * nom.tau * 0.4)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.6)
utils.plot(res, base)
