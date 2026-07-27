"""
5.6-kW PM-SyRM, GradNet from FEM data, FVC
==========================================

This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance
machine (Baldor ECS101M0H7EF4) drive. GradNet saturation models, trained on the FEM
dataset with spatial harmonics, are used.

"""

# %%

from pathlib import Path

import numpy as np

import motulator.drive.control.sm as control
import motulator.drive.gradnet as gn
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Determine the path of the current script.

p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()

# %%
# Configure the system model using the GradNet saturation model with spatial harmonics,
# trained on the FEM data.

path = "trained_models/baldor_fem_curr_map_harm_softmax_d48_sub10.pth"
gradnet = gn.load_gradnet(p / path, activation=gn.Softmax)
magnetic_map = gn.CurrentMapWithHarmonics(gradnet)
par = model.SpatialSaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
)

machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.05)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

# Load the GradNet flux map trained on the FEM data for the estimated machine model
path = "trained_models/baldor_fem_flux_map_pnorm_d12_sub20.pth"
est_flux_map = gn.FluxMap(gn.load_gradnet(p / path, activation=gn.PNormGradient))

# Parametrize the estimated machine model and the control system
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
)
cfg = control.FluxVectorControllerCfg(
    i_s_max=2 * base.i, alpha_i=0, alpha_o=2 * np.pi * 8, J=0.05, sensorless=False
)
vector_ctrl = control.FluxVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.05, alpha_s=2 * np.pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.25) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 1.25) * 0.5 * base.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.75)
utils.plot(res, base)
