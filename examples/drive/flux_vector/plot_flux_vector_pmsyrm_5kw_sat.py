"""
5.5-kW PM-SyRM, saturated
=========================

This example simulates sensorless stator-flux-vector control of a 5.5-kW PM-SyRM (Baldor
ECS101M0H7EF4) drive. The machine model is parametrized using the flux map data,
measured using the constant-speed test. The control system is parametrized using the
algebraic saturation model from [#Lel2024]_, fitted to the measured data. This
saturation model can capture the de-saturation phenomenon of thin iron ribs, see
[#Arm2009]_ for details. For comparison, the measured data is plotted together with the
model predictions.

"""
# %%

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=8.8, f=60, P=5.5e3, tau=29.2)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Plot the saturation model (surfaces) and the measured flux map data (points). This
# data is used to parametrize the machine model.

# Load the measured data from the MATLAB file
p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
meas_data = loadmat(p / "ABB_400rpm_map.mat")

# Create the flux map from the measured data
meas_flux_map = utils.MagneticModel(
    i_s_dq=meas_data["id_map"] + 1j * meas_data["iq_map"],
    psi_s_dq=meas_data["psid_map"] + 1j * meas_data["psiq_map"],
    type="flux_map",
)

# Plot the measured data
# sphinx_gallery_thumbnail_number = 4
utils.plot_flux_vs_current(meas_flux_map, base, lims=(-1.5, 1.5))

# %%
# Create a saturation model, which will be used in the control system.

i_s_dq_fcn = utils.SaturationModelPMSyRM(
    a_d0=3.96,
    a_dd=28.5,
    S=4,
    a_q0=1.1 * 5.89,  # Unsaturated q-axis inductance is underestimated for robustness
    a_qq=2.67,
    T=6,
    a_dq=41.5,
    U=1,
    V=1,
    a_b=81.75,
    a_bp=1,
    k_q=0.1,
    psi_n=0.804,
    W=2,
)

# %%
# Compare the saturation model with the measured data.

# Generate the flux map using the saturation model
est_curr_map = i_s_dq_fcn.as_magnetic_model(
    d_range=np.linspace(-0.1, 1.3 * base.psi, 256),
    q_range=np.linspace(-1.7 * base.psi, 1.7 * base.psi, 256),
)
est_flux_map = est_curr_map.invert()

# Plot the saturation model (surface) and the measured data (points)
utils.plot_maps(
    est_flux_map, base, x_lims=(-1.8, 1.8), y_lims=(-2.15, 2.15), raw_data=meas_flux_map
)

# %%
# Configure the system model.

meas_curr_map = meas_flux_map.invert()
par = model.SaturatedSynchronousMachinePars(n_p=2, R_s=0.63, i_s_dq_fcn=meas_curr_map)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.05)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, i_s_dq_fcn=est_curr_map, psi_s_dq_fcn=est_flux_map
)
cfg = control.FluxVectorControllerCfg(i_s_max=2 * base.i, alpha_o=2 * np.pi * 50)
vector_ctrl = control.FluxVectorController(est_par, cfg, sensorless=True)
speed_ctrl = control.SpeedController(J=0.05, alpha_s=2 * np.pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)


# %%
# Visualize the control loci.

i_s_vals = [1, 2, 3]  # Current values for the plots
mc = utils.MachineCharacteristics(est_par)
mc.plot_flux_vs_torque(i_s_vals, base)
mc.plot_current_vs_torque(i_s_vals, base)
mc.plot_current_loci(i_s_vals, base)
mc.plot_flux_loci(i_s_vals, base)
plt.show()

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.8) * 0.7 * nom.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.5)
utils.plot(res, base)

# %%
# .. rubric:: References
#
# .. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a
#    simplified equivalent magnetic circuit for permanent magnet machines," Proc. ICEM,
#    2024, https://doi.org/10.1109/ICEM60801.2024.10700403
#
# .. [#Arm2009] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate modeling
#    and performance analysis of IPM-PMASR motors," IEEE Trans. Ind. Appl., 2009,
#    https://doi.org/10.1109/TIA.2008.2009493
