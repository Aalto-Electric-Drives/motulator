"""
5.6-kW PM-SyRM, plot maps
=========================

This example demonstrates how to plot figures using the LaTeX option and save them as
PDF files. The flux linkage and current maps of a 5.6-kW PM-SyRM (Baldor ECS101M0H7EF4)
are plotted, using both the measured data from the constant-speed test [#Arm2013]_ and
fitted analytical model [#Lel2024]_. This saturation model can capture the de-saturation
phenomenon of thin iron ribs, see [#Arm2009]_ for details. Most other plotting scripts
have similar functionalities.

"""
# %%

from pathlib import Path

import numpy as np
from scipy.io import loadmat

from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Plot the saturation model (surfaces) and the measured flux map data (points). The
# variable `p` determines the path of the folder where the script is located.

# Load the measured data from the MATLAB file
p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
meas_data = loadmat(p / "ABB_400rpm_map.mat")

# Create the flux map from the measured data
meas_flux_map = utils.MagneticModel(
    i_s_dq=meas_data["id_map"] + 1j * meas_data["iq_map"],
    psi_s_dq=meas_data["psid_map"] + 1j * meas_data["psiq_map"],
    type="flux_map",
)

# %%
# Create a saturation model.

i_s_dq_fcn = utils.SaturationModelPMSyRM(
    a_d0=3.96,
    a_dd=28.5,
    S=4,
    a_q0=1.1 * 5.89,
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
# Compare the saturation model with the measured data. To save the PDF figures,
# uncomment the `save_path` argument in the plotting functions.

# Generate the current and flux maps using the saturation model
est_curr_map = i_s_dq_fcn.as_magnetic_model(
    d_range=np.linspace(-0.1 * base.psi, 1 * base.psi, 256),
    q_range=np.linspace(-1.4 * base.psi, 1.4 * base.psi, 256),
)
est_flux_map = est_curr_map.invert()


# Plot the saturation model (surface) and the measured data (points)
est_flux_map_for_plot = est_curr_map.invert(
    d_range=np.linspace(-24, 24, 25), q_range=np.linspace(-30, 30, 31)
)
utils.plot_map(
    est_flux_map_for_plot,
    "d",
    base,
    lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (0, 1)},
    ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2]},
    raw_data=meas_flux_map,
    axlim_clip=False,
    latex=True,
    pad_inches=0.5,  # Add more space to avoid clipping the z axis
    # save_path=p / "flux_map_d.pdf",
)
utils.plot_map(
    est_flux_map_for_plot,
    "q",
    base,
    lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (-1.5, 1.5)},
    ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2]},
    raw_data=meas_flux_map,
    axlim_clip=False,
    latex=True,
    pad_inches=0.5,  # Add more space to avoid clipping the z axis
    # save_path=p / "flux_map_q.pdf",
)

est_curr_map_for_plot = i_s_dq_fcn.as_magnetic_model(
    d_range=np.linspace(0.0 * base.psi, 0.9 * base.psi, 24),
    q_range=np.linspace(-1.3 * base.psi, 1.3 * base.psi, 24),
)

utils.plot_map(
    est_curr_map_for_plot,
    "d",
    base,
    lims={"x": (0, 0.9), "y": (-1.5, 1.5), "z": (-2.5, 3.5)},
    ticks={"x": [0, 0.3, 0.6, 0.9], "y": [-1.5, 0, 1.5], "z": [-2, -1, 0, 1, 2, 3]},
    raw_data=meas_flux_map,
    axlim_clip=False,
    latex=True,
    pad_inches=0.5,  # Add more space to avoid clipping the z axis
    # save_path=p / "curr_map_d.pdf",
)

utils.plot_map(
    est_curr_map_for_plot,
    "q",
    base,
    lims={"x": (0, 0.9), "y": (-1.5, 1.5), "z": (-3.5, 3.5)},
    ticks={"x": [0, 0.3, 0.6, 0.9], "y": [-1.5, 0, 1.5], "z": [-3, -2, -1, 0, 1, 2, 3]},
    raw_data=meas_flux_map,
    axlim_clip=False,
    latex=True,
    pad_inches=0.5,  # Add more space to avoid clipping the z axis
    # save_path=p / "curr_map_q.pdf",
)

# %%
# Visualize the control loci. The `save_path` argument could be similarly used.

par = model.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, i_s_dq_fcn=est_curr_map, psi_s_dq_fcn=est_flux_map
)

i_s_vals = [1, 2, 3]  # Current values for the plots
mc = utils.MachineCharacteristics(par)
mc.plot_flux_vs_torque(i_s_vals, base, latex=True)
mc.plot_current_vs_torque(i_s_vals, base, latex=True)
mc.plot_current_loci(i_s_vals, base, latex=True)
mc.plot_flux_loci(i_s_vals, base, latex=True)

# %%
# .. rubric:: References
#
# .. [#Arm2013] Armando, Bojoi, Guglielmi, Pellegrino, Pastorelli, "Experimental
#    identification of the magnetic model of synchronous machines," IEEE Trans. Ind.
#    Appl., 2013, https://doi.org/10.1109/TIA.2013.2258876
#
# .. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a
#    simplified equivalent magnetic circuit for permanent magnet machines," Proc. ICEM,
#    2024, https://doi.org/10.1109/ICEM60801.2024.10700403
#
# .. [#Arm2009] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate modeling
#    and performance analysis of IPM-PMASR motors," IEEE Trans. Ind. Appl., 2009,
#    https://doi.org/10.1109/TIA.2008.2009493
