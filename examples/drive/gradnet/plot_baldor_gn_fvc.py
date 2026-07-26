"""
5.6-kW PM-SyRM, GradNet-based saturation model, FVC
===================================================

This example simulates sensorless flux-vector control (FVC) of
a 5.6-kW permanent-magnet synchronous reluctance machine
(PM-SyRM, Baldor ECS101M0H7EF4) drive. GradNet-based saturation models,
trained on the FEM and measured datasets, are used. Due to partly
unknown geometry and material properties, the FEM and measured datasets differ in
magnitude, but their shapes are similar.

"""

# %%
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils
from motulator.drive.utils import gn

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Determine the path of the current script.

p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()

# %%
# Configure the system model using the GradNet saturation model (with or without spatial
# harmonics).

spatial_harmonics = True
if spatial_harmonics:
    # GradNet with spatial harmonics, trained on the FEM data
    path = "trained_models/baldor_fem_current_map_harm_squareplus_d48_sub10_.pth"
    gradnet = gn.load_gradnet(p / path, activation=gn.Squareplus)
    magnetic_map = gn.CurrentMapWithHarmonics(gradnet)
    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, magnetic_map_fcn=magnetic_map
    )
else:
    # GradNet without spatial harmonics, trained on the measured data
    path = "trained_models/baldor_meas_current_map_squareplus_d12_sub10_.pth"
    gradnet = gn.load_gradnet(p / path, activation=gn.Squareplus)
    current_map = gn.CurrentMap(gradnet)
    par = model.SaturatedSynchronousMachinePars(n_p=2, R_s=0.63, i_s_dq_fcn=current_map)

machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.05)
# converter = model.VoltageSourceConverter(u_dc=540)
converter = model.VoltageSourceConverter(u_dc=540 * 0.95)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

# Load the flux map for the estimated machine model
if spatial_harmonics:
    # GradNet with spatial harmonics, trained on the FEM data
    path = "trained_models/baldor_fem_flux_map_pnorm_d12_sub20_.pth"
    est_flux_map = gn.FluxMap(gn.load_gradnet(p / path, activation=gn.PNormGradient))
else:
    # GradNet without spatial harmonics, trained on the measured data
    path = "trained_models/baldor_meas_flux_map_pnorm_d6_sub10_.pth"
    est_flux_map = gn.FluxMap(gn.load_gradnet(p / path, activation=gn.PNormGradient))

# Parametrize the estimated machine model and the control system
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
)
cfg = control.FluxVectorControllerCfg(
    i_s_max=2 * base.i,
    alpha_i=0,
    alpha_o=2 * np.pi * 8,
    J=0.05,
    k_mtpv=0.7,
    psi_s_max=2 * base.psi,
    sensorless=False,
    T_s=1 / 12000,
)
vector_ctrl = control.FluxVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(
    J=0.05, alpha_s=2 * np.pi * 4, tau_M_max=4 * nom.tau
)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)


# %%
# Visualize the control loci.

if True:
    i_s_vals = [1, 2, 3]  # Current values for the plots
    mc = utils.MachineCharacteristics(est_par)
    mc.plot_flux_vs_torque(
        i_s_vals, base, num=50, latex=False, save_path=p / "figs" / "flux_vs_torque.pdf"
    )
    # mc.plot_current_vs_torque(i_s_vals, base)
    # mc.plot_current_loci(i_s_vals, base)
    # mc.plot_flux_loci(i_s_vals, base)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.25) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 1.25) * 0.5 * base.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)

time0 = time.time()
res = sim.simulate(t_stop=1.75)
time1 = time.time()
print(f"Simulation time: {time1 - time0:.2f} s")


# %%
# Plot figures

subplots = ["speed", "torque", "current", "flux"]
drop = {"speed": [2], "torque": [], "current": [], "flux": [1]}
colors = {  # order
    "torque": ["b", "gray", "r", "m"],  # τ_ref, τ_m, τ̂_m, τ_L
    "current": ["b", "r"],  # i_d, i_q
    "flux": ["b", "r"],  # ψ_ref, ψ̂_s
}
legend_loc = {
    "speed": "right",
    "torque": "upper center",
    "current": "upper center",
    "flux": "upper right",
}

_show, plt.show = plt.show, lambda *a, **k: None
utils.plot(
    res,
    base,
    subplots=subplots,
    latex=False,
    y_lims=[(-0.2, 2.2), (-0.2, 2.2), (-2.2, 2), (0, 1.25)],
    y_ticks=[
        [0, 0.5, 1.0, 1.5, 2.0],
        [0, 0.5, 1.0, 1.5, 2.0],
        [-2, -1, 0, 1, 2],
        [0, 0.25, 0.5, 0.75, 1.0, 1.25],
    ],
)
plt.show = _show
fig = plt.gcf()
w, h = plt.rcParams["figure.figsize"]
fig.set_size_inches(w, h * 3 * 4 / 5)
# adjust details
for ax, name in zip(fig.axes, subplots, strict=False):
    for idx in sorted(drop.get(name, []), reverse=True):
        ax.lines[idx].remove()
    for line, col in zip(ax.lines, colors.get(name, []), strict=False):
        line.set_color(col)
    if name == "torque":
        ax.lines[1].set(linewidth=0.5, zorder=1, alpha=0.5)
        ax.lines[3].set(linestyle="--")
        ax.lines[3].set_label(r"${\tau}_\mathrm{L}$")
        ax.legend(handles=[ax.lines[0], ax.lines[2], ax.lines[3], ax.lines[1]])
    elif ax.get_legend():
        ax.legend(loc=legend_loc[name])  # type: ignore

save_path = Path(p / "figs" / "sim.pdf")
save_path.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(save_path, bbox_inches="tight")
plt.show()
