"""
5.6-kW saturated PM-SyRM, CVC
=============================

This example simulates sensorless current-vector control (CVC) of the same 5.6-kW
permanent-magnet synchronous reluctance machine (PM-SyRM) as the corresponding flux-
vector control example. See :ref:`plot_6kw_pmsyrm_sat_fvc` for the machine model, flux-
map data, and saturation-model description.

This example demonstrates online reference generation in current-vector control, which
allows the optimal current references to be generated in real time [#Sar2026]_.

"""
# %%

from pathlib import Path

import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

# Load the measured flux map data used to parametrize the machine model
p = Path(utils.__file__).resolve().parents[3] / "examples/drive/data"
meas_data = np.load(p / "baldor_400rpm_map.npz")
i_s_dq_map = meas_data["i_s_dq"]
psi_s_dq_map = meas_data["psi_s_dq"]

# Create the flux map from the measured data and invert it to get the current map
meas_flux_map = utils.MagneticModel(
    i_s_dq=i_s_dq_map, psi_s_dq=psi_s_dq_map, type="flux_map"
)
meas_curr_map = meas_flux_map.invert()

# Create the drive model
par = model.SaturatedSynchronousMachinePars(n_p=2, R_s=0.63, i_s_dq_fcn=meas_curr_map)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.05)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

# Create a saturation model for the controller
est_current_map = utils.SaturationModelPMSyRM(
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

# Generate the estimated flux map
est_current_map = est_current_map.as_magnetic_model(
    d_range=np.linspace(-0.1 * base.psi, base.psi, 256),
    q_range=np.linspace(-1.4 * base.psi, 1.4 * base.psi, 256),
)
est_flux_map = est_current_map.invert()
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, psi_s_dq_fcn=est_flux_map
)
cfg = control.CurrentVectorControllerCfg(
    i_s_max=2 * base.i, J=0.05, sensorless=True, online=True
)
vector_ctrl = control.CurrentVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.05, alpha_s=2 * np.pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 1.25) * 0.7 * nom.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=2)
utils.plot(res, base)

# %%
# .. rubric:: References
#
# .. [#Sar2026] Sarén, Hartikainen, Piippo, Hinkkanen, "Decoupled online feedforward
#    generation of optimal references for saturated synchronous machine drives,
#    2026, https://arxiv.org/abs/2607.08528
