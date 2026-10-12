"""
12.5-kVA, DO-GFM, converter output admittance identification
============================================================

This example demonstrates converter output admittance identification using a 12.5-kVA
disturbance-observer-based grid-forming (DO-GFM) converter. The converter and grid
models are the same as in :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm`.

"""

# %%
import numpy as np

from motulator.grid import control, model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# Configure the identification.

identification_cfg = utils.IdentificationCfg(
    abs_u_e=0.01 * base.u, freqs=np.geomspace(1, 10e3, 100)
)

# %%
# Configure the system model.

ac_filter = model.LFilter(L_f=0.15 * base.L, R_f=0.05 * base.Z, L_g=0.74 * base.L)
ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
converter = model.VoltageSourceConverter(u_dc=650)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

cfg = control.ObserverBasedGridFormingControllerCfg(
    i_max=1.3 * base.i,
    L=0.35 * base.L,
    R=0.05 * base.Z,
    R_a=0.2 * base.Z,
    u_nom=base.u,
    w_nom=base.w,
    T_s=1 / 10e3,
)
inner_ctrl = control.ObserverBasedGridFormingController(cfg)
ctrl = control.GridConverterControlSystem(inner_ctrl)

ctrl.set_power_ref(0.5 * base.p)
ctrl.set_ac_voltage_ref(base.u)

# %%
# Run the identification and plot results.

if __name__ == "__main__":
    res = utils.run_identification(identification_cfg, mdl, ctrl)
    # To save the results, e.g., in MATLAB format:
    # from dataclasses import asdict
    # from scipy.io import savemat
    # savemat("do-gfm_admittance.mat", asdict(res))
    utils.plot_identification(res, plot_style="re_im")
    utils.plot_vector_identification(res, base)
