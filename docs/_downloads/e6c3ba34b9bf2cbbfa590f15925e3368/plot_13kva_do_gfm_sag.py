"""
12.5-kVA, DO-GFM, grid-voltage sag
==================================

This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM)
converter during a grid-voltage sag in a weak grid. The grid voltage drops to 0.5 p.u.
while the active-power reference is kept at 1 p.u. The active-power reference is limited
to a realizable level, prioritizing the reactive current [#Maa2026]_. Without this
limitation (i.e., if `i_d_max` is not given), the converter loses synchronism during the
sag.

"""

# %%
from motulator.grid import control, model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# Configure the system model. The grid voltage drops to 0.5 p.u. at t = 0.4 s and
# recovers at t = 1 s.

ac_filter = model.LFilter(L_f=0.15 * base.L, R_f=0.05 * base.Z, L_g=0.74 * base.L)
ac_source = model.ThreePhaseSource(
    w_g=base.w, e_g=lambda t: (1 - (t > 0.4) * 0.5 + (t > 1) * 0.5) * base.u
)
converter = model.VoltageSourceConverter(u_dc=650)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system, including the active-power reference limitation.

cfg = control.ObserverBasedGridFormingControllerCfg(
    i_max=1.3 * base.i,
    L=0.35 * base.L,
    R=0.05 * base.Z,
    R_a=0.2 * base.Z,
    u_nom=base.u,
    w_nom=base.w,
    i_d_max=1.1 * base.i,
)
inner_ctrl = control.ObserverBasedGridFormingController(cfg)
ctrl = control.GridConverterControlSystem(inner_ctrl)

# %%
# Set the references for converter output voltage magnitude and active power.

ctrl.set_ac_voltage_ref(base.u)
ctrl.set_power_ref(lambda t: (t > 0.1) * nom.P)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.4)
utils.plot_control_signals(res, base)
utils.plot_grid_waveforms(res, base)

# %%
# .. rubric:: References
#
# .. [#Maa2026] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors,
#    "Disturbance-observer-based grid-forming control for unbalanced grids," 2026,
#    https://arxiv.org/abs/2608.11857
