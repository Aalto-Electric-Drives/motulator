"""
10-kVA converter, LCL filter
============================

This example simulates a grid-following-controlled converter connected to a strong grid
through an LCL filter. The control system includes a phase-locked loop (PLL) to
synchronize with the grid, a current reference generator, and a PI-type current
controller. The LCL-filter dynamics are not taken into account in the control system.

"""

# %%
from motulator.grid import control, model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=14.5, f=50, P=10e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# Configure the system model.

ac_filter = model.LCLFilter(
    L_fc=0.073 * base.L, L_fg=0.073 * base.L, C_f=0.043 * base.C, u_f0_ab=base.u
)
ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
converter = model.VoltageSourceConverter(u_dc=650)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

inner_ctrl = control.CurrentVectorController(
    i_max=1.5 * base.i, L=0.073 * base.L, T_s=100e-6
)
ctrl = control.GridConverterControlSystem(inner_ctrl)


# %%
# Set external references.

# Set the active and reactive power references
ctrl.set_power_ref(lambda t: (t > 0.02) * 5e3)
ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=0.08)

# %%
# Plot the results.

utils.plot_control_signals(res, base)
utils.plot_grid_waveforms(res, base)
