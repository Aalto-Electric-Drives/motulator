"""
10-kVA converter, LCL filter
============================
    
This example simulates a grid-following-controlled converter connected to a
strong grid through an LCL filter. The control system includes a phase-locked
loop (PLL) to synchronize with the grid, a current reference generator, and a
PI-type current controller. The dynamics of the LCL filter are not taken into
account in the control system.

"""

# %%
from motulator.grid import model, control
from motulator.grid.utils import (
    BaseValues, ACFilterPars, NominalValues, plot)

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid and filter
par = ACFilterPars(L_fc=.073*base.L, L_fg=.073*base.L, C_f=.043*base.C)
ac_filter = model.ACFilter(par, e_gs0=base.u)
ac_source = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)
# Inverter model with constant DC voltage
converter = model.VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

cfg = control.GFLControlCfg(
    L=.073*base.L, nom_u=base.u, nom_w=base.w, max_i=1.5*base.i)
ctrl = control.GFLControl(cfg)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > .02)*5e3
ctrl.ref.q_g = lambda t: (t > .04)*4e3

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

plot(sim, base)
