"""
12.5-kVA converter, RFPSC
=========================

This example simulates reference-feedforward power-synchronization control
(RFPSC) of a converter connected to a weak grid.

"""

# %%
from motulator.grid import model, control
from motulator.grid.utils import (
    BaseValues, ACFilterPars, NominalValues, plot)

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=18, f=50, P=12.5e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Filter and grid
par = ACFilterPars(L_fc=.15*base.L, R_fc=.05*base.Z, L_g=.74*base.L)
# par.L_g = 0  # Uncomment this line to simulate a strong grid
ac_filter = model.ACFilter(par)
# Grid voltage source with constant frequency and voltage magnitude
ac_source = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)
# Inverter with constant DC voltage
converter = model.VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

# Control configuration parameters
cfg = control.PowerSynchronizationControlCfg(
    nom_u=base.u,
    nom_w=base.w,
    max_i=1.3*base.i,
    R=.05*base.Z,
    R_a=.2*base.Z,
    T_s=100e-6)

# Create the control system
ctrl = control.PowerSynchronizationControl(cfg)

# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference
ctrl.ref.v_c = lambda t: base.u

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)/3 + (t > .5)/3 + (t > .8)/3 -
                          (t > 1.2))*nom.P

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot the results.

plot(sim, base)
