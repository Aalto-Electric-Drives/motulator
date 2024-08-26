"""
12.5-kVA converter, disturbance observer
========================================
    
This example simulates a converter using disturbance-observer-based control in
grid-forming mode. The converter output voltage and the active power are 
directly controlled, and grid synchronization is provided by the disturbance 
observer. A transparent current controller is included for current limitation.

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

# Filter and grid parameters
par = ACFilterPars(L_fc=.15*base.L, R_fc=.05*base.Z, L_g=.74*base.L)
# Uncomment the line below to simulate a strong grid
# par.L_g = 0
ac_filter = model.ACFilter(par, e_gs0=base.u)
ac_source = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)
# Inverter with constant DC voltage
converter = model.VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

# Set the configuration parameters
cfg = control.ObserverBasedGFMControlCfg(
    L=.35*base.L,
    R=.05*base.Z,
    nom_u=base.u,
    nom_w=base.w,
    max_i=1.3*base.i,
    T_s=100e-6,
    R_a=.2*base.Z)

# Create the control system
ctrl = control.ObserverBasedGFMControl(cfg)

# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference
ctrl.ref.v_c = lambda t: base.u

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)/3 + (t > .5)/3 + (t > .8)/3 -
                          (t > 1.2))*nom.P

# Uncomment line below to simulate operation in rectifier mode
# ctrl.ref.p_g = lambda t: ((t > .2) - (t > .7)*2 + (t > 1.2))*nom.P

# Uncomment lines below to simulate a grid voltage sag with constant ref.p_g
# mdl.ac_source.par.abs_e_g = lambda t: (1 - (t > .2)*.8 + (t > 1)*.8)*base.u
# ctrl.ref.p_g = lambda t: nom.P

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot the results.

plot(sim, base)
