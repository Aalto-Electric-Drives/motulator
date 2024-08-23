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
    BaseValues, FilterPars, GridPars, NominalValues, plot)

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=18, f=50, P=12.5e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(u_gN=base.u, w_gN=base.w, L_g=.74*base.L)
# Uncomment the line below to simulate a strong grid
# grid_par.L_g = 0

# Filter parameters
filter_par = FilterPars(L_fc=.15*base.L, R_fc=.05*base.Z)

# Create AC filter with given parameters
ac_filter = model.ACFilter(filter_par, grid_par)

# Grid voltage source with constant frequency and voltage magnitude
grid_model = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)

# Inverter with constant DC voltage
converter = model.VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, grid_model)

# %%
# Configure the control system.

# Estimates for the grid parameters, grid inductance estimate is left at 0
grid_par_est = GridPars(u_gN=base.u, w_gN=base.w, L_g=.2*base.L)

# Set the configuration parameters
cfg = control.ObserverBasedGFMControlCfg(
    grid_par_est, filter_par, max_i=1.3*base.i, T_s=100e-6, R_a=.2*base.Z)

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
# mdl.grid_model.par.abs_e_g = lambda t: (1 - (t > .2)*.8 + (t > 1)*.8)*base.u
# ctrl.ref.p_g = lambda t: nom.P

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot the results.

plot(sim, base)
