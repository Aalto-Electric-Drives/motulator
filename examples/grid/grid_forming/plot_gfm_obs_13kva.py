"""
12.5-kVA converter, disturbance observer
========================================
    
This example simulates a converter using disturbance-observer-based control in
grid-forming mode. The converter output voltage and the active power are 
directly controlled, and grid synchronization is provided by the disturbance 
observer. A transparent current controller is included for current limitation.

"""

# %%
from motulator.common.model import VoltageSourceConverter, Simulation
from motulator.common.utils import BaseValues, NominalValues
from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import FilterPars, GridPars, plot_grid
# from motulator.common.model import CarrierComparison

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
grid_model = model.ThreePhaseVoltageSource(
    w_g=grid_par.w_gN, abs_e_g=grid_par.u_gN)

# Inverter with constant DC voltage
converter = VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, grid_model)

# Uncomment the lines below to enable the PWM model
# mdl.pwm = CarrierComparison()

# %%
# Configure the control system.

# Estimates for the grid parameters, grid inductance estimate is left at 0
grid_par_est = GridPars(u_gN=base.u, w_gN=base.w, L_g=.2*base.L)

# Set the configuration parameters
cfg = control.ObserverBasedGFMControlCfg(
    grid_par=grid_par_est,
    filter_par=filter_par,
    T_s=100e-6,
    max_i=1.3*base.i,
    R_a=.2*base.Z)

# Create the control system
ctrl = control.ObserverBasedGFMControl(cfg)

# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference
ctrl.ref.v_c = lambda t: grid_par.u_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(4.15e3) + (t > .5)*(4.15e3) + (t > .8)*
                          (4.2e3) - (t > 1.2)*(12.5e3))

# Uncomment line below to simulate operation in rectifier mode
# ctrl.ref.p_g = lambda t: ((t > .2) - (t > .7)*2 + (t > 1.2))*12.5e3

# Uncomment lines below to simulate a grid voltage sag with constant ref.p_g
# mdl.grid_model.par.e_g_abs = lambda t: (
#    1 - (t > .2)*(0.8) + (t > 1)*(0.8))*grid_par.u_gN
# ctrl.ref.p_g = lambda t: 12.5e3

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot_grid(sim=sim, base=base, plot_pcc_voltage=False)
