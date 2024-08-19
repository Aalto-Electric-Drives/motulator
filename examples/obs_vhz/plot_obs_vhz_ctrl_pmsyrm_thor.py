"""
5-kW PM-SyRM, flux maps from SyR-e
==================================

This example simulates observer-based V/Hz control of a saturated 5-kW
permanent-magnet synchronous reluctance motor. The flux maps of this example
motor, known as THOR, are from the SyR-e project:

    https://github.com/SyR-e/syre_public

The SyR-e project has been licensed under the Apache License, Version 2.0. We
acknowledge the developers of the SyR-e project. The flux maps from other
sources can be used in a similar manner. It is worth noticing that the 
saturation is not taken into account in the control method, only in the system 
model. Naturally, the control performance could be improved by taking the
saturation into account in the control algorithm.

"""
# %%

from os import path
import inspect

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.interpolate import LinearNDInterpolator

from motulator.common.model import Simulation, VoltageSourceConverter
from motulator.common.utils import (BaseValues, NominalValues, Sequence)
from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import plot, SynchronousMachinePars
from motulator.drive.utils import (
    import_syre_data, plot_flux_vs_current, plot_flux_map)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=220, I=15.6, f=85, P=5.07e3, tau=19)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Load and plot the flux maps.

# Get the path of this file
p = path.dirname(path.abspath(inspect.getfile(inspect.currentframe())))
# Load the data from the MATLAB file
data = import_syre_data(p + "/THOR.mat")

# You may also downsample or invert the flux map by uncommenting the following
# lines. Not needed here, but these methods could be useful for other purposes.

# from motulator.model.sm_flux_maps import downsample_flux_map, invert_flux_map
# data = downsample_flux_map(data, N_d=32, N_q=32)
# data = invert_flux_map(data, N_d=128, N_q=128)
plot_flux_vs_current(data)
plot_flux_map(data)

# %%
# Create the saturation model.

# The coordinates assume the PMSM convention, i.e., that the PM flux is along
# the d-axis. The piecewise linear interpolant `LinearNDInterpolator` is based
# on triangularization and allows to use unstructured flux maps.

# Data points for creating the interpolant
psi_s_data = np.asarray(data.psi_s).ravel()
i_s_data = np.asarray(data.i_s).ravel()

# Create the interpolant, i_s = current_dq(psi_s.real, psi_s.imag)
current_dq = LinearNDInterpolator(
    list(zip(psi_s_data.real, psi_s_data.imag)), i_s_data)

# Solve the PM flux for the initial value of the stator flux
res = minimize_scalar(
    lambda psi_d: np.abs(current_dq(psi_d, 0)),
    bounds=(0, np.max(psi_s_data.real)),
    method="bounded")
psi_s0 = complex(res.x)


# Package the input such that i_s = i_s(psi_s)
def i_s(psi_s):
    """Current as a function of the flux linkage."""
    return current_dq(psi_s.real, psi_s.imag)


# %%
# Configure the system model.

# Create the machine model

mdl_par = SynchronousMachinePars(
    n_p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134)
machine = model.SynchronousMachine(mdl_par, i_s=i_s, psi_s0=psi_s0)
# Magnetically linear PM-SyRM model
# mdl_par = SynchronousMachinePars(
#     n_p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134)
# machine = model.SynchronousMachine(mdl_par)
# Quadratic load torque profile (corresponding to pumps and fans)
k = nom.tau/(base.w/base.n_p)**2
mechanics = model.StiffMechanicalSystem(J=.0042, B_L=lambda w_M: k*np.abs(w_M))
converter = VoltageSourceConverter(u_dc=310)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

par = SynchronousMachinePars(n_p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134)
cfg = control.ObserverBasedVHzControlCfg(par, max_i_s=2*base.i)
ctrl = control.ObserverBasedVHzControl(par, cfg, T_s=250e-6)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.ref.w_m = Sequence(times, values)

# External load torque set to zero
mdl.mechanics.tau_L = lambda t: (t > 0)*0

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=8)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
