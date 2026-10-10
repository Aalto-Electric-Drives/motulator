"""
2.2-kW IPMSM, current-sensor errors, FVC
========================================

This example simulates sensorless flux-vector control (FVC) of a 2.2-kW interior
permanent-magnet synchronous machine (IPMSM) drive with nonideal current sensors. The
phase currents are measured with offsets, gain errors, and noise. Apart from the sensors
and the speed reference, the drive is the same as in
:doc:`/drive_examples/flux_vector/plot_2kw_ipmsm_fvc`.

"""

# %%
from math import pi

import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the current sensors. A sensor maps the actual signal `x` to the measured
# signal `gain*x + offset + std*n`, where `n` is white Gaussian noise. Here, the
# parameters are given per phase. The random number generator is seeded to make the
# results reproducible.

i_s_sensor = model.Sensor(
    gain=[1.0, 1.02, 0.99],
    offset=[0.02 * base.i, -0.01 * base.i, 0.0],
    std=0.005 * base.i,
    rng=np.random.default_rng(seed=1),
)

# %%
# Configure the system model. A sensor is a parameter of the subsystem whose signals it
# measures. Similarly, a DC-bus voltage sensor could be given to the converter model as
# `u_dc_sensor`. Without sensors, the measurements are ideal.

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(par, i_s_sensor=i_s_sensor)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.FluxVectorControllerCfg(i_s_max=1.5 * base.i, sensorless=True)
vector_ctrl = control.FluxVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque. The speed is first increased
# to 0.5 p.u. and then, under the load, decreased to 0.1 p.u.

ctrl.set_speed_ref(lambda t: ((t > 0.2) * 0.5 - (t > 1.2) * 0.4) * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.8) * 0.7 * nom.tau)

# %%
# Simulate the drive and plot the results. The control system computes its flux and
# torque estimates from the measured currents, so the current-sensor errors appear in
# the actual torque. In synchronous coordinates, the offsets cause a ripple at the
# stator frequency, and the gain mismatch between the phases causes a ripple at twice
# the stator frequency. Therefore, the ripple frequency follows the speed. An offset or
# a gain error common to all three phases would cause no ripple: the common offset is a
# zero-sequence component, which is removed in the space-vector transformation, and the
# common gain error only scales the currents. Note that the currents in the figure are
# the measured currents in the estimated rotor coordinates, while the torque `tau_m` is
# the actual torque of the machine model.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=2)
utils.plot(res, base, subplots=["speed", "torque", "current"])
