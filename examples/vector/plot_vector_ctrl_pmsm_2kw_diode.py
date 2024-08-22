"""
2.2-kW PMSM, diode bridge
=========================

This example simulates sensorless current-vector control of a 2.2-kW PMSM
drive, equipped with a diode bridge rectifier.

"""

# %%
from motulator.common.model import CarrierComparison, FrequencyConverter, Simulation
from motulator.common.utils import BaseValues, NominalValues
from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import plot, plot_extra, SynchronousMachinePars

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

# Machine model and mechanical subsystem
mdl_par = SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=0.015)

# Frequency converter with a diode bridge
converter = FrequencyConverter(C_dc=235e-6, L_dc=2e-3, U_g=400, f_g=50)
mdl = model.Drive(converter, machine, mechanics)

mdl.pwm = CarrierComparison()  # Enable the PWM model

# %%
# Configure the control system.

par = mdl_par  # Assume accurate machine model parameter estimates
ref = control.CurrentReferenceCfg(par, nom_w_m=base.w, max_i_s=1.5 * base.i)
ctrl = control.CurrentVectorControl(par, ref, J=0.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
ctrl.ref.w_m = lambda t: (t > 0.2) * base.w

# External load torque
mdl.mechanics.tau_L = lambda t: (t > 0.6) * nom.tau

# %%
# Create the simulation object and simulate it.

# Simulate the system
sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=1)

# Plot results in per-unit values
plot(sim, base)
plot_extra(sim, base, t_span=(0.8, 0.825))
