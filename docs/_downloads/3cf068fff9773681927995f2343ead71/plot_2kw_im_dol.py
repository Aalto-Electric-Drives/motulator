"""
2.2-kW IM, direct-on-line start
===============================

This example simulates the direct-on-line start of a 2.2-kW induction machine (IM),
whose stator is connected directly to a stiff grid. The machine drives a fan, whose load
torque is proportional to the square of the speed.

"""

# %%
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model. The machine is connected to the grid at the start, when
# the phase-a voltage is at its peak.

par = model.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
machine = model.InductionMachine(par)
k = nom.tau / base.w_M**2  # Quadratic load torque profile
mechanics = model.MechanicalSystem(J=0.03, B_L=lambda w_M: k * abs(w_M))
ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
mdl = model.GridConnectedMachine(machine, mechanics, ac_source)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values. The
# currents and voltages are plotted as the magnitudes of their space vectors.

sim = model.Simulation(mdl)
res = sim.simulate(t_stop=0.5)
utils.plot(res, base)

# %%
# Plot the stator voltage and currents during the start. The currents contain decaying
# DC components, which depend on the instant of the connection.

utils.plot_stator_waveforms(res, base, t_lims=(0, 0.1))
