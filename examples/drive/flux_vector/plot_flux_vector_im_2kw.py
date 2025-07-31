"""
2.2-kW induction motor, saturated
=================================

This example simulates sensorless flux-vector control of a 2.2-kW induction machine.
The magnetic saturation of the machine is included in the system model as well as taken
into account in the control system. This example also applies the mechanical-model-based
speed observer.

"""

# %%
from math import pi

import motulator.drive.control.im as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = utils.BaseValues.from_nominal(nom, n_p=2)


# %%
# Configure the system model. The Γ-equivalent machine model with main-flux saturation
# is used. The parameters are based on the measured data of a 2.2-kW machine [#Qu2012]_.

par = model.InductionMachinePars(
    n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_s=lambda psi: 0.34 / (1 + (0.84 * psi) ** 7)
)
machine = model.InductionMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = par  # Assume the machine model is perfectly known
# Since the inertia estimate `J` is provided below, the mechanical-model-based speed
# observer is used. You can disable the mechanical-model-based speed observer by
# removing the `J` parameter or setting it to `None`.
cfg = control.FluxVectorControllerCfg(
    psi_s_nom=0.95 * base.psi,
    i_s_max=1.5 * base.i,
    J=0.015,  # Inertia estimate enables the speed observer
    alpha_i=0,  # Integral action is not necessary with the speed observer
)
vector_ctrl = control.FluxVectorController(est_par, cfg, sensorless=True)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)


# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * 2 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.8) * 0.5 * nom.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.5)
utils.plot(res, base)

# %%
# .. rubric:: References
#
# .. [#Qu2012] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
#    induction motor drives," IEEE Trans. Ind. Appl., 2012,
#    https://doi.org/10.1109/TIA.2012.2190818
