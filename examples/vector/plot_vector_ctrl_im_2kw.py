"""
2.2-kW induction motor, saturated
=================================

This example simulates sensorless current-vector control of a 2.2-kW induction 
motor drive. The magnetic saturation of the machine is also included in the 
system model, while the control system assumes constant parameters. 

"""

# %%

from motulator import model, control
from motulator import base_values, plot, NominalValues

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = base_values(nom, n_p=2)

# %%
# Configure the system model.

# %%
# The main-flux saturation is modeled based on [#Qu2012]_. For simplicity, the
# parameters are hardcoded in the function below, but this model structure can
# be used also for other induction machines.


def L_s(psi):
    """
    Stator inductance saturation model for a 2.2-kW induction machine.

    Parameters
    ----------
    psi : float
        Magnitude of the stator flux linkage (Vs).
    
    Returns
    -------
    float
        Stator inductance (H).

    """
    # Saturation model parameters for a 2.2-kW induction machine, based on the
    # measured data
    L_su, beta, S = .34, .84, 7
    # Stator inductance
    return L_su/(1 + (beta*psi)**S)


# %%
# Create the system model.

# Γ-equivalent machine model with main-flux saturation included
machine = model.im.InductionMachineSaturated(
    n_p=2, R_s=3.7, R_r=2.5, L_ell=.023, L_s=L_s)
# Unsaturated machine model, using its inverse-Γ parameters (uncomment to try)
#machine = model.im.InductionMachineInvGamma(
#    n_p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224)
# Alternatively, configure the machine model using its Γ parameters
# machine = model.im.InductionMachine(
#     n_p=2, R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.im.Drive(machine, mechanics, converter)
# mdl.pwm = model.CarrierComparison()  # Try to enable the PWM model
# mdl.delay = model.Delay(2)  # Try longer computational delay

# %%
# Configure the control system.

# Machine model parameters
par = control.im.ModelPars(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
# Set nominal values and limits for reference generation
cfg = control.im.CurrentReferenceCfg(
    par, max_i_s=1.5*base.i, nom_u_s=base.u, nom_w_s=base.w)
# Create the control system
ctrl = control.im.CurrentVectorCtrl(par, cfg, T_s=250e-6, sensorless=True)
# As an example, you may replace the default 2DOF PI speed controller with the
# regular PI speed controller by uncommenting the following line
# ctrl.speed_ctrl = control.PICtrl(k_p=1, k_i=1)

# %%
# Set the speed reference and the external load torque. You may also try to
# uncomment the field-weakening sequence.

# Simple acceleration and load torque step
ctrl.ref.w_m = lambda t: (t > .2)*(.5*base.w)
mdl.mechanics.tau_L_t = lambda t: (t > .75)*nom.tau

# No load, field-weakening (uncomment to try)
# ctrl.ref.w_m = lambda t: (t > .2)*(2*base.w)
# mdl.mechanics.tau_L_t = lambda t: 0

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)

# %%
# .. rubric:: References
#
# .. [#Qu2012] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control
#    of induction motor drives," IEEE Trans. Ind. Appl., 2012,
#    https://doi.org/10.1109/TIA.2012.2190818
