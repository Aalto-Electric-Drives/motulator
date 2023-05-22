"""
2.2-kW induction motor, saturated
=================================

This example simulates sensorless vector control of a 2.2-kW induction motor
drive. The magnetic saturation of the machine is also included in the system 
model, while the control system assumes constant parameters. 

"""

# %%
# Imports.

from motulator import model, control
from motulator import BaseValues, plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# Configure the system model.

# %%
# The main-flux saturation is modeled based on [#Qu2012]_. For simplicity, the
# parameters are hardcoded in the function below, but this model structure can
# be used also for other induction machines.


def L_s(psi):
    """
    Stator inductance saturation model for a 2.2-kW machine.

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
# machine = model.im.InductionMachineInvGamma(
#    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
# Alternatively, configure the machine model using its Γ parameters
# machine = model.im.InductionMachine(
#     R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.im.Drive(machine, mechanics, converter)

# %%
# Configure the control system.
# Create the control system

# Machine model parameters
par = control.im.ModelPars(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
# Set nominal values and limits for reference generation
ref = control.im.CurrentReferencePars(
    par, i_s_max=1.5*base.i, u_s_nom=base.u, w_s_nom=base.w)
# Create the control system
ctrl = control.im.VectorCtrl(par, ref, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque. You may also try to
# uncomment the field-weakening sequence.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(.5*base.w)
mdl.mechanics.tau_L_t = lambda t: (t > .75)*base.tau_nom

# No load, field-weakening (uncomment to try)
# ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)
# mdl.mechanics.tau_L_t = lambda t: 0

# Speed reversals under the rated load (uncomment to try, change t_stop=8 below)
# import numpy as np
# from motulator.helpers import Sequence
# times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*8
# values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
# ctrl.w_m_ref = Sequence(times, values)
# # External load torque
# times = np.array([0, .125, .125, .875, .875, 1])*8
# values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
# mdl.mechanics.tau_L_t = Sequence(times, values)

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower).

sim = model.Simulation(mdl, ctrl, pwm=False)
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
