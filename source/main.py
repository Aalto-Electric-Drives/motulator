"""
This module contains an example main program.

Three different simulation objects are created and simulated.

"""
from time import time
import numpy as np
from simulation import Simulation
from helpers import BaseValues, ref_ramp, ref_step
from model.mech import Mechanics
from model.converter import Inverter, FrequencyConverter  # , PWMInverter
from model.im_drive import (InductionMotorDrive, InductionMotorDriveDiode,
                            InductionMotor, InductionMotorSaturated)
from model.sm_drive import SynchronousMotorDrive, SynchronousMotor
from control.im_vhz import InductionMotorVHzCtrl, InductionMotorVHzCtrlPars
from control.im_vector import (InductionMotorVectorCtrl,
                               InductionMotorVectorCtrlPars)
from control.sm_vector import (SynchronousMotorVectorCtrl,
                               SynchronousMotorVectorCtrlPars)

# %% Create three example simulation objects and simulate them
if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    # %% 2.2-kW induction motor, sensorless vector control

    # Compute base values based on the nominal values (just for figures)
    base = BaseValues(U_nom=400,        # Line-line rms voltage
                      I_nom=5,          # Rms current
                      f_nom=50,         # Frequency
                      tau_nom=14.6,     # Torque
                      P_nom=2.2e3,      # Power
                      p=2)              # Number of pole pairs

    # System model
    mdl = InductionMotorDrive(
        # Gamma-model parameters
        motor=InductionMotor(R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, p=2),
        mech=Mechanics(J=.015),
        conv=Inverter(u_dc0=540))       # conv=PWMInverter(u_dc0=540))

    # Controller
    ctrl = InductionMotorVectorCtrl(InductionMotorVectorCtrlPars(
        sensorless=True,                # Enable sensorless mode
        T_s=250e-6,                     # Sampling period
        delay=1,                        # Amount of computational delay
        alpha_c=2*np.pi*200,            # Current-control bandwidth
        alpha_o=2*np.pi*40,             # Observer bandwidth
        alpha_s=2*np.pi*4,              # Speed-control bandwidth
        psi_R_nom=.9,                   # Nominal rotor flux
        i_s_max=1.5*base.i,             # Current limit
        tau_M_max=1.5*base.tau_nom,     # Torque limit (for the speed ctrl)
        J=.015,                         # Inertia estimate (for the speed ctrl)
        p=2,                            # Number of pole pairs
        # Inverse-Gamma model parameter estimates
        R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224))

    # Speed reference and external load torque
    ref_ramp(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=4)

    # Create the simulation object
    sim1 = Simulation(mdl, ctrl, base=base, print_opts=True)

    # %% 2.2-kW induction motor drive equipped with a diode bridge,
    # main-flux saturation modeled, open-loop V/Hz control

    # Base values
    base = BaseValues(U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6,
                      P_nom=2.2e3, p=2)

    # System model (default values used here)
    mdl = InductionMotorDriveDiode(
        motor=InductionMotorSaturated(),
        mech=Mechanics(),
        conv=FrequencyConverter())

    # Controller (parametrized to open-loop V/Hz mode)
    ctrl = InductionMotorVHzCtrl(InductionMotorVHzCtrlPars(
        R_s=0, R_R=0, k_u=0, k_w=0))

    # Speed reference and external load torque
    ref_step(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=1.5)

    # Create the simulation object
    sim2 = Simulation(mdl, ctrl, base=base, print_opts=False)

    # %% 2.2-kW PMSM, sensorless vector control

    # Base values
    base = BaseValues(U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14,
                      P_nom=2.2e3, p=3)

    # System model (default values used here)
    mdl = SynchronousMotorDrive(
        motor=SynchronousMotor(),
        mech=Mechanics(),
        conv=Inverter())

    # Controller (default values used here)
    ctrl = SynchronousMotorVectorCtrl(SynchronousMotorVectorCtrlPars())

    # Speed reference and external load torque
    ref_ramp(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=4)

    # Create the simulation object
    sim3 = Simulation(mdl, ctrl, base=base, print_opts=False)

    # %% Simulate (uncomment to simulate them all)
    sim1.simulate()
    # sim2.simulate()
    # sim3.simulate()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))

    # %% Plot results (uncomment to plot them all)
    sim1.plot_figure()
    # sim2.plot_figure(extra=True)
    # sim3.plot_figure()
