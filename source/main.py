"""
This module contains the main program.

"""
from time import time
import numpy as np
from simulation import Simulation
from helpers import BaseValues, ref_ramp, ref_step
from model.mech import Mechanics
from model.converter import Inverter, PWMInverter, FrequencyConverter
from model.im_drive import (InductionMotorDrive, InductionMotorDriveDiode,
                            InductionMotor, InductionMotorSaturated)
from model.sm_drive import SynchronousMotorDrive, SynchronousMotor
from control.im_vhz import InductionMotorVHzCtrl, InductionMotorVHzCtrlPars
from control.im_vector import (InductionMotorVectorCtrl,
                               InductionMotorVectorCtrlPars)
from control.sm_vector import (SynchronousMotorVectorCtrl,
                               SynchronousMotorVectorCtrlPars)

# %%
if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    # Create example simulation objects

    # %% 2.2-kW induction motor, saturated, sensorless vector control
    # Base values
    base = BaseValues(U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6,
                      P_nom=2.2e3, p=2)
    # System model
    mdl = InductionMotorDrive(
        motor=InductionMotorSaturated(R_s=3.7, R_r=2.5, L_ell=.023, p=2,
                                      L_su=.34, beta=.84, S=7),
        mech=Mechanics(J=.015),
        conv=PWMInverter(u_dc0=540))
    # Controller
    ctrl = InductionMotorVectorCtrl(InductionMotorVectorCtrlPars(
        sensorless=True,
        T_s=250e-6, delay=1,
        alpha_c=2*np.pi*200, alpha_o=2*np.pi*40, alpha_s=2*np.pi*4,
        tau_M_max=1.5*base.tau_nom, i_s_max=1.5*base.i,
        psi_R_nom=.9,
        R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, p=2, J=.015))
    # Speed reference and external load torque
    ref_ramp(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=4)
    # Create the simulation object
    sim1 = Simulation(mdl, ctrl, base=base, print_opts=True)

    # %% 2.2-kW induction motor, open-loop V/Hz control
    # Base values
    base = BaseValues(U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6,
                      P_nom=2.2e3, p=2)
    # System model
    mdl = InductionMotorDriveDiode(
        motor=InductionMotor(R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, p=2),
        mech=Mechanics(J=.015),
        conv=FrequencyConverter(C=235e-6, L=2e-3, U_g=400, f_g=50))
    # Controller
    ctrl = InductionMotorVHzCtrl(InductionMotorVHzCtrlPars(
        T_s=250e-6, delay=1,
        psi_s_nom=base.psi,
        rate_limit=2*np.pi*120,
        R_s=0*3.7, R_R=0*2.1, L_sgm=.021, L_M=.224,
        k_u=0*1, k_w=0*4))
    # Speed reference and external load torque
    ref_step(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=1.5)
    # Create the simulation object
    sim2 = Simulation(mdl, ctrl, base=base, print_opts=True)

    # %% 2.2-kW PMSM, sensorless vector control
    # Base values
    base = BaseValues(U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14,
                      P_nom=2.2e3, p=3)
    # System model
    mdl = SynchronousMotorDrive(
        motor=SynchronousMotor(R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, p=3),
        mech=Mechanics(J=.015),
        conv=Inverter(u_dc0=540))
    # Controller
    ctrl = SynchronousMotorVectorCtrl(SynchronousMotorVectorCtrlPars(
        sensorless=True,
        T_s=250e-6, delay=1,
        alpha_c=2*np.pi*200, alpha_fw=2*np.pi*20, alpha_s=2*np.pi*4,
        tau_M_max=2*base.tau_nom, i_s_max=1.5*base.i,
        w_nom=base.w,
        R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, p=3, J=.015,
        w_o=2*np.pi*40))
    # Speed reference and external load torque
    ref_ramp(mdl, w_max=base.w, tau_max=base.tau_nom, t_max=4)
    # Create the simulation object
    sim3 = Simulation(mdl, ctrl, base=base, print_opts=True)

    # %% Simulate
    sim1.simulate()
    # sim2.simulate()
    sim3.simulate()
    # Plot
    sim1.plot_figure()
    # sim2.plot_figure()
    sim3.plot_figure()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))
