Usage
=====
After :doc:`installation`, *motulator* can be used by creating a continuous-time system model, a discrete-time controller, and a simulation object, as shown below. A 2.2-kW induction motor drive with a sensorless vector controller is used as an example.

.. code:: python

   # Import packages
   import numpy as np
   from motulator.drive import model  # Drive system models
   import motulator.drive.control.im as control  # Controllers for IMs
   from motulator.drive.utils import (
      plot,  # Example plotting functions
      InductionMachinePars, InductionMachineInvGammaPars)  # Helper classes

   # Continuous-time model for the drive system
   par = InductionMachineInvGammaPars(
      n_p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224)
   mdl_par = InductionMachinePars.from_inv_gamma_model_pars(par)
   machine = model.InductionMachine(mdl_par)
   mechanics = model.StiffMechanicalSystem(J=.015)
   converter = model.VoltageSourceConverter(u_dc=540)
   mdl = model.Drive(converter, machine, mechanics)

   # Discrete-time controller
   par = InductionMachineInvGammaPars( # Machine model parameter estimates
      n_p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224)
   cfg = control.CurrentReferenceCfg(par, max_i_s=1.5*np.sqrt(2)*5)
   ctrl = control.CurrentVectorControl(par, cfg, J=.015)

   # Acceleration at t = 0.2 s and load torque step of 14 Nm at t = 0.75 s
   ctrl.ref.w_m = lambda t: (t > .2)*2*np.pi*50
   mdl.mechanics.tau_L = lambda t: (t > .75)*14

   # Create a simulation object, simulate, and plot example figures
   sim = model.Simulation(mdl, ctrl)
   sim.simulate(t_stop=1.5)
   plot(sim)

The Examples section includes a variety of different example scripts to run the simulation. System model and controller configurations can be changed as shown in the examples. Furthermore, new features can be added by modifying the source code.