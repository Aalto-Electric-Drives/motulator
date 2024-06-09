Usage
=====
After :doc:`installation`, *motulator* can be used by creating a continuous-time system model, a discrete-time controller, and a simulation object, as shown below. A 2.2-kW induction motor drive with a sensorless vector controller is used as an example.

.. code:: bash

   # Import packages
   import numpy as np
   from motulator.drive import model # Drive system models
   import motulator.drive.control.im as control # Controllers for IMs
   from motulator.drive.utils import plot  # Example plotting functions

   # Continuous-time model for the drive system
   converter = model.Inverter(u_dc=540)
   machine = model.InductionMachineInvGamma(
      R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
   mechanics = model.StiffMechanicalSystem(J=.015)
   mdl = model.Drive(converter, machine, mechanics)
   
   # Discrete-time controller
   par = control.ModelPars(
      R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
   cfg = control.CurrentReferenceCfg(par, max_i_s=1.5*np.sqrt(2)*5)
   ctrl = control.VectorCtrl(par, cfg)

   # Acceleration at t = 0.2 s and load torque step of 14 Nm at t = 0.75 s 
   ctrl.ref.w_m = lambda t: (t > .2)*(2*np.pi*50)
   mdl.mechanics.tau_L = lambda t: (t > .75)*14

   # Create a simulation object, simulate, and plot example figures
   sim = model.Simulation(mdl, ctrl)
   sim.simulate(t_stop=1.5)
   plot(sim)

The :doc:`auto_examples/index` folder includes a variety of different example scripts to run the simulation. Drive system and controller configurations can be changed as shown in the examples. Furthermore, new features can be added by modifying the source code.