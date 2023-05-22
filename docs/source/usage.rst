Usage
=====
After :doc:`installation`, motulator can be used by creating a continuous-time system model, a discrete-time controller, and a simulation object, as shown below. A 2.2-kW induction motor drive with a sensorless vector controller is used as an example.

.. code:: bash

   # Import packages
   import numpy as np
   from motulator import model  # System models
   from motulator import control  # Controllers
   from motulator import plot  # Example plotting functions

   # Continuous-time model for the drive system
   machine = model.im.InductionMachineInvGamma(
      R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
   mechanics = model.Mechanics(J=.015)
   converter = model.Inverter(u_dc=540)
   mdl = model.im.Drive(machine, mechanics, converter)
   
   # Discrete-time controller
   par = control.im.ModelPars(
      R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
   ref = control.im.CurrentReferencePars(par, i_s_max=1.5*np.sqrt(2)*5)
   ctrl = control.im.VectorCtrl(par, ref)

   # Acceleration at t = 0.2 s and load torque step of 14 Nm at t = 0.75 s 
   ctrl.w_m_ref = lambda t: (t > .2)*(2*np.pi*50)
   mdl.mechanics.tau_L_t = lambda t: (t > .75)*14

   # Create a simulation object, simulate, and plot example figures
   sim = model.Simulation(mdl, ctrl, pwm=False)
   sim.simulate(t_stop=1.5)
   plot(sim)

The :doc:`auto_examples/index` folder includes a variety of different example scripts to run the simulation. Drive system and controller configurations can be changed as shown in the examples. Furthermore, new features can be added by modifying the source code.