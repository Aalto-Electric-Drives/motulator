Usage
=====
After :doc:`installation`, motulator can be used by creating a continuous-time system model, a discrete-time controller, and a simulation object, as shown below

.. code:: bash

   import motulator as mt

   # Continuous-time model for the drive system
   motor = mt.InductionMotor()     # Motor model
   mech = mt.Mechanics()           # Mechanics model
   conv = mt.Inverter()            # Converter model
   mdl = mt.InductionMotorDrive(motor, mech, conv)
   
   # Discrete-time controller
   pars = mt.InductionMotorVectorCtrlPars()    # Dataclass of control parameters
   ctrl = mt.InductionMotorVectorCtrl(pars)    # Sensorless controller
   
   # Create a simulation object, simulate, and plot example figures
   sim = mt.Simulation(mdl, ctrl)
   sim.simulate()
   mt.plot(sim)

The :doc:`auto_examples/index` folder includes a variety of different example scripts to run the simulation. Drive system and controller configurations can be changed as shown in the examples. Furthermore, new features can be added by modifying the source code.