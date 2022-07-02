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

The :ref:`examples` includes a variety of different example scripts to run the simulation. These example scripts use the :mod:`motulator.simulation` class in order to simulate a model and :mod:`motulator.plots` to plot the simulation results. Basic drive system and controller configurations for the simulation can be changed as shown in the example scripts. Optionally multiple simulation objects can be run in parallel (see :doc:`tutorial`).