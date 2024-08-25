Common
======

Main Control Loop
-----------------

By default, discrete-time control systems in *motulator* run the following scheme in their main control loops:

   1. Get the feedback signals for the controllers. This step may contain first getting the measurements and then optionally computing the observer outputs. These measured and estimated signals are collected to the SimpleNamespace object named `fbk`. 
   2. Get the reference signals and compute the controller outputs based on the feedback signals `fbk`. These reference signals are collected to the SimpleNamespace object named `ref`. 
   3. Update the states of the control system for the next sampling instant.
   4. Save the feedback signals `fbk` and the reference signals `ref` so they can be accessed after the simulation.
   5. Return the sampling period `T_s` and the duty ratios `d_abc` for the carrier comparison.

A template of this main control loop is available in the base class for control systems in :class:`motulator.common.control.ControlSystem`. Using this template is not necessary, but it may simplify the implementation of new control systems.
