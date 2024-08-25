Control Systems
===============

Main Control Loop
-----------------

By default, the discrete-time controllers in *motulator* run the following scheme in a main control loop:

   1. Get the feedback signals. This step may contain first getting the measurements and then optionally computing the observer outputs.
   2. Compute the reference signals (controller outputs) based on the feedback signals.
   3. Update the control system states for the next sampling instant.
   4. Save the feedback signals and the reference signals.
   5. Return the sampling period `T_s` and the duty ratios `d_abc` for the carrier comparison.

The main control loop is implemented in the base class for control systems in :class:`motulator.common.control.ControlSystem`.