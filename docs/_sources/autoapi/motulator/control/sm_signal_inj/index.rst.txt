:py:mod:`motulator.control.sm_signal_inj`
=========================================

.. py:module:: motulator.control.sm_signal_inj

.. autoapi-nested-parse::

   Sensorless control with signal injection for synchronous motor drives.

   This module contains a simple example of square-wave signal injection for low-
   speed operation. A phase-locked loop is used to track the rotor position. For
   a wider speed range, signal injection could be combined to a model-based
   observer. The effects of magnetic saturation are not compensated for in this
   version.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm_signal_inj.SynchronousMotorSignalInjectionCtrlPars
   motulator.control.sm_signal_inj.SynchronousMotorSignalInjectionCtrl
   motulator.control.sm_signal_inj.SignalInjection
   motulator.control.sm_signal_inj.PhaseLockedLoop




.. py:class:: SynchronousMotorSignalInjectionCtrlPars

   Bases: :py:obj:`motulator.control.sm_vector.SynchronousMotorVectorCtrlPars`

   
   Square-wave signal injection parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: U_inj
      :type: float
      :value: 200

      


.. py:class:: SynchronousMotorSignalInjectionCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Sensorless control with signal injection for a synchronous motor drive.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: SignalInjection(pars)

   
   Estimate the rotor position error based on signal injection.

   This signal injection method estimates the rotor position error based on
   the injected switching frequency signal, according to [R50acc62e7f3b-1]_. The estimate
   can be used in a phase-locked loop or in a state observer to robustify
   low-speed sensorless operation.

   :param pars: Control parameters.
   :type pars: SynchronousMotorSignalInjectionCtrlPars

   .. rubric:: References

   .. [R50acc62e7f3b-1] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless
      method in IPMSM," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2210175















   ..
       !! processed by numpydoc !!
   .. py:method:: output(i_sq)

      
      Compute the rotor position estimation error.

      :param i_sq: Stator current q-component in estimated rotor coordinates.
      :type i_sq: float

      :returns: **err** -- Rotor position estimation error.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: update(i_s)

      
      Store the old current values for the next sampling period.

      :param i_s: Stator current in estimated rotor coordinates.
      :type i_s: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: filter_current(i_s)

      
      Filter the stator current using the previously measured value.

      :param i_s: Unfiltered stator current in estimated rotor coordinates.
      :type i_s: complex

      :returns: **i_s_filt** -- Filtered stator current in estimated rotor coordinates.
      :rtype: complex















      ..
          !! processed by numpydoc !!


.. py:class:: PhaseLockedLoop(pars)

   
   Simple phase-locked loop for rotor-position estimation.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: update(err)

      
      Update the states for the next sampling period.

      :param err:
      :type err: Rotor position error.















      ..
          !! processed by numpydoc !!


