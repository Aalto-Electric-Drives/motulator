:orphan:

:py:mod:`motulator.control.sm._signal_inj`
==========================================

.. py:module:: motulator.control.sm._signal_inj

.. autoapi-nested-parse::

   Sensorless control with signal injection for synchronous machine drives.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._signal_inj.SignalInjectionCtrl
   motulator.control.sm._signal_inj.SignalInjection
   motulator.control.sm._signal_inj.PhaseLockedLoop




.. py:class:: SignalInjectionCtrl(par, ref, T_s=0.00025)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Sensorless control with signal injection for synchronous machine drives.

   This class implements a square-wave signal injection for low-speed
   operation according to [#Kim2012]_. A phase-locked loop is used to track
   the rotor position.

   .. rubric:: Notes

   For a wider speed range, signal injection could be combined to a
   model-based observer. The effects of magnetic saturation are not
   compensated for in this version.

   .. rubric:: References

   .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection
      sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2210175

   :param T_s: Sampling period (s).
   :type T_s: float
   :param pars: Machine model parameters.
   :type pars: ModelPars
   :param U_inj: Amplitude of the injected voltage (V).
   :type U_inj: float
   :param w_o: PLL natural frequency (rad/s).
   :type w_o: float

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentCtrl

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: current_ref

      Current reference generator.

      :type: CurrentReference

   .. attribute:: pll

      Phase-locked loop.

      :type: PhaseLockedLoop

   .. attribute:: signal_inj

      Signal injection.

      :type: SignalInjection

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s).

      :type: callable

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM















   ..
       !! processed by numpydoc !!

.. py:class:: SignalInjection(par, U_inj)


   
   Estimate the rotor position error based on signal injection.

   This signal injection method estimates the rotor position error based on
   the injected switching frequency signal. The estimate can be used in a
   phase-locked loop or in a state observer to robustify low-speed sensorless
   operation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param U_inj: Injected voltage amplitude (V).
   :type U_inj: float















   ..
       !! processed by numpydoc !!
   .. py:method:: output(T_s, i_sq)

      
      Compute the rotor position estimation error.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param i_sq: q-axis stator current (A) in estimated rotor coordinates.
      :type i_sq: float

      :returns: **err** -- Rotor position estimation error (electrical rad).
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

      :param i_s: Unfiltered stator current (A) in estimated rotor coordinates.
      :type i_s: complex

      :returns: **i_s_filt** -- Filtered stator current (A) in estimated rotor coordinates.
      :rtype: complex















      ..
          !! processed by numpydoc !!


.. py:class:: PhaseLockedLoop(w_o)


   
   Simple phase-locked loop for rotor-position estimation.

   :param w_o: Natural frequency (rad/s).
   :type w_o: float















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, err)

      
      Update the states for the next sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param err: Rotor position error (rad).
      :type err: float















      ..
          !! processed by numpydoc !!


