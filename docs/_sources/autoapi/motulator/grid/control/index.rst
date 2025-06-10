motulator.grid.control
======================

.. py:module:: motulator.grid.control

.. autoapi-nested-parse::

   
   Controllers for grid converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.control.CurrentController
   motulator.grid.control.CurrentLimiter
   motulator.grid.control.CurrentVectorController
   motulator.grid.control.DCBusVoltageController
   motulator.grid.control.GridConverterControlSystem
   motulator.grid.control.ObserverBasedGridFormingController
   motulator.grid.control.PLL
   motulator.grid.control.PowerSynchronizationController


Package Contents
----------------

.. py:class:: CurrentController(L, alpha_c, alpha_i = None)

   Bases: :py:obj:`motulator.common.control._controllers.ComplexPIController`


   
   2DOF PI current controller for grid converters.

   This class provides an interface for a current controller for grid converters. The
   gains are initialized based on the desired closed-loop bandwidth and the filter
   inductance.

   :param L: Inductance (H).
   :type L: float
   :param alpha_c: Current-control bandwidth (rad/s).
   :type alpha_c: float
   :param alpha_i: Integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentLimiter(i_max)

   
   Limit the amplitude of the input signal.

   :param i_max: Maximum current (A).
   :type i_max: float

   :returns: Limited signal.
   :rtype: complex















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentVectorController(i_max, L, alpha_c = 2 * pi * 400, alpha_i = None, u_nom = sqrt(2 / 3) * 400, w_nom = 2 * pi * 50, alpha_pll = 2 * pi * 20, T_s = 0.000125)

   
   Current-vector grid-following controller.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param L: Filter inductance (H).
   :type L: float
   :param alpha_c: Current-control bandwidth (rad/s), defaults to 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_i: Integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to
                 `sqrt(2/3)*400`.
   :type u_nom: float, optional
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
   :param alpha_pll: PLL frequency-tracking bandwidth (rad/s), defaults to 2*pi*20.
   :type alpha_pll: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, q_g_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: DCBusVoltageController(C_dc, alpha_dc, p_max = inf)

   Bases: :py:obj:`motulator.common.control._controllers.PIController`


   
   DC-bus voltage PI controller.

   This controller regulates the energy stored in the DC-bus capacitor (scaled square
   of the DC-bus voltage) in order to have a linear closed-loop system [#Hur2001]_.

   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float
   :param alpha_dc: Approximate closed-loop bandwidth (rad/s).
   :type alpha_dc: float
   :param p_max: Limit for the maximum converter power (W), defaults to `inf`.
   :type p_max: float, optional

   .. rubric:: References

   .. [#Hur2001] Hur, Jung, Nam, "A fast dynamic DC-link power-balancing scheme for a
      PWM converter-inverter system," IEEE Trans. Ind. Electron., 2001,
      https://doi.org/10.1109/41.937412















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(y_ref, y, u_ff = 0.0)

      
      Compute the controller output.

      :param y_ref: Reference signal.
      :type y_ref: float
      :param y: Feedback signal.
      :type y: float
      :param u_ff: Feedforward signal, defaults to 0.
      :type u_ff: float, optional

      :returns: **u** -- Controller output.
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: GridConverterControlSystem(inner_ctrl, dc_bus_voltage_ctrl = None)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Grid converter control system.

   This class defines the interface for control systems of grid converters. It is a
   generic class that can be used with different models, measurements, feedback
   signals, and reference signals.

   :param inner_ctrl: Inner controller.
   :type inner_ctrl: GridFormingController | GridFollowingController
   :param dc_bus_voltage_ctrl: DC-bus voltage controller. If not given, power-control mode is used.
   :type dc_bus_voltage_ctrl: DCBusVoltageController, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller outputs based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements from sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_ac_voltage_ref(ref_fcn)

      
      Set the external ac voltage reference.

      :param ref_fcn: AC-side converter voltage reference (V), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_dc_bus_voltage_ref(ref_fcn)

      
      Set the external DC-bus voltage reference.

      :param ref_fcn: DC-bus voltage reference (V), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_power_ref(ref_fcn)

      
      Set the external active power reference.

      :param ref_fcn: Active power reference (W), constant or a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_reactive_power_ref(ref_fcn)

      
      Set the external reactive power reference.

      :param ref_fcn: Power reference (VAr), constant or a function of time.
      :type ref_fcn: Callable[[float], float] | float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedGridFormingController(i_max, L, R = 0.0, R_a = None, k_v = None, alpha_o = 2 * pi * 50, alpha_c = 2 * pi * 400, u_nom = sqrt(2 / 3) * 400, w_nom = 2 * pi * 50, T_s = 0.000125)

   
   Disturbance-observer-based grid-forming controller.

   This implements the RFPSC-type grid-forming mode of the control method described in
   [#Nur2024]_. Transparent current control is also implemented.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param L: Total inductance (H).
   :type L: float
   :param R: Total series resistance (Ω), defaults to 0.
   :type R: float, optional
   :param R_a: Active resistance (Ω), defaults to `0.25*u_nom/i_max`.
   :type R_a: float, optional
   :param k_v: Voltage control gain, defaults to `alpha_o/w_nom`.
   :type k_v: float, optional
   :param alpha_o: Observer gain (rad/s), defaults to 2*pi*50.
   :type alpha_o: float, optional
   :param alpha_c: Current control bandwidth (rad/s), defaults to 2*pi*400.
   :type alpha_c: float, optional
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to
                 `sqrt(2/3)*400`.
   :type u_nom: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional

   .. rubric:: Notes

   In this implementation, the control system operates in synchronous coordinates
   rotating at the nominal grid angular frequency. For other implementation options,
   see [#Nur2024]_.

   .. rubric:: References

   .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming
      converter control based on a disturbance observer," IEEE Trans. Power Electron.,
      2024, https://doi.org/10.1109/TPEL.2024.3433503















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, v_c_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: PLL(u_nom, w_nom, alpha_pll)

   
   Phase-locked loop including the voltage-magnitude filtering.

   This class provides a simple frequency-tracking phase-locked loop. The magnitude of
   the measured PCC voltage is also filtered.

   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value.
   :type u_nom: float
   :param w_nom: Nominal grid angular frequency (rad/s).
   :type w_nom: float
   :param alpha_pll: PLL frequency-tracking bandwidth (rad/s).
   :type alpha_pll: float















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_c_ab, i_c_ab, u_g_meas_ab)

      
      Output estimates and coordinate transformed quantities.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update integral states.
















      ..
          !! processed by numpydoc !!


.. py:class:: PowerSynchronizationController(u_nom, w_nom, i_max, R = 0.0, R_a = None, w_b = 2 * pi * 5, T_s = 0.000125)

   
   Reference-feedforward power-synchronization controller.

   This implements the reference-feedforward power-synchronization control [#Har2020]_.

   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value.
   :type u_nom: float
   :param w_nom: Nominal grid angular frequency (rad/s).
   :type w_nom: float
   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param R: Total series resistance (Ω), defaults to 0.
   :type R: float, optional
   :param R_a: Active resistance (Ω), defaults to 0.25*u_nom/i_max.
   :type R_a: float, optional
   :param w_b: Low-pass filter bandwidth (rad/s), defaults to 2*pi*5.
   :type w_b: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional

   .. rubric:: References

   .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
      power-synchronization control," IEEE Trans. Power Electron., 2020,
      https://doi.org/10.1109/TPEL.2020.2970991















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, v_c_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


