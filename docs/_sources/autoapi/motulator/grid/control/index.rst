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
   motulator.grid.control.CurrentVectorControllerCfg
   motulator.grid.control.DCBusVoltageController
   motulator.grid.control.GridConverterControlSystem
   motulator.grid.control.ObserverBasedGridFormingController
   motulator.grid.control.ObserverBasedGridFormingControllerCfg
   motulator.grid.control.PLL
   motulator.grid.control.PWM
   motulator.grid.control.PowerSynchronizationController
   motulator.grid.control.PowerSynchronizationControllerCfg


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

.. py:class:: CurrentVectorController(cfg)

   
   Current-vector grid-following controller.

   :param cfg: Configuration parameters.
   :type cfg: CurrentVectorControllerCfg















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, q_g_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_c_ab, meas)

      
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


.. py:class:: CurrentVectorControllerCfg

   
   Configuration for current-vector grid-following controller.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param L: Filter inductance (H).
   :type L: float
   :param alpha_c: Current-control bandwidth (rad/s), defaults to 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_i: Integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float | None, optional
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to sqrt(2/3)*400.
   :type u_nom: float, optional
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
   :param alpha_pll: PLL frequency-tracking bandwidth (rad/s), defaults to 2*pi*20.
   :type alpha_pll: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















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


.. py:class:: GridConverterControlSystem(inner_ctrl, dc_bus_voltage_ctrl = None, pwm = None)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Grid converter control system.

   This class defines the interface for control systems of grid converters. It is a
   generic class that can be used with different models, measurements, feedback
   signals, and reference signals.

   :param inner_ctrl: Inner controller.
   :type inner_ctrl: GridFormingController | GridFollowingController
   :param dc_bus_voltage_ctrl: DC-bus voltage controller. If not given, power-control mode is used.
   :type dc_bus_voltage_ctrl: DCBusVoltageController, optional
   :param pwm: Pulse-width modulator, defaults to `PWM()`.
   :type pwm: PWM, optional















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
      :type ref_fcn: float | Callable[[float], float]















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


.. py:class:: ObserverBasedGridFormingController(cfg)

   
   Disturbance-observer-based grid-forming controller.

   This implements the RFPSC-type grid-forming mode of the control method described in
   [#Nur2024]_. Transparent current control is also implemented. Optionally, the
   active-power reference is limited to a realizable level, prioritizing the reactive
   current, as described in [#Maa2026]_ (here reduced to balanced conditions). This
   limitation helps to maintain synchronism during grid-voltage sags in weak grids.

   :param cfg: Grid-forming control configuration.
   :type cfg: ObserverBasedGridFormingControllerCfg

   .. rubric:: Notes

   In this implementation, the control system operates in synchronous coordinates
   rotating at the nominal grid angular frequency. For other implementation options,
   see [#Nur2024]_.

   .. rubric:: References

   .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming
      converter control based on a disturbance observer," IEEE Trans. Power Electron.,
      2024, https://doi.org/10.1109/TPEL.2024.3433503

   .. [#Maa2026] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors,
      "Disturbance-observer-based grid-forming control for unbalanced grids," 2026,
      https://arxiv.org/abs/2608.11857















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, v_c_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_c_ab, meas)

      
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


.. py:class:: ObserverBasedGridFormingControllerCfg

   
   Disturbance-observer-based grid-forming controller configuration.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param L: Total inductance estimate (H).
   :type L: float
   :param R: Total series resistance estimate (Ω), defaults to 0.
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
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional
   :param i_d_max: Maximum active current (A), peak value, for the active-power reference
                   limitation. If not given, the active-power reference is not limited. A value
                   somewhat below `i_max` is recommended, e.g., `i_d_max = 0.85*i_max`.
   :type i_d_max: float, optional
   :param alpha_l: Power-limitation bandwidth (rad/s), defaults to 2*pi*50.
   :type alpha_l: float, optional















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


.. py:class:: PWM(k_comp = 1.5, overmodulation = 'MPE', d_err = None, feedforward = True, k_pred = 1.5, d_min = 0.0)

   
   Duty ratios and realized voltage for three-phase space-vector PWM.

   This computes the duty ratios corresponding to standard space-vector PWM and
   overmodulation [#Hav1999]_. The realized voltage is computed from the duty ratios
   and the measured DC-bus voltage. The digital delay effects are taken into account
   in the realized voltage [#Bae2003]_.

   Optionally, the duty-ratio error caused by the inverter nonlinearities (such as the
   dead time and the voltage drops of the power devices) is modeled as a function of
   the phase currents and duty ratios. The realized voltage is corrected for this
   error using the measured currents, which correspond to the same instant as the
   realized voltage, and the duty ratios of the corresponding sampling periods.
   Furthermore, the error can be compensated for by feedforward, in which case the
   currents are predicted to the middle of the sampling period in which the duty ratios
   are applied, assuming that they rotate at the angular speed of the synchronous
   coordinates. Near the duty-ratio limits, the feedforward may not fully cancel a
   duty-dependent error.

   Optionally, the pulses are limited to a minimum width, e.g., due to the gate
   drivers [#Wel2006]_: the duty ratio of a switching leg is in the range
   `[d_min, 1 - d_min]`, while the duty ratios 0 and 1 (a leg not switching) are not
   limited. A duty ratio in the range `(0, d_min)` is rounded to 0 (the pulse is
   dropped) or to `d_min`, and in the range `(1 - d_min, 1)` to `1 - d_min` or 1,
   whichever gives the realized duty ratio nearer to the duty ratio of the voltage
   reference. If the duty-ratio error is compensated for, the realized duty ratios are
   those given by `d_err`, so that the compensation also accounts for the minimum
   pulses. The limited voltage reference and the realized voltage are computed from
   the limited duty ratios.

   :param k_comp: Compensation factor for the angular delay effect on the voltage reference,
                  defaults to 1.5. Use 0 if the controller compensates for the delays itself,
                  e.g., in direct discrete-time designs.
   :type k_comp: float, optional
   :param overmodulation: Overmodulation method, defaults to "MPE". Valid options are:
                          - "MPE": minimum phase error
                          - "MME": minimum magnitude error
                          - "six_step": six-step operation
   :type overmodulation: Literal["MPE", "MME", "six_step"], optional
   :param d_err: Duty-ratio error as a function of the phase currents (A) and duty ratios,
                 i.e., the realized duty ratios are `d_abc - d_err(i_abc, d_abc)`, defaults
                 to None (no error). For the dead time, use
                 `motulator.common.utils.dead_time_error`, whose sampling period must equal
                 that of the control system.
   :type d_err: Callable[[np.ndarray, np.ndarray], np.ndarray], optional
   :param feedforward: Compensate for `d_err` in the duty ratios, defaults to True.
   :type feedforward: bool, optional
   :param k_pred: Prediction factor of the currents for the feedforward compensation, defaults to
                  1.5, which corresponds to the middle of the sampling period in which the duty
                  ratios are applied after the computational delay of one sampling period.
   :type k_pred: float, optional
   :param d_min: Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0
                 (no limit). The duty ratio applies to a sampling period (a half of the carrier
                 period), so the shortest pulse is `d_min*T_s` when a leg starts or stops
                 switching and about `2*d_min*T_s` otherwise.
   :type d_min: float, optional

   .. rubric:: References

   .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
      triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
      https://doi.org/10.1109/28.777199

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
      synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
      2003, https://doi.org/10.1109/TIA.2003.810660

   .. [#Wel2006] Welchko, Schulz, Hiti, "Effects and compensation of dead-time and
      minimum pulse-width limitations in two-level PWM voltage source inverters,"
      Proc. IEEE IAS Annu. Meeting, 2006, https://doi.org/10.1109/IAS.2006.256630















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(T_s, u_c_ref_ab, u_dc, w)

      
      Compute the duty ratios and the corresponding voltage.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float
      :param w: Angular speed of synchronous coordinates (rad/s).
      :type w: float

      :returns: * **d_abc** (*list[float]*) -- Duty ratios for the next sampling period.
                * **u_c_ab** (*complex*) -- Voltage (V) in stationary coordinates corresponding to the duty ratios
                  `d_abc`, i.e., the limited voltage reference with the delay-compensating
                  angle advance and the duty-ratio error compensation (if used) included.















      ..
          !! processed by numpydoc !!


   .. py:method:: duty_ratios(u_c_ref_ab, u_dc)

      
      Compute the duty ratios for three-phase space-vector PWM.

      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **d_abc** -- Duty ratios.
      :rtype: list[float]















      ..
          !! processed by numpydoc !!


   .. py:method:: get_realized_voltage(i_c_ab, u_dc, *, average = True)

      
      Get the realized voltage.

      This method is to be called at the sampling instant before the next duty ratios
      are computed, and a computational delay of one sampling period is assumed. The
      voltage is computed from the duty ratios of the previous and the ongoing
      sampling periods and the measured DC-bus voltage. The measured currents are also
      stored for the feedforward compensation of the next duty ratios.

      :param i_c_ab: Measured converter current (A) in stationary coordinates.
      :type i_c_ab: complex
      :param u_dc: Measured DC-bus voltage (V).
      :type u_dc: float
      :param average: If True, the average voltage of the previous and the ongoing sampling
                      periods is returned. If False, the voltage of the ongoing sampling period
                      is returned. Defaults to True.
      :type average: bool, optional

      :returns: Realized converter voltage (V) in stationary coordinates. If `d_err` is
                given, the voltage is corrected for it using the measured currents.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: limit_pulses(d_abc, d_ref, i_abc = None)

      
      Limit the duty ratios of the switching legs to `[d_min, 1 - d_min]`.

      A duty ratio in the range `(0, d_min)` is replaced by 0 or `d_min`, and in the
      range `(1 - d_min, 1)` by `1 - d_min` or 1, whichever gives the realized duty
      ratio nearer to the reference (if equally near, the leg switches). The duty
      ratios 0 and 1 are not changed.

      :param d_abc: Duty ratios, including the compensation of the duty-ratio error.
      :type d_abc: list[float]
      :param d_ref: Duty ratios of the voltage reference, i.e., the realized duty ratios aimed
                    at.
      :type d_ref: ndarray, shape (3,)
      :param i_abc: Phase currents (A) for the duty-ratio error `d_err`, if it is compensated
                    for, in which case the realized duty ratios are `d_abc - d_err(i_abc,
                    d_abc)`. Defaults to None, in which case the realized duty ratios are
                    `d_abc`.
      :type i_abc: ndarray, shape (3,), optional

      :returns: * **d_abc** (*list[float]*) -- Limited duty ratios.
                * **d_realized** (*ndarray, shape (3,)*) -- Realized duty ratios: `d_ref` for the legs whose duty ratios were not
                  changed and the realized duty ratios of the limited legs.















      ..
          !! processed by numpydoc !!


   .. py:method:: six_step_overmodulation(u_c_ref_ab, u_dc)
      :staticmethod:


      
      Overmodulation up to six-step operation.

      This method modifies the angle of the voltage reference vector in the
      overmodulation region such that the six-step operation is reached [#Bol1997]_.

      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **u_c_ref_ab** -- Modified converter voltage reference (V) in stationary coordinates.
      :rtype: complex

      .. rubric:: References

      .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of SVM
         inverters in the overmodulation range," IEEE Trans. Ind. Appl., 1997,
         https://doi.org/10.1109/28.568019















      ..
          !! processed by numpydoc !!


   .. py:method:: update(d_abc)

      
      Store the duty ratios of the next sampling period.
















      ..
          !! processed by numpydoc !!


.. py:class:: PowerSynchronizationController(cfg)

   
   Reference-feedforward power-synchronization controller.

   This implements the reference-feedforward power-synchronization control [#Har2020]_.

   :param cfg: Configuration object.
   :type cfg: PowerSynchronizationControllerCfg

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


   .. py:method:: get_feedback(u_c_ab, meas)

      
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


.. py:class:: PowerSynchronizationControllerCfg

   
   Configuration for the reference-feedforward power-synchronization controller.

   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value.
   :type u_nom: float
   :param w_nom: Nominal grid angular frequency (rad/s).
   :type w_nom: float
   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param R: Total series resistance (Ω), defaults to 0.
   :type R: float, optional
   :param R_a: Active resistance (Ω), defaults to ``0.25*u_nom/i_max``.
   :type R_a: float | None, optional
   :param w_b: Low-pass filter bandwidth (rad/s), defaults to 2*pi*5.
   :type w_b: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

