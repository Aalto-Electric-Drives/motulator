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
   motulator.grid.control.DualVoltageFormingController
   motulator.grid.control.DualVoltageFormingControllerCfg
   motulator.grid.control.DualVoltageFormingOutputs
   motulator.grid.control.Feedbacks
   motulator.grid.control.GridConverterControlSystem
   motulator.grid.control.InverterNonlinearity
   motulator.grid.control.Measurements
   motulator.grid.control.ObserverBasedGridFormingController
   motulator.grid.control.ObserverBasedGridFormingControllerCfg
   motulator.grid.control.ObserverOutputs
   motulator.grid.control.PLL
   motulator.grid.control.PLLOutputs
   motulator.grid.control.PWM
   motulator.grid.control.PowerSynchronizationController
   motulator.grid.control.PowerSynchronizationControllerCfg
   motulator.grid.control.References
   motulator.grid.control.SequenceObserverBasedGridFormingController
   motulator.grid.control.SequenceObserverBasedGridFormingControllerCfg


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


   .. py:method:: compute_standby_output(fbk)

      
      Compute references while the gates are blocked.
















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


.. py:class:: DualVoltageFormingController(cfg)

   
   Dual-voltage-forming controller.

   This controller regulates both the DC-bus voltage and the converter output voltage
   magnitude [#Mou2025]_, extending the disturbance-observer-based grid-forming control
   [#Nur2024b]_. The disturbance observer estimates the quasi-static converter voltage
   as in `ObserverBasedGridFormingController`, and a second observer estimates the
   disturbance power, i.e., the external power fed to the DC bus minus the grid
   power and the resistive losses, from the energy balance of the DC-bus capacitor
   and the inductor. The control law feeds the estimated disturbance power forward and
   corrects the DC-bus energy error, which controls the active power without a
   cascaded DC-bus voltage controller. The disturbance observers provide the integral
   action and the synchronization. Transparent current control is also implemented.

   :param cfg: Dual-voltage-forming control configuration.
   :type cfg: DualVoltageFormingControllerCfg

   .. rubric:: Notes

   The control law equals the grid-forming mode of `ObserverBasedGridFormingController`
   with the power control gain `R_a/(1.5*v_c_ref) = k_d` and the active-power
   reference `p_g_ref = p_dis + p_g - alpha_dc*(W_dc_ref - W_dc)`, where `p_dis + p_g`
   is the estimated external power fed to the DC bus, excluding the resistive losses
   `1.5*R*|i_c|**2`. This equivalent reference is logged in `ref.p_g`.

   .. rubric:: References

   .. [#Mou2025] Mourouvin, Nurminen, Hinkkanen, Routimo, "Direct DC-bus control for
      grid-forming converters: Toward the concept of dual-voltage-forming converters,"
      IEEE Trans. Power Electron., 2025, https://doi.org/10.1109/TPEL.2025.3537332

   .. [#Nur2024b] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
      grid-forming converter control based on a disturbance observer," IEEE Trans.
      Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_dc_ref, v_c_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)

      
      Compute references while the gates are blocked.
















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


.. py:class:: DualVoltageFormingControllerCfg

   
   Dual-voltage-forming controller configuration.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param L: Total inductance estimate (H).
   :type L: float
   :param C_dc: DC-bus capacitance estimate (F).
   :type C_dc: float
   :param R: Total series resistance estimate (Ω), defaults to 0.
   :type R: float, optional
   :param k_d: DC-bus power gain (V/W), defaults to `1/(1.5*i_max)`, i.e., 1 p.u. with the
               base power `1.5*u_nom*i_max`.
   :type k_d: float | None, optional
   :param alpha_dc: Approximate closed-loop bandwidth of the DC-bus energy control (rad/s),
                    defaults to 2*pi*7.5.
   :type alpha_dc: float, optional
   :param alpha_o: Observer bandwidth of the quasi-static converter voltage (rad/s), defaults to
                   2*pi*100.
   :type alpha_o: float, optional
   :param alpha_o_dc: Observer bandwidth of the disturbance power (rad/s), defaults to 2*pi*60.
   :type alpha_o_dc: float, optional
   :param k_v: Voltage control gain, defaults to `alpha_o/w_nom`.
   :type k_v: float | None, optional
   :param alpha_c: Current control bandwidth (rad/s), defaults to 2*pi*400.
   :type alpha_c: float, optional
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to
                 `sqrt(2/3)*400`.
   :type u_nom: float, optional
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: DualVoltageFormingOutputs

   Bases: :py:obj:`motulator.grid.control._gfm_observer.ObserverOutputs`


   
   Feedback signals for the control system.
















   ..
       !! processed by numpydoc !!

.. py:class:: Feedbacks

   Bases: :py:obj:`Protocol`


   
   Protocol defining the required fields for feedback signals.
















   ..
       !! processed by numpydoc !!

.. py:class:: GridConverterControlSystem(inner_ctrl, dc_bus_voltage_ctrl = None, pwm = None)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Grid converter control system.

   This class defines the interface for control systems of grid converters. It is a
   generic class that can be used with different models, measurements, feedback
   signals, and reference signals.

   The inner controller gets the realized voltage of the ongoing sampling period for
   its observer, while the feedback voltage `u_c` for the control laws is the realized
   voltage averaged over the previous and the ongoing sampling periods. Before the
   modulation, the angle of the voltage reference is advanced by `k_comp*T_s*w_c`,
   where `k_comp` is given by the inner controller. The value 1.5 compensates for the
   computational delay of one sampling period and the delay of half a sampling period
   caused by the zero-order hold of the PWM [#Bae2003]_. The value 0 means that the
   angle is not advanced, which is used if the controller takes the delays into account
   itself.

   :param inner_ctrl: Inner controller. A grid-forming controller takes the AC voltage reference
                      and a grid-following controller the reactive power reference. A
                      DC-bus-forming controller takes the DC-bus voltage reference instead of the
                      active power reference.
   :type inner_ctrl: InnerController
   :param dc_bus_voltage_ctrl: DC-bus voltage controller, which gives the active power reference. If not
                               given, the active power reference is set externally, unless the inner
                               controller is DC-bus forming.
   :type dc_bus_voltage_ctrl: DCBusVoltageController, optional
   :param pwm: Pulse-width modulator, defaults to `PWM()`.
   :type pwm: PWM, optional

   .. rubric:: References

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
      synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
      2003, https://doi.org/10.1109/TIA.2003.810660















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller outputs based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)

      
      Compute controller output while the gates are blocked.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_command(ref)

      
      Extract the sampling period and the duty ratios from the references.
















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

      :param ref_fcn: Reactive power reference (VAr), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


.. py:class:: InverterNonlinearity(t_d = 0.0, u_th = 0.0, sign = np.sign)

   
   Duty-ratio error due to the dead time and the threshold voltages.

   The dead time blanks a leg for `t_d` after each switching instant, and the current
   direction determines the leg state during the blanking. Away from the duty-ratio
   limits, the resulting error is `sign(i)*t_d/(2*T_half)`, where `T_half` is the half
   carrier period. Pulses shorter than the dead time are suppressed, and clamped legs
   have no dead-time error. The threshold voltage `u_th` of the power devices, assumed
   equal for the switches and the diodes, causes the voltage drop `sign(i)*u_th` in
   each phase, i.e., the duty-ratio error `sign(i)*u_th/u_dc`, also in the clamped
   legs. The realized duty ratios are `d_abc - d_err`. This model is consistent with
   `motulator.common.model.VoltageSourceConverter`.

   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
   :param u_th: Threshold voltage (V) of the power devices, defaults to 0.
   :type u_th: float, optional
   :param sign: Current-direction function, defaults to `np.sign`. A smooth approximation may be
                used, matching the converter model.
   :type sign: Callable[[np.ndarray], np.ndarray], optional

   .. rubric:: Notes

   This switching-cycle averaged model assumes constant duty ratios and currents over a
   carrier period. It does not include transients due to changes in the switching
   commands.















   ..
       !! processed by numpydoc !!

.. py:class:: Measurements

   
   Measured signals.
















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
   rotating at the nominal grid angular frequency, and the disturbance observer is
   designed directly in discrete time. For other implementation options, see
   [#Nur2024]_.

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


   .. py:method:: compute_standby_output(fbk)

      
      Compute references while the gates are blocked.
















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
   :type R_a: float | None, optional
   :param k_v: Voltage control gain, defaults to `alpha_o/w_nom`.
   :type k_v: float | None, optional
   :param alpha_o: Observer bandwidth (rad/s), defaults to 2*pi*50.
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

.. py:class:: ObserverOutputs

   
   Feedback signals for the control system.
















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

   .. py:method:: compute_output(i_c_ab, u_g_meas_ab)

      
      Output estimates and coordinate transformed quantities.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update integral states.
















      ..
          !! processed by numpydoc !!


.. py:class:: PLLOutputs

   
   Feedback signals for the control system.
















   ..
       !! processed by numpydoc !!

.. py:class:: PWM(method = 'SVPWM', overmodulation = 'MPE', d_err = None, feedforward = True, k_pred = 1.5, d_min = 0.0, single_update = False)

   
   Duty ratios and realized voltage for three-phase carrier-based PWM.

   This computes the duty ratios for space-vector PWM or discontinuous PWM (DPWM)
   [#Hav1999]_, including the overmodulation [#Hav1999b]_. The realized voltage is
   reconstructed from the duty ratios and the measured DC-bus voltage, taking into
   account the computational delay of one sampling period.

   In DPWM, one leg at a time is clamped to the positive or negative DC rail, so that
   each leg stops switching for 120° of the fundamental period in the linear modulation
   range, and the average switching frequency is about 2/3 of the carrier frequency.
   The leg is clamped after the overmodulation, so the voltage is the same as in
   space-vector PWM. The switching losses are minimized if the clamped leg conducts the
   largest current: DPWM1 suits a current in phase with the voltage, DPWM2 a current
   lagging by about 30°, DPWM0 a current leading by about 30°, and DPWM3 a nearly
   reactive current. At the same average switching frequency, DPWM gives a lower
   current ripple than space-vector PWM only in the high modulation range [#Hav1998]_.

   Optionally, the duty-ratio error `d_err` caused by the inverter nonlinearities, such
   as the dead time and the threshold voltages of the power devices, is taken into
   account. The realized voltage is corrected for the error using the measured
   currents. By default, the error is also compensated for by feedforward using the
   predicted currents. In DPWM, the clamped leg stays at the rail, and its error is
   compensated for by shifting the other legs.

   Optionally, the duty ratio of a switching leg is limited to the range
   `[d_min, 1 - d_min]`, e.g., due to the gate drivers [#Wel2006]_. A duty ratio in the
   range `(0, d_min)` is rounded to 0 or `d_min`, and in the range `(1 - d_min, 1)` to
   `1 - d_min` or 1, whichever gives the realized duty ratio nearer to that of the
   voltage reference. The limited voltage reference and the realized voltage include
   the minimum pulses.

   :param method: PWM method, defaults to "SVPWM". Valid options are:

                  - "SVPWM": space-vector PWM
                  - "DPWM0": clamping in the 60° segments preceding the peaks of the phase voltage
                    reference
                  - "DPWM1": clamping in the 60° segments centered at the peaks
                  - "DPWM2": clamping in the 60° segments following the peaks
                  - "DPWM3": clamping in the 30° segments from 30° to 60° on both sides of the
                    peaks
   :type method: Literal["SVPWM", "DPWM0", "DPWM1", "DPWM2", "DPWM3"], optional
   :param overmodulation: Overmodulation method, defaults to "MPE". Valid options are:

                          - "MPE": minimum phase error
                          - "MME": minimum magnitude error
                          - "six_step": six-step operation
   :type overmodulation: Literal["MPE", "MME", "six_step"], optional
   :param d_err: Duty-ratio error as a function of the phase currents (A), duty ratios, DC-bus
                 voltage (V), and half carrier period (s), i.e., the realized duty ratios are
                 `d_abc - d_err(i_abc, d_abc, u_dc, T_half)`, defaults to None (no error). See
                 `InverterNonlinearity` for the dead time and the threshold voltages.
   :type d_err: Callable[[np.ndarray, np.ndarray, float, float], np.ndarray], optional
   :param feedforward: Compensate for `d_err` in the duty ratios, defaults to True.
   :type feedforward: bool, optional
   :param k_pred: Prediction factor of the currents for the feedforward compensation, defaults to
                  1.5, which corresponds to the middle of the sampling period in which the duty
                  ratios are applied after the computational delay of one sampling period.
   :type k_pred: float, optional
   :param d_min: Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0 (no
                 limit). The duty ratio applies to a half carrier period, so the shortest pulse
                 is `d_min` times the half carrier period when a leg starts or stops switching
                 and about twice that otherwise.
   :type d_min: float, optional
   :param single_update: Duty ratios are updated once per carrier period, i.e., the half carrier period
                         is `T_s/2`, defaults to False (twice per carrier period, the half carrier period
                         is `T_s`). Must match the system model.
   :type single_update: bool, optional

   .. rubric:: References

   .. [#Hav1999] Hava, Kerkman, Lipo, "Simple analytical and graphical methods for
      carrier-based PWM-VSI drives," IEEE Trans. Power Electron., 1999,
      https://doi.org/10.1109/63.737592

   .. [#Hav1999b] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
      triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
      https://doi.org/10.1109/28.777199

   .. [#Hav1998] Hava, Kerkman, Lipo, "A high-performance generalized discontinuous PWM
      algorithm," IEEE Trans. Ind. Appl., 1998, https://doi.org/10.1109/28.720446

   .. [#Wel2006] Welchko, Schulz, Hiti, "Effects and compensation of dead-time and
      minimum pulse-width limitations in two-level PWM voltage source inverters," Proc.
      IEEE IAS Annu. Meeting, 2006, https://doi.org/10.1109/IAS.2006.256630















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
      :param w: Angular speed (rad/s) of the synchronous coordinates, at which the currents
                are assumed to rotate in their prediction for the feedforward compensation
                of `d_err`.
      :type w: float

      :returns: * **d_abc** (*list[float]*) -- Duty ratios for the next sampling period.
                * **u_c_ab** (*complex*) -- Voltage (V) in stationary coordinates corresponding to the duty ratios
                  `d_abc`, i.e., the limited voltage reference with the duty-ratio error
                  compensation (if used) included.















      ..
          !! processed by numpydoc !!


   .. py:method:: get_realized_voltage(i_c_ab, u_dc, *, average = True)

      
      Get the realized voltage.

      This method is to be called at the sampling instant before the next duty ratios
      are computed, and a computational delay of one sampling period is assumed. The
      voltage is reconstructed from the duty ratios of the previous and the ongoing
      sampling periods and the measured DC-bus voltage, i.e., the variations of the
      DC-bus voltage and of the currents within the periods are not known. The
      measured currents are also stored for the feedforward compensation of the next
      duty ratios.

      :param i_c_ab: Measured converter current (A) in stationary coordinates.
      :type i_c_ab: complex
      :param u_dc: Measured DC-bus voltage (V).
      :type u_dc: float
      :param average: If True, the average voltage of the previous and the ongoing sampling
                      periods is returned. If False, the voltage of the ongoing sampling period is
                      returned. Defaults to True.
      :type average: bool, optional

      :returns: Realized converter voltage (V) in stationary coordinates. If `d_err` is
                given, the voltage is corrected for it using the measured currents.
      :rtype: complex















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


   .. py:method:: update(T_s, d_abc)

      
      Store the next duty ratios and the ongoing half carrier period.
















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


   .. py:method:: compute_standby_output(fbk)
      :abstractmethod:


      
      Compute references while the gates are blocked.
















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

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to sqrt(2/3)*400.
   :type u_nom: float, optional
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
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

.. py:class:: References

   
   Reference signals of the grid converter controllers.
















   ..
       !! processed by numpydoc !!

.. py:class:: SequenceObserverBasedGridFormingController(cfg)

   
   Sequence-separated disturbance-observer-based grid-forming controller.

   This implements the grid-forming mode of the control method described in
   [#Maa2026b]_, with transparent current control. The positive-sequence active power
   and converter-voltage magnitude are controlled, while the negative-sequence
   converter voltage is suppressed. The disturbance observer estimates the positive-
   and negative-sequence grid voltages. The active-power reference is limited to a
   realizable level, prioritizing the reactive and negative-sequence currents, and
   the sequence current references are limited such that the peak phase current does
   not exceed `i_max`.

   :param cfg: Grid-forming control configuration.
   :type cfg: SequenceObserverBasedGridFormingControllerCfg

   .. rubric:: Notes

   The control system operates in synchronous coordinates rotating at the nominal grid
   angular frequency. The controller compensates for the computational delay of one
   sampling period and the zero-order-hold delay of half a sampling period itself:
   the angle of the positive-sequence voltage reference is advanced by
   `1.5*w_nom*T_s`, and that of the negative sequence, which rotates in the opposite
   direction, is retarded by the same angle. Therefore, the control system does not
   advance the angle of the voltage reference (`k_comp = 0`).

   .. rubric:: References

   .. [#Maa2026b] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors,
      "Disturbance-observer-based grid-forming control for unbalanced grids," 2026,
      https://arxiv.org/abs/2608.11857















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(p_g_ref, v_c_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)

      
      Compute references while the gates are blocked.
















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


.. py:class:: SequenceObserverBasedGridFormingControllerCfg

   
   Sequence-separated disturbance-observer-based grid-forming controller configuration.

   :param i_max: Maximum current (A), peak value.
   :type i_max: float
   :param i_d_max: Maximum active current (A), peak value.
   :type i_d_max: float
   :param L: Total inductance estimate (H).
   :type L: float
   :param R: Total series resistance estimate (Ω), defaults to 0.
   :type R: float, optional
   :param R_a: Active resistance (Ω), defaults to `0.2*u_nom/i_max`.
   :type R_a: float | None, optional
   :param k_v: Voltage control gain, defaults to 1.
   :type k_v: float, optional
   :param k_n: Negative-sequence voltage gain, defaults to 2.
   :type k_n: float, optional
   :param alpha_c: Current control bandwidth (rad/s), defaults to 2*pi*300.
   :type alpha_c: float, optional
   :param alpha_l: Power reference limiter bandwidth (rad/s), defaults to 2*pi*50.
   :type alpha_l: float, optional
   :param w_o: Observer natural frequency (rad/s), defaults to `w_nom`.
   :type w_o: float | None, optional
   :param zeta_o: Observer damping ratio, defaults to 1.
   :type zeta_o: float, optional
   :param u_nom: Nominal grid voltage (V), line-to-neutral peak value, defaults to
                 `sqrt(2/3)*400`.
   :type u_nom: float, optional
   :param w_nom: Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
   :type w_nom: float, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

