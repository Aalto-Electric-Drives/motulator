motulator.common.control
========================

.. py:module:: motulator.common.control

.. autoapi-nested-parse::

   
   Common control functions and classes.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.common.control.ComplexPIController
   motulator.common.control.ControlSystem
   motulator.common.control.DiscreteComplexController
   motulator.common.control.InverterNonlinearity
   motulator.common.control.PIController
   motulator.common.control.PWM
   motulator.common.control.RateLimiter
   motulator.common.control.TimeSeries


Package Contents
----------------

.. py:class:: ComplexPIController(k_p, k_i, k_t = None)

   
   2DOF synchronous-frame complex-vector PI controller.

   This implements a discrete-time 2DOF synchronous-frame complex-vector PI controller
   [#Bri2000]_. The continuous-time counterpart of the controller is::

       u = k_t*i_ref - k_p*i + (k_i + 1j*w_c*k_t)/s*(i_ref - i) + u_ff

   where `u` is the controller output, `i_ref` is the reference signal, `i` is the
   feedback signal, `w_c` is the angular speed of synchronous coordinates, `u_ff` is
   the feedforward signal, and `1/s` refers to integration. The 1DOF version is
   obtained by setting ``k_t = k_p``. The integrator anti-windup is implemented based
   on the realized controller output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain, defaults to `k_p`.
   :type k_t: float, optional

   .. rubric:: Notes

   This controller can be used, e.g., as a current controller. In this case, `i`
   corresponds to the stator current and `u` to the stator voltage.

   .. rubric:: References

   .. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current regulators using
      complex vectors," IEEE Trans. Ind. Appl., 2000, https://doi.org/10.1109/28.845057















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(i_ref, i, u_ff = 0j)

      
      Compute the controller output.

      :param i_ref: Reference signal.
      :type i_ref: complex
      :param i: Feedback signal.
      :type i: complex
      :param u_ff: Feedforward signal, defaults to 0.
      :type u_ff: complex, optional

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: preset(u, w_c)

      
      Preset the integral state to the steady state of the output.

      In the steady state, the reference and feedback signals are zero, and the
      output is `u`, e.g., the back-EMF of a machine at zero current.

      :param u: Controller output.
      :type u: complex
      :param w_c: Angular speed of the reference frame (rad/s), not needed in this design.
      :type w_c: float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u, w_c)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u: Realized (limited) controller output.
      :type u: complex
      :param w_c: Angular speed of the reference frame (rad/s).
      :type w_c: float















      ..
          !! processed by numpydoc !!


.. py:class:: ControlSystem

   Bases: :py:obj:`Protocol`


   
   Base class for control systems.

   This class defines the interface for control systems. It is a generic class that can
   be used with different models, measurements, feedback signals, and reference
   signals. The class provides methods for saving, post-processing, and clearing data.

   The gates of the converter can be blocked with `set_enable`. While the gates are
   blocked, the control system runs its control loop with `compute_standby_output`
   in place of `compute_output`, so that its observers keep running, and the model
   holds the switches open.















   ..
       !! processed by numpydoc !!

   .. py:method:: clear_data()

      
      Clear all stored data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)
      :abstractmethod:


      
      Compute controller output while the gates are blocked.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_duty_ratios(ref)

      
      Extract duty ratios from the reference signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_enable()

      
      Evaluate the gate enable at the controller time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals from the model.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements from the model.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Convert stored lists to numpy arrays.
















      ..
          !! processed by numpydoc !!


   .. py:method:: run_control_loop(mdl)

      
      Run the default control loop, can be overridden.
















      ..
          !! processed by numpydoc !!


   .. py:method:: save(t, **signal_groups)

      
      Save a single timestep of data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_enable(enable)

      
      Set the gate enable.

      :param enable: Gate enable, constant or a function of time, evaluated at the sampling
                     instants. While the gates are blocked (False), the control system
                     computes its standby output, see `compute_standby_output`.
      :type enable: bool | Callable[[float], bool]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller internal states.
















      ..
          !! processed by numpydoc !!


.. py:class:: DiscreteComplexController(alpha_c, T_s)

   
   Direct discrete-time 2DOF complex-vector controller.

   This implements a 2DOF complex-vector controller designed directly in discrete time
   [#Awa2019]_. The controlled system is an integrator in coordinates rotating at the
   angular speed `w_c`. Its hold-equivalent model, including the computational delay of
   one sampling period, is::

       i(k + 1) = Phi*(i(k) + T_s*u(k))
       u(k + 1) = Phi*u_lim(k)

   where `i` is the feedback signal, `u` is the realized output, `u_lim` is the limited
   controller output, and ``Phi = exp(-1j*w_c*T_s)``. The design gives the
   reference-tracking dynamics ``i(k) = (1 - beta)/(z*(z - beta))*i_ref(k)``, where
   ``beta = exp(-alpha_c*T_s)``. The integral action compensates for disturbances.
   Since the design includes the delays, the angle of the output must not be advanced
   (`k_comp = 0` in the control system).

   :param alpha_c: Reference-tracking bandwidth (rad/s).
   :type alpha_c: float
   :param T_s: Sampling period (s).
   :type T_s: float

   .. rubric:: References

   .. [#Awa2019] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
      saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(i_ref, i)

      
      Compute the controller output.

      :param i_ref: Reference signal.
      :type i_ref: complex
      :param i: Feedback signal.
      :type i: complex

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: preset(u, w_c)

      
      Preset the integral state to the steady state of the output.

      In the steady state, the reference and feedback signals are zero, and the
      output is `u`, e.g., the back-EMF of a machine at zero current. The limited
      output of the previous sampling period is kept, since it has been realized.
      Hence, the next output equals `u` only if the previous one did, and otherwise
      it also compensates for their difference.

      :param u: Controller output.
      :type u: complex
      :param w_c: Angular speed of the coordinates (rad/s).
      :type w_c: float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u, w_c)

      
      Update the states.

      :param T_s: Sampling period (s), which must equal the design value.
      :type T_s: float
      :param u: Limited controller output.
      :type u: complex
      :param w_c: Angular speed of the coordinates (rad/s).
      :type w_c: float















      ..
          !! processed by numpydoc !!


.. py:class:: InverterNonlinearity(t_d = 0.0, u_th = 0.0, sign = np.sign)

   
   Duty-ratio error due to the dead time and the threshold voltages.

   The dead time blanks a leg for `t_d` after each switching instant, and the current
   direction determines the leg state during the blanking. Away from the duty-ratio
   limits, the resulting error is `sign(i)*t_d/(2*T_half)`, where `T_half` is the
   half carrier period. Pulses shorter than the dead time are suppressed, and clamped
   legs have no dead-time error. The threshold voltage `u_th` of the power devices,
   assumed equal for the switches and the diodes, causes the voltage drop
   `sign(i)*u_th` in each phase, i.e., the duty-ratio error `sign(i)*u_th/u_dc`,
   also in the clamped legs. The realized duty ratios are `d_abc - d_err`. This model
   is consistent with `motulator.common.model.VoltageSourceConverter`.

   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
   :param u_th: Threshold voltage (V) of the power devices, defaults to 0.
   :type u_th: float, optional
   :param sign: Current-direction function, defaults to `np.sign`. A smooth approximation may
                be used, matching the converter model.
   :type sign: Callable[[np.ndarray], np.ndarray], optional

   .. rubric:: Notes

   This switching-cycle averaged model assumes constant duty ratios and currents
   over a carrier period. It does not include transients due to changes in the
   switching commands.















   ..
       !! processed by numpydoc !!

.. py:class:: PIController(k_p, k_i, k_t = None, u_max = inf)

   
   2DOF PI controller.

   This implements a discrete-time 2DOF PI controller, whose continuous-time
   counterpart is::

       u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y) + u_ff

   where `u` is the controller output, `y_ref` is the reference signal, `y` is the
   feedback signal, `u_ff` is the feedforward signal, and `1/s` refers to integration.
   The standard PI controller is obtained by choosing ``k_t = k_p``. The integrator
   anti-windup is implemented based on the realized controller output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain, defaults to `k_p`.
   :type k_t: float, optional
   :param u_max: Maximum controller output, defaults to `inf`.
   :type u_max: float, optional

   .. rubric:: Notes

   This controller can be used, e.g., as a speed controller. In this case, `y`
   corresponds to the rotor angular speed `w_M` and `u` to the torque reference
   `tau_M_ref`.















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


   .. py:method:: update(T_s, u)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u: Realized (limited) controller output.
      :type u: float















      ..
          !! processed by numpydoc !!


.. py:class:: PWM(overmodulation = 'MPE', d_err = None, feedforward = True, k_pred = 1.5, d_min = 0.0, single_update = False)

   
   Duty ratios and realized voltage for three-phase space-vector PWM.

   This computes the duty ratios corresponding to standard space-vector PWM and
   overmodulation [#Hav1999]_. The realized voltage is reconstructed from the duty
   ratios and the measured DC-bus voltage, taking into account the computational delay
   of one sampling period. The angle of the voltage reference is not advanced here:
   the control systems compensate for the delays, if needed by the controller.

   Optionally, the duty-ratio error caused by the inverter nonlinearities (such as the
   dead time and the threshold voltages of the power devices) is modeled as a function
   of the phase currents, duty ratios, DC-bus voltage, and half carrier period. The
   realized voltage is corrected for this error using the measured currents, which
   correspond to the same instant as the realized voltage, and the duty ratios of the
   corresponding sampling periods.
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
   :param d_min: Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0
                 (no limit). The duty ratio applies to a half carrier period, so the shortest
                 pulse is `d_min` times the half carrier period when a leg starts or stops
                 switching and about twice that otherwise.
   :type d_min: float, optional
   :param single_update: Duty ratios are updated once per carrier period, i.e., the half carrier period
                         is `T_s/2`, defaults to False (twice per carrier period, the half carrier
                         period is `T_s`). Must match the system model.
   :type single_update: bool, optional

   .. rubric:: References

   .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
      triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
      https://doi.org/10.1109/28.777199

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
      :param w: Angular speed of the synchronous coordinates (rad/s), used for predicting
                the currents in the feedforward compensation of `d_err`.
      :type w: float

      :returns: * **d_abc** (*list[float]*) -- Duty ratios for the next sampling period.
                * **u_c_ab** (*complex*) -- Voltage (V) in stationary coordinates corresponding to the duty ratios
                  `d_abc`, i.e., the limited voltage reference with the duty-ratio error
                  compensation (if used) included.















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
      voltage is reconstructed from the duty ratios of the previous and the ongoing
      sampling periods and the measured DC-bus voltage, i.e., the variations of the
      DC-bus voltage and of the currents within the periods are not known. The
      duty-ratio error is evaluated with the half carrier period of the previous
      sampling period, which is assumed to equal that of the ongoing period. The
      measured currents are also stored for the feedforward compensation of the next
      duty ratios.

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


   .. py:method:: limit_pulses(d_abc, d_ref, d_err = None)

      
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
      :param d_err: Duty-ratio error as a function of the duty ratios, if it is compensated
                    for, in which case the realized duty ratios are `d_abc - d_err(d_abc)`.
                    Defaults to None, in which case the realized duty ratios are `d_abc`.
      :type d_err: Callable[[np.ndarray], np.ndarray], optional

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


   .. py:method:: update(T_s, d_abc)

      
      Store the next duty ratios and the ongoing half carrier period.
















      ..
          !! processed by numpydoc !!


.. py:class:: RateLimiter(rate_limit = inf)

   
   Rate limiter.

   :param rate_limit: Rate limit, defaults to `inf`.
   :type rate_limit: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: TimeSeries

   
   Container for control system's discrete-time data.

   Each signal group saved by `ControlSystem.save` becomes an attribute in
   `ControlSystem.post_process`, e.g., `fbk` and `ref`.















   ..
       !! processed by numpydoc !!

