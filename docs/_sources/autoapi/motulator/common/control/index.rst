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


   .. py:method:: get_duty_ratios(ref)

      
      Extract duty ratios from the reference signals.
















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


   .. py:method:: update(ref, fbk)

      
      Update controller internal states.
















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


.. py:class:: PWM(k_comp = 1.5, u_c0_ab = 0j, overmodulation = 'MPE', d_err = None, feedforward = True, k_pred = 1.5)

   
   Duty ratios and realized voltage for three-phase space-vector PWM.

   This computes the duty ratios corresponding to standard space-vector PWM and
   overmodulation [#Hav1999]_. The realized voltage is computed based on the measured
   DC-bus voltage and the duty ratios. The digital delay effects are taken into account
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

   :param k_comp: Compensation factor for the angular delay effect on the voltage reference,
                  defaults to 1.5. Use 0 if the controller compensates for the delays itself,
                  e.g., in direct discrete-time designs.
   :type k_comp: float, optional
   :param u_c0_ab: Initial voltage (V) in stationary coordinates. This is used to compute the
                   realized voltage, defaults to 0.
   :type u_c0_ab: float, optional
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

   .. rubric:: References

   .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
      triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
      https://doi.org/10.1109/28.777199

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
      synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
      2003, https://doi.org/10.1109/TIA.2003.810660















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(T_s, u_c_ref_ab, u_dc, w)

      
      Compute the duty ratios and the limited voltage reference.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float
      :param w: Angular speed of synchronous coordinates (rad/s).
      :type w: float

      :returns: * **d_abc** (*list[float]*) -- Duty ratios for the next sampling period.
                * **u_c_ab** (*complex*) -- Limited voltage reference (V) in stationary coordinates.















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
      measured currents are also stored for the feedforward compensation of the next
      duty ratios.

      :param i_c_ab: Measured converter current (A) in stationary coordinates.
      :type i_c_ab: complex
      :param u_dc: Measured DC-bus voltage (V).
      :type u_dc: float
      :param average: If True (default), the average voltage of the previous and the ongoing
                      sampling periods is returned, which represents the voltage at the sampling
                      instant, as needed in continuous-time designs. If False, the voltage of the
                      ongoing sampling period is returned, as needed in hold-equivalent models of
                      direct discrete-time designs.
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


   .. py:method:: update(u_c_ab, d_abc)

      
      Update the realized voltage.
















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
















   ..
       !! processed by numpydoc !!

