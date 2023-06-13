:orphan:

:py:mod:`motulator.control._common`
===================================

.. py:module:: motulator.control._common

.. autoapi-nested-parse::

   Common control functions and classes.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control._common.PWM
   motulator.control._common.PICtrl
   motulator.control._common.SpeedCtrl
   motulator.control._common.ComplexPICtrl
   motulator.control._common.RateLimiter
   motulator.control._common.Clock
   motulator.control._common.Ctrl




.. py:class:: PWM(six_step=False)


   
   Duty ratios and realized voltage for three-phase PWM.

   This contains the computation of the duty ratios and the realized voltage.
   The realized voltage is computed based on the measured DC-bus voltage and
   the duty ratios. The digital delay effects are taken into account in the
   realized voltage, assuming the delay of a single sampling period. The total
   delay is 1.5 sampling periods due to the PWM (or ZOH) delay [#Bae2003]_.

   :param six_step: Enable six-step operation in overmodulation. The default is False.
   :type six_step: bool, optional

   .. attribute:: realized_voltage

      Realized voltage (V) in synchronous coordinates.

      :type: complex

   .. rubric:: References

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of
      full-digital synchronous frame current regulator of PWM AC drives," IEEE
      Trans. Ind. Appl., 2003, https://doi.org/10.1109/TIA.2003.810660















   ..
       !! processed by numpydoc !!
   .. py:method:: six_step_overmodulation(u_s_ref, u_dc)
      :staticmethod:

      
      Overmodulation up to six-step operation.

      This method modifies the angle of the voltage reference vector in the
      overmodulation region such that the six-step operation is reached
      [#Bol1997]_.

      :param u_s_ref: Reference voltage (V) in stator coordinates.
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **u_s_ref_mod** -- Reference voltage (V) in stator coordinates.
      :rtype: complex

      .. rubric:: References

      .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of
         SVM inverters in the overmodulation range," IEEE Trans. Ind. Appl.,
         1997, https://doi.org/10.1109/28.568019















      ..
          !! processed by numpydoc !!

   .. py:method:: duty_ratios(u_s_ref, u_dc)
      :staticmethod:

      
      Compute the duty ratios for three-phase PWM.

      This computes the duty ratios using a symmetrical suboscillation
      method. This method is identical to the standard space-vector PWM.

      :param u_s_ref: Voltage reference in stator coordinates (V).
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **d_abc** -- Duty ratios.
      :rtype: ndarray, shape (3,)















      ..
          !! processed by numpydoc !!


.. py:class:: PICtrl(k_p, k_i, k_t=None, u_max=np.inf)


   
   2DOF PI controller.

   This implements a discrete-time 2DOF PI controller, whose continuous-time
   counterpart is::

       u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y)

   where `u` is the controller output, `y_ref` is the reference signal, `y` is
   the feedback signal, and `1/s` refers to integration. The standard PI
   controller is obtained by choosing ``k_t = k_p``. The integrator anti-windup
   is implemented based on the realized controller output.

   .. rubric:: Notes

   This contoller can be used, e.g., as a speed controller. In this case, `y`
   corresponds to the rotor angular speed `w_M` and `u` to the torque reference
   `tau_M_ref`.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain. The default is `k_p`.
   :type k_t: float, optional
   :param u_max: Maximum controller output. The default is inf.
   :type u_max: float, optional

   .. attribute:: v

      Input disturbance estimate.

      :type: float

   .. attribute:: u_i

      Integral state.

      :type: float















   ..
       !! processed by numpydoc !!
   .. py:method:: output(y_ref, y)

      
      Compute the controller output.

      :param y_ref: Reference signal.
      :type y_ref: float
      :param y: Feedback signal.
      :type y: float

      :returns: **u** -- Controller output.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: update(T_s, u_lim)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_lim: Realized (limited) controller output. If the actuator does not
                    saturate, ``u_lim = u``.
      :type u_lim: float















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedCtrl(J, alpha_s, tau_M_max=np.inf)


   Bases: :py:obj:`PICtrl`

   
   2DOF PI speed controller.

   This provides an interface for a speed controller. The gains are initialized
   based on the desired closed-loop bandwidth and the rotor inertia estimate.

   :param J: Total inertia of the rotor (kgm²).
   :type J: float
   :param alpha_s: Closed-loop bandwidth (rad/s).
   :type alpha_s: float
   :param tau_M_max: Maximum motor torque (Nm). The default is inf.
   :type tau_M_max: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ComplexPICtrl(k_p, k_i, k_t=None)


   
   2DOF synchronous-frame complex-vector PI controller.

   This implements a discrete-time 2DOF synchronous-frame complex-vector PI
   controller, whose continuous-time counterpart is [#Bri2000]_::

       u = k_t*i_ref - k_p*i + (k_i + 1j*w*k_t)/s*(i_ref - i)

   where `u` is the controller output, `i_ref` is the reference signal, `i` is
   the feedback signal, `w` is the angular speed of synchronous coordinates,
   and `1/s` refers to integration. This algorithm is compatible with both real
   and complex signals. The 1DOF version is obtained by setting ``k_t = k_p``.
   The integrator anti-windup is implemented based on the realized controller
   output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain. The default is `k_p`.
   :type k_t: float, optional

   .. attribute:: v

      Input disturbance estimate.

      :type: complex

   .. attribute:: u_i

      Integral state.

      :type: complex

   .. rubric:: Notes

   This contoller can be used, e.g., as a current controller. In this case, `i`
   corresponds to the stator current and `u` to the stator voltage.

   .. rubric:: References

   .. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current
      regulators using complex vectors," IEEE Trans. Ind. Appl., 2000,
      https://doi.org/10.1109/28.845057















   ..
       !! processed by numpydoc !!
   .. py:method:: output(i_ref, i)

      
      Compute the controller output.

      :param i_ref: Reference signal.
      :type i_ref: complex
      :param i: Feedback signal.
      :type i: complex

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: update(T_s, u_lim, w)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_lim: Realized (limited) controller output. If the actuator does not
                    saturate, ``u_lim = u``.
      :type u_lim: complex
      :param w: Angular speed of the reference frame (rad/s).
      :type w: float















      ..
          !! processed by numpydoc !!


.. py:class:: RateLimiter(rate_limit=np.inf)


   
   Rate limiter.

   :param rate_limit: Rate limit. The default is inf.
   :type rate_limit: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Clock


   
   Digital clock.
















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s)

      
      Update the digital clock.

      :param T_s: Sampling period (s).
      :type T_s: float















      ..
          !! processed by numpydoc !!


.. py:class:: Ctrl


   
   Base class for the control system.
















   ..
       !! processed by numpydoc !!
   .. py:method:: save(data)

      
      Save the internal date of the control system.

      :param data: Contains the data to be saved.
      :type data: bunch or dict















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format.

      This method can be run after the simulation has been completed in order
      to simplify plotting and analysis of the stored data.















      ..
          !! processed by numpydoc !!


