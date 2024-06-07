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

   motulator.common.control.Ctrl
   motulator.common.control.ComplexPICtrl
   motulator.common.control.PICtrl
   motulator.common.control.PWM
   motulator.common.control.RateLimiter


Package Contents
----------------

.. py:class:: Ctrl(T_s)

   Bases: :py:obj:`abc.ABC`


   
   Base class for control systems.

   This base class provides typical functionalities for control systems. It
   can be used as a template for implementing custom controllers. An instance
   of this class can be called as a function. When called, it runs the main
   control loop.

   :param T_s: Sampling period (s).
   :type T_s: float

   .. attribute:: clock

      Digital clock.

      :type: Clock

   .. attribute:: data

      Saved simulation data.

      :type: SimpleNamespace

   .. attribute:: pwm

      Pulse-width modulator.

      :type: PWM















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)
      :abstractmethod:


      
      Get the feedback signals.

      :param mdl: Continuous-time system model.
      :type mdl: Model

      :returns: **fbk** -- Feedback signals.
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)
      :abstractmethod:


      
      Compute the controller outputs.

      :param fbk: Feedback signals.
      :type fbk: SimpleNamespace

      :returns: **ref** --

                References, containing at least the following fields:

                    T_s : float
                        Next sampling period (s).
                    d_abc : ndarray, shape (3,)
                        Duty ratios.
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)
      :abstractmethod:


      
      Update the states.

      :param fbk: Feedback signals.
      :type fbk: SimpleNamespace
      :param ref: Reference signals.
      :type ref: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: save(**kwargs)

      
      Save the data of the control system.

      Each keyword represents a data category, and its value (a
      SimpleNamespace) contains the data for that category.

      :param \*\*kwargs: One or more keyword arguments where the key is the name and the
                         value is a SimpleNamespace containing the data to be saved.
      :type \*\*kwargs: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Transform the lists to the ndarray format.

      This method can be run after the simulation has been completed in order
      to simplify plotting and analysis of the stored data.















      ..
          !! processed by numpydoc !!


   .. py:method:: main(mdl)

      
      Main control loop.

      This method runs the main control loop, having the following structure:

      1. Get the feedback signals. This step may contain first getting the
         measurements and then optionally computing the observer outputs.
      2. Compute the reference signals (controller outputs) based on the
         feedback signals.
      3. Update the control system states for the next sampling instant.
      4. Save the feedback signals and the reference signals.
      5. Return the sampling period `T_s` and the duty ratios `d_abc` for the
         carrier comparison.

      :param mdl: Continuous-time system model.
      :type mdl: Model

      :returns: * **T_s** (*float*) -- Sampling period (s).
                * **d_abc** (*ndarray, shape (3,)*) -- Duty ratios.















      ..
          !! processed by numpydoc !!


.. py:class:: ComplexPICtrl(k_p, k_i, k_t=None)

   
   2DOF synchronous-frame complex-vector PI controller.

   This implements a discrete-time 2DOF synchronous-frame complex-vector PI
   controller, whose continuous-time counterpart is [#Bri2000]_::

       u = k_t*ref_i - k_p*i + (k_i + 1j*w*k_t)/s*(ref_i - i)

   where `u` is the controller output, `ref_i` is the reference signal, `i` is
   the feedback signal, `w` is the angular speed of synchronous coordinates,
   and `1/s` refers to integration. This algorithm is compatible with both
   real and complex signals. The 1DOF version is obtained by setting
   ``k_t = k_p``. The integrator anti-windup is implemented based on the
   realized controller output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain. The default is `k_p`.
   :type k_t: float, optional

   .. rubric:: Notes

   This controller can be used, e.g., as a current controller. In this case,
   `i` corresponds to the stator current and `u` to the stator voltage.

   .. rubric:: References

   .. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current
      regulators using complex vectors," IEEE Trans. Ind. Appl., 2000,
      https://doi.org/10.1109/28.845057















   ..
       !! processed by numpydoc !!

   .. py:method:: output(ref_i, i)

      
      Compute the controller output.

      :param ref_i: Reference signal.
      :type ref_i: complex
      :param i: Feedback signal.
      :type i: complex

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u, w)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u: Realized (limited) controller output.
      :type u: complex
      :param w: Angular speed of the reference frame (rad/s).
      :type w: float















      ..
          !! processed by numpydoc !!


.. py:class:: PICtrl(k_p, k_i, k_t=None, max_u=np.inf)

   
   2DOF PI controller.

   This implements a discrete-time 2DOF PI controller, whose continuous-time
   counterpart is::

       u = k_t*ref_y - k_p*y + (k_i/s)*(ref_y - y)

   where `u` is the controller output, `y_ref` is the reference signal, `y` is
   the feedback signal, and `1/s` refers to integration. The standard PI
   controller is obtained by choosing ``k_t = k_p``. The integrator
   anti-windup is implemented based on the realized controller output.

   .. rubric:: Notes

   This controller can be used, e.g., as a speed controller. In this case, `y`
   corresponds to the rotor angular speed `w_M` and `u` to the torque
   reference `ref_tau_M`.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain. The default is `k_p`.
   :type k_t: float, optional
   :param max_u: Maximum controller output. The default is `inf`.
   :type max_u: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: output(ref_y, y)

      
      Compute the controller output.

      :param ref_y: Reference signal.
      :type ref_y: float
      :param y: Feedback signal.
      :type y: float

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


.. py:class:: PWM(six_step=False, k_comp=1.5)

   
   Duty ratios and realized voltage for three-phase space-vector PWM.

   This computes the duty ratios corresponding to standard space-vector PWM
   and minimum-amplitude-error overmodulation [#Hav1999]_. The realized
   voltage is computed based on the measured DC-bus voltage and the duty
   ratios. The digital delay effects are taken into account in the realized
   voltage [#Bae2003]_.

   :param six_step: Enable six-step operation in overmodulation. The default is False.
   :type six_step: bool, optional
   :param k_comp: Compensation factor for the delay effect on the voltage vector angle.
                  The default is 1.5.
   :type k_comp: float, optional

   .. rubric:: References

   .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation
      characteristics of triangle intersection PWM methods," IEEE Trans. Ind.
      Appl., 1999, https://doi.org/10.1109/28.777199

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of
      full-digital synchronous frame current regulator of PWM AC drives," IEEE
      Trans. Ind. Appl., 2003, https://doi.org/10.1109/TIA.2003.810660















   ..
       !! processed by numpydoc !!

   .. py:method:: six_step_overmodulation(ref_u_cs, u_dc)
      :staticmethod:


      
      Overmodulation up to six-step operation.

      This method modifies the angle of the voltage reference vector in the
      overmodulation region such that the six-step operation is reached
      [#Bol1997]_.

      :param ref_u_cs: Converter voltage reference (V) in stationary coordinates.
      :type ref_u_cs: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **ref_u_cs** -- Modified converter voltage reference (V) in stationary coordinates.
      :rtype: complex

      .. rubric:: References

      .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control
         of SVM inverters in the overmodulation range," IEEE Trans. Ind.
         Appl., 1997, https://doi.org/10.1109/28.568019















      ..
          !! processed by numpydoc !!


   .. py:method:: duty_ratios(ref_u_cs, u_dc)
      :staticmethod:


      
      Compute the duty ratios for three-phase space-vector PWM.

      This computes the duty ratios corresponding to standard space-vector
      PWM and minimum-amplitude-error overmodulation [#Hav1999]_.

      :param ref_u_cs: Converter voltage reference (V) in stationary coordinates.
      :type ref_u_cs: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **d_abc** -- Duty ratios.
      :rtype: ndarray, shape (3,)















      ..
          !! processed by numpydoc !!


   .. py:method:: output(T_s, ref_u_cs, u_dc, w)

      
      Compute the duty ratios and the limited voltage reference.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param ref_u_cs: Converter voltage reference (V) in stationary coordinates.
      :type ref_u_cs: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float
      :param w: Angular speed of synchronous coordinates (rad/s).
      :type w: float

      :returns: * **d_abc** (*ndarray, shape (3,)*) -- Duty ratios for the next sampling period.
                * **u_cs** (*complex*) -- Limited voltage reference (V) in stationary coordinates.















      ..
          !! processed by numpydoc !!


   .. py:method:: get_realized_voltage()

      
      Get the realized voltage.

      :returns: **realized_voltage** -- Realized converter voltage (V) in stationary coordinates. The
                effect of the digital delays on the angle are compensated for.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: update(u_cs)

      
      Update the realized voltage.
















      ..
          !! processed by numpydoc !!


.. py:class:: RateLimiter(rate_limit=np.inf)

   
   Rate limiter.

   :param rate_limit: Rate limit. The default is inf.
   :type rate_limit: float, optional















   ..
       !! processed by numpydoc !!

