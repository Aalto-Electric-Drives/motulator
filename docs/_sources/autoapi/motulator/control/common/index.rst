:py:mod:`motulator.control.common`
==================================

.. py:module:: motulator.control.common

.. autoapi-nested-parse::

   Common control functions and classes.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.common.PWM
   motulator.control.common.SpeedCtrl
   motulator.control.common.RateLimiter
   motulator.control.common.Ctrl




.. py:class:: PWM(pars)

   
   Duty ratio references and realized voltage for three-phase PWM.

   This contains the computation of the duty ratio references and the realized
   voltage. The digital delay effects are taken into account in the realized
   voltage.

   :param pars: Control parameters.
   :type pars: data object















   ..
       !! processed by numpydoc !!
   .. py:method:: six_step_overmodulation(u_s_ref, u_dc)
      :staticmethod:

      
      Overmodulation up to six-step operation.

      This method modifies the angle of the voltage reference vector in the
      overmodulation region such that the six-step operation is reached [R1e1de184dba8-1]_.

      :param u_s_ref: Voltage reference in stator coordinates.
      :type u_s_ref: complex

      :returns: **u_s_ref_mod** -- Voltage reference in stator coordinates.
      :rtype: complex

      .. rubric:: References

      .. [R1e1de184dba8-1] Bolognani, Zigliotto, "Novel digital continuous control of SVM
         inverters in the overmodulation range," IEEE Trans. Ind. Appl.,
         1997, https://doi.org/10.1109/28.568019















      ..
          !! processed by numpydoc !!

   .. py:method:: duty_ratios(u_s_ref, u_dc)
      :staticmethod:

      
      Compute the duty ratios for three-phase PWM.

      This computes the duty ratios using a symmetrical suboscillation
      method. This method is identical to the standard space-vector PWM.

      :param u_s_ref: Voltage reference in stator coordinates.
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage.
      :type u_dc: float

      :returns: **d_abc_ref** -- Duty ratio references.
      :rtype: ndarray, shape (3,)















      ..
          !! processed by numpydoc !!

   .. py:method:: __call__(u_ref, u_dc, theta, w)

      
      Compute the duty ratios and update the state.

      :param u_ref: Voltage reference in synchronous coordinates.
      :type u_ref: complex
      :param u_dc: DC-bus voltage.
      :type u_dc: float
      :param theta: Angle of synchronous coordinates.
      :type theta: float
      :param w: Angular frequency of synchronous coordinates.
      :type w: float

      :returns: **d_abc_ref** -- Duty ratio references.
      :rtype: ndarray, shape (3,)















      ..
          !! processed by numpydoc !!

   .. py:method:: output(u_ref, u_dc, theta, w)

      
      Compute the duty ratio limited voltage reference.
















      ..
          !! processed by numpydoc !!

   .. py:method:: update(u_ref_lim)

      
      Update the voltage estimate for the next sampling instant.

      :param u_ref_lim: Limited voltage reference in synchronous coordinates.
      :type u_ref_lim: complex















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedCtrl(pars)

   
   2DOF PI speed controller.

   This controller is implemented using the disturbance-observer structure.
   The controller is mathematically identical to the 2DOF PI speed controller.

   :param pars: Control parameters.
   :type pars: data object















   ..
       !! processed by numpydoc !!
   .. py:method:: output(w_M_ref, w_M)

      
      Compute the torque reference and the load torque estimate.

      :param w_M_ref: Rotor speed reference (in mechanical rad/s).
      :type w_M_ref: float
      :param w_M: Rotor speed (in mechanical rad/s).
      :type w_M: float

      :returns: **tau_M_ref** -- Torque reference.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: update(tau_M_ref_lim)

      
      Update the integral state.

      :param tau_M_ref_lim: Realized (limited) torque reference.
      :type tau_M_ref_lim: float















      ..
          !! processed by numpydoc !!


.. py:class:: RateLimiter(pars)

   
   Rate limiter.

   :param pars: Control parameters.
   :type pars: data object















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(u)

      
      Limit the input signal.

      :param u: Input signal.
      :type u: float

      :returns: **y** -- Rate-limited output signal.
      :rtype: float

      .. rubric:: Notes

      In this implementation, the falling rate limit equals the (negative)
      rising rate limit. If needed, these limits can be separated with minor
      modifications in the class.















      ..
          !! processed by numpydoc !!


.. py:class:: Ctrl

   
   Base class for main control loops.
















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)
      :abstractmethod:

      
      Run the main control loop.

      The main control loop is callable that returns the sampling
      period `T_s` (float)  and the duty ratio references `d_abc_ref`
      (ndarray, shape (3,)) for the next sampling period.

      :param mdl: System model containing methods for getting the feedback signals.
      :type mdl: Model















      ..
          !! processed by numpydoc !!

   .. py:method:: update_clock(T_s)

      
      Update the digital clock.

      :param T_s: Sampling period.
      :type T_s: float















      ..
          !! processed by numpydoc !!

   .. py:method:: save(data)

      
      Save the internal controller data.

      :param data: Contains the data to be saved.
      :type data: bunch or dict















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format.

      This can be run after the simulation has been completed in order to
      simplify plotting and analysis of the stored data.















      ..
          !! processed by numpydoc !!


