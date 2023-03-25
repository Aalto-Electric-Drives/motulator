:py:mod:`motulator.control.sm_vector`
=====================================

.. py:module:: motulator.control.sm_vector

.. autoapi-nested-parse::

   Current vector control for synchronous motor drives.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm_vector.SynchronousMotorVectorCtrlPars
   motulator.control.sm_vector.SynchronousMotorVectorCtrl
   motulator.control.sm_vector.CurrentCtrl
   motulator.control.sm_vector.CurrentRef
   motulator.control.sm_vector.SensorlessObserver




.. py:class:: SynchronousMotorVectorCtrlPars

   
   Vector control parameters for synchronous motors.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: sensorless
      :type: bool
      :value: True

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: alpha_c
      :type: float

      

   .. py:attribute:: alpha_fw
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: psi_s_min
      :type: float

      

   .. py:attribute:: k_u
      :type: float
      :value: 0.95

      

   .. py:attribute:: w_nom
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.6

      

   .. py:attribute:: L_d
      :type: float
      :value: 0.036

      

   .. py:attribute:: L_q
      :type: float
      :value: 0.051

      

   .. py:attribute:: psi_f
      :type: float
      :value: 0.545

      

   .. py:attribute:: n_p
      :type: int
      :value: 3

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      

   .. py:attribute:: w_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.2

      


.. py:class:: SynchronousMotorVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Vector control for a synchronous motor drive.

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


.. py:class:: CurrentCtrl(pars)

   
   2DOF PI current controller.

   This controller corresponds to [Ra42ac92f4471-1]_. The continuous-time complex-vector
   design corresponding to (13) is used here. This design could be
   equivalently presented as a 2DOF PI controller.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars (or its subset)

   .. rubric:: Notes

   For better performance at high speeds with low sampling frequencies, the
   discrete-time design in (18) is recommended. This implementation does not
   take the magnetic saturation into account.

   .. rubric:: References

   .. [Ra42ac92f4471-1] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
      saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















   ..
       !! processed by numpydoc !!
   .. py:method:: output(i_s_ref, i_s)

      
      Compute the unlimited voltage reference.

      :param i_s_ref: Current reference.
      :type i_s_ref: complex
      :param i_s: Measured current.
      :type i_s: complex

      :returns: **u_s_ref** -- Unlimited voltage reference.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: update(u_s_ref_lim, w_m)

      
      Update the integral state.

      :param u_s_ref_lim: Limited voltage reference.
      :type u_s_ref_lim: complex
      :param w_m: Angular rotor speed.
      :type w_m: float















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentRef(pars)

   
   Current reference calculation.

   This method includes the MTPA locus and field-weakenting operation based on
   the unlimited voltage reference feedback. The MTPV and current limits are
   taken into account. This resembles the method presented [Rb97b66e26a3e-2]_.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars (or its subset)

   .. rubric:: Notes

   Instead of the PI controller used in [Rb97b66e26a3e-2]_, we use a simpler integral
   controller with a constant gain. The resulting operating-point-dependent
   closed-loop pole could be derived using (12) of the paper. Unlike in [Rb97b66e26a3e-2]_,
   the MTPV limit is also included here by means of limiting the reference
   torque and the d-axis current reference.

   .. rubric:: References

   .. [Rb97b66e26a3e-2] Bedetti, Calligaro, Petrella, "Analytical design and autotuning of
      adaptive flux-weakening voltage regulation loop in IPMSM drives with
      accurate torque regulation," IEEE Trans. Ind. Appl., 2020,
      https://doi.org/10.1109/TIA.2019.2942807















   ..
       !! processed by numpydoc !!
   .. py:method:: output(tau_M_ref, w_m, u_dc)

      
      Compute the stator current reference.

      :param tau_M_ref: Torque reference.
      :type tau_M_ref: float
      :param w_m: Rotor speed (in electrical rad/s)
      :type w_m: float
      :param u_dc: DC-bus voltage.
      :type u_dc: float

      :returns: * **i_s_ref** (*complex*) -- Stator current reference.
                * **tau_M_ref_lim** (*float*) -- Limited torque reference.















      ..
          !! processed by numpydoc !!

   .. py:method:: update(tau_M_ref_lim, u_s_ref, u_dc)

      
      Field-weakening based on the unlimited reference voltage.

      :param tau_M_ref_lim: Limited torque reference.
      :type tau_M_ref_lim: float
      :param u_s_ref: Unlimited stator voltage reference.
      :type u_s_ref: complex
      :param u_dc: float.
      :type u_dc: DC-bus voltage.















      ..
          !! processed by numpydoc !!


.. py:class:: SensorlessObserver(pars)

   
   Sensorless observer.

   This observer corresponds to [Ra040d21f48f0-3]_. The observer gain decouples the
   electrical and mechanical dynamics and allows placing the poles of the
   corresponding linearized estimation error dynamics. This implementation
   operates in estimated rotor coordinates.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars (or its subset)

   .. rubric:: References

   .. [Ra040d21f48f0-3] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis,"
      IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753















   ..
       !! processed by numpydoc !!
   .. py:method:: update(u_s, i_s, *_)

      
      Update the states for the next sampling period.

      :param u_s: Stator voltage in estimated rotor coordinates.
      :type u_s: complex
      :param i_s: Stator current in estimated rotor coordinates.
      :type i_s: complex















      ..
          !! processed by numpydoc !!


