:py:mod:`motulator.control.im_vector`
=====================================

.. py:module:: motulator.control.im_vector

.. autoapi-nested-parse::

   Vector control methods for induction motor drives.

   The algorithms are written based on the inverse-Γ model.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im_vector.InductionMotorVectorCtrlPars
   motulator.control.im_vector.InductionMotorVectorCtrl
   motulator.control.im_vector.CurrentRef
   motulator.control.im_vector.CurrentCtrl
   motulator.control.im_vector.SensorlessObserver
   motulator.control.im_vector.Observer




.. py:class:: InductionMotorVectorCtrlPars

   
   Vector control parameters for induction motor drives.
















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

      

   .. py:attribute:: alpha_o
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: g
      :value: 0.2

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: psi_R_nom
      :type: float
      :value: 0.9

      

   .. py:attribute:: u_dc_nom
      :type: float
      :value: 540

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.7

      

   .. py:attribute:: R_R
      :type: float
      :value: 2.1

      

   .. py:attribute:: L_sgm
      :type: float
      :value: 0.021

      

   .. py:attribute:: L_M
      :type: float
      :value: 0.224

      

   .. py:attribute:: n_p
      :type: int
      :value: 2

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      


.. py:class:: InductionMotorVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Vector control for an induction motor drive.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorControlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of an induction motor drive for getting the
                  feedback signals.
      :type mdl: InductionMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentRef(pars)

   
   Current reference calculation.

   This method includes field-weakenting operation based on the unlimited
   voltage reference feedback. The breakdown torque and current limits are
   taken into account.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorCtrlPars

   .. rubric:: Notes

   The field-weakening method and its tuning corresponds roughly to [Rb97b66e26a3e-1]_.

   .. rubric:: References

   .. [Rb97b66e26a3e-1] Hinkkanen, Luomi, "Braking scheme for vector-controlled induction
      motor drives equipped with diode rectifier without braking resistor,"
      IEEE Trans. Ind. Appl., 2006, https://doi.org/10.1109/TIA.2006.880852















   ..
       !! processed by numpydoc !!
   .. py:method:: output(tau_M_ref, psi_R)

      
      Compute the stator current reference.

      :param tau_M_ref: Torque reference.
      :type tau_M_ref: float
      :param psi_R: Rotor flux magnitude.
      :type psi_R: float

      :returns: * **i_s_ref** (*complex*) -- Stator current reference.
                * **tau_M** (*float*) -- Limited torque reference.















      ..
          !! processed by numpydoc !!

   .. py:method:: update(u_s_ref, u_dc)

      
      Field-weakening based on the unlimited reference voltage.

      :param u_s_ref: Unlimited stator voltage reference.
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage.
      :type u_dc: float















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentCtrl(pars)

   
   2DOF PI current controller.

   This controller corresponds to [Ra42ac92f4471-2]_. The continuous-time complex-vector
   design corresponding to (13) is used here. The rotor flux linkage is
   considered as a quasi-constant disturbance. This design could be
   equivalently presented as a 2DOF PI controller.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorCtrlPars

   .. rubric:: Notes

   This implementation does not take the magnetic saturation into account.

   .. rubric:: References

   .. [Ra42ac92f4471-2] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
      saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















   ..
       !! processed by numpydoc !!
   .. py:method:: output(i_s_ref, i_s)

      
      Compute the unlimited voltage reference.

      :param i_s_ref: Stator current reference.
      :type i_s_ref: complex
      :param i_s: Measured stator current.
      :type i_s: complex

      :returns: **u_s_ref** -- Unlimited voltage reference.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: update(u_s_ref_lim, w_s)

      
      Update the integral state.

      :param u_s_ref_lim: Limited voltage reference.
      :type u_s_ref_lim: complex
      :param w_s: Angular stator frequency.
      :type w_s: float















      ..
          !! processed by numpydoc !!


.. py:class:: SensorlessObserver(pars)

   
   Sensorless reduced-order flux observer.

   This observer corresponds to [Ra040d21f48f0-3]_. The observer gain decouples the
   electrical and mechanical dynamics and allows placing the poles of the
   corresponding linearized estimation error dynamics. This implementation
   operates in estimated rotor flux coordinates.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorCtrlPars

   .. rubric:: Notes

   This implementation corresponds to (26)-(30) in [Ra040d21f48f0-3]_ with the choice
   c = w_s**2 in (17). The closed-loop poles, cf. (40), can still be
   affected via the coefficient b > 0.

   .. rubric:: References

   .. [Ra040d21f48f0-3] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
      stator-resistance adaptation for speed-sensorless induction motor
      drives," IEEE Trans. Power Electron., 2010,
      https://doi.org/10.1109/TPEL.2009.2039650















   ..
       !! processed by numpydoc !!
   .. py:method:: output(u_s, i_s, *_)

      
      Compute the output.

      :param u_s: Stator voltage in estimated rotor flux coordinates.
      :type u_s: complex
      :param i_s: Stator current in estimated rotor flux coordinates.
      :type i_s: complex

      :returns: **w_s** -- Angular frequency of the rotor flux.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: update(i_s, w_s)

      
      Update the states for the next sampling period.
















      ..
          !! processed by numpydoc !!


.. py:class:: Observer(pars)

   
   Sensored reduced-order flux observer.

   This reduced-order flux observer [Rf20b6db00950-4]_ uses the measured rotor speed. The
   selected default gain allows smooth transition from the current model at
   zero speed to the (damped) voltage model at higher speeds.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorCtrlPars

   .. rubric:: Notes

   This implementation places the pole in synchronous coordinates at::

       s = -R_R/L_M - g*abs(w_m) - 1j*(w_s - w_m)

   .. rubric:: References

   .. [Rf20b6db00950-4] Verghese, Sanders, “Observers for flux estimation in induction
      machines,” IEEE Trans. Ind. Electron., 1988,
      https://doi.org/10.1109/41.3067















   ..
       !! processed by numpydoc !!
   .. py:method:: output(u_s, i_s, w_m)

      
      Compute the output of the observer.

      :param u_s: Stator voltage in estimated rotor flux coordinates.
      :type u_s: complex
      :param i_s: Stator current in estimated rotor flux coordinates.
      :type i_s: complex
      :param w_m: Rotor angular speed (in electrical rad/s)
      :type w_m: float

      :returns: **w_s** -- Angular frequency of the rotor flux.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: update(i_s, w_s)

      
      Update the states for the next sampling period.
















      ..
          !! processed by numpydoc !!


