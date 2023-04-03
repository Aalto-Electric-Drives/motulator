:py:mod:`motulator.model.im`
============================

.. py:module:: motulator.model.im

.. autoapi-nested-parse::

   Continuous-time models for induction motors.

   Peak-valued complex space vectors are used. The space vector models are
   implemented in stator coordinates. The default values correspond to a 2.2-kW
   induction motor.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.im.InductionMotor
   motulator.model.im.InductionMotorSaturated
   motulator.model.im.InductionMotorInvGamma




.. py:class:: InductionMotor(n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_s=0.245)

   
   Γ-equivalent model of an induction motor.

   An induction motor is modeled using the Γ-equivalent model [R743146ac54e0-1]_. The model
   is implemented in stator coordinates. The flux linkages are used as state
   variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_r: Rotor resistance.
   :type R_r: float
   :param L_ell: Leakage inductance.
   :type L_ell: float
   :param L_s: Stator inductance.
   :type L_s: float

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic
   saturation model in a staightforward manner. If the magnetic saturation is
   omitted, the Γ model is mathematically identical to the inverse-Γ and T
   models [R743146ac54e0-1]_.

   .. rubric:: References

   .. [R743146ac54e0-1] Slemon, "Modelling of induction machines for electric drives," IEEE
      Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Compute the stator and rotor currents.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current.
                * **i_rs** (*complex*) -- Rotor current.















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_ss, psi_rs)

      
      Magnetic model.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current.
                * **i_rs** (*complex*) -- Rotor current.
                * **tau_M** (*float*) -- Electromagnetic torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_ss, psi_rs, u_ss, w_M)

      
      Compute the state derivatives.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex
      :param u_ss: Stator voltage.
      :type u_ss: complex
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_ss, dpsi_rs]
                * **i_ss** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.

      .. rubric:: Notes

      In addition to the state derivatives, this method also returns the
      output signals (stator current `i_ss` and torque `tau_M`) needed for
      interconnection with other subsystems. This avoids overlapping
      computation in simulation.















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the phase currents at the end of the sampling period.

      :returns: **i_s_abc** -- Phase currents.
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorSaturated(n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_su=0.34, beta=0.84, S=7)

   Bases: :py:obj:`InductionMotor`

   
   Γ-equivalent model of an induction motor model with main-flux saturation.

   This extends the InductionMotor class with a main-flux magnetic saturation
   model [R31fc15c1345a-2]_::

       L_s(psi_ss) = L_su/(1 + (beta*abs(psi_ss)**S)

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_r: Rotor resistance.
   :type R_r: float
   :param L_ell: Leakage inductance.
   :type L_ell: float
   :param L_su: Unsaturated stator inductance.
   :type L_su: float
   :param beta: Positive coefficient.
   :type beta: float
   :param S: Positive coefficient.
   :type S: float

   .. rubric:: References

   .. [R31fc15c1345a-2] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
      induction motor drives," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2190818















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorInvGamma(n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224)

   Bases: :py:obj:`InductionMotor`

   
   Inverse-Γ model of an induction motor.

   This extends the InductionMotor class (based on the Γ model) by providing
   an interface for the inverse-Γ model parameters. Linear magnetics are
   assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_R: Rotor resistance.
   :type R_R: float
   :param L_sgm: Leakage inductance.
   :type L_sgm: float
   :param L_M: Magnetizing inductance.
   :type L_M: float















   ..
       !! processed by numpydoc !!

