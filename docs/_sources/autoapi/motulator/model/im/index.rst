:py:mod:`motulator.model.im`
============================

.. py:module:: motulator.model.im

.. autoapi-nested-parse::

   Continuous-time models for induction motors.

   Peak-valued complex space vectors are used. The space vector models are
   implemented in stator coordinates.

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




.. py:class:: InductionMotor(n_p, R_s, R_r, L_ell, L_s)

   
   Γ-equivalent model of an induction motor.

   An induction motor is modeled using the Γ-equivalent model [R743146ac54e0-1]_. The model
   is implemented in stator coordinates. The flux linkages are used as state
   variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ohm).
   :type R_s: float
   :param R_r: Rotor resistance (Ohm).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H).
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

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current (A).
                * **i_rs** (*complex*) -- Rotor current (A).















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_ss, psi_rs)

      
      Magnetic model.

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current (A).
                * **i_rs** (*complex*) -- Rotor current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_ss, psi_rs, u_ss, w_M)

      
      Compute the state derivatives.

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex
      :param u_ss: Stator voltage (V).
      :type u_ss: complex
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_ss, dpsi_rs]
                * **i_ss** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).

      .. rubric:: Notes

      In addition to the state derivatives, this method also returns the
      output signals (stator current `i_ss` and torque `tau_M`) needed for
      interconnection with other subsystems. This avoids overlapping
      computation in simulation.















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the phase currents at the end of the sampling period.

      :returns: **i_s_abc** -- Phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorSaturated(n_p, R_s, R_r, L_ell, L_s)

   Bases: :py:obj:`InductionMotor`

   
   Γ-equivalent model of an induction motor model with main-flux saturation.

   This extends the InductionMotor class with a main-flux magnetic saturation
   model::

       L_s = L_s(abs(psi_ss))

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ohm).
   :type R_s: float
   :param R_r: Rotor resistance (Ohm).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H) as a function of the stator-flux magnitude.
   :type L_s: callable















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorInvGamma(n_p, R_s, R_R, L_sgm, L_M)

   Bases: :py:obj:`InductionMotor`

   
   Inverse-Γ model of an induction motor.

   This extends the InductionMotor class (based on the Γ model) by providing
   an interface for the inverse-Γ model parameters. Linear magnetics are
   assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ohm).
   :type R_s: float
   :param R_R: Rotor resistance (Ohm).
   :type R_R: float
   :param L_sgm: Leakage inductance (H).
   :type L_sgm: float
   :param L_M: Magnetizing inductance (H).
   :type L_M: float















   ..
       !! processed by numpydoc !!

