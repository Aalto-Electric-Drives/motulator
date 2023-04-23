:py:mod:`motulator.model.sm`
============================

.. py:module:: motulator.model.sm

.. autoapi-nested-parse::

   Continuous-time models for synchronous motors.

   The motor models can be parametrized to represent permanent-magnet synchronous
   motors and synchronous reluctance motors. Peak-valued complex space vectors are
   used.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.sm.SynchronousMotor
   motulator.model.sm.SynchronousMotorSaturated




.. py:class:: SynchronousMotor(n_p, R_s, L_d, L_q, psi_f)

   
   Synchronous motor model.

   This models a synchronous motor in rotor coordinates. The stator flux
   linkage and the electrical angle of the rotor are the state variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ohm).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: PM-flux linkage (Vs).
   :type psi_f: float















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Compute the stator current.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: **i_s** -- Stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_s)

      
      Magnetic model.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_s, u_s, w_M)

      
      Compute the state derivative.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex
      :param u_s: Stator voltage (V).
      :type u_s: complex
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_s, dtheta_m0]
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).

      .. rubric:: Notes

      In addition to the state derivative, this method also returns the output
      signals (stator current `i_ss` and torque `tau_M`) needed for
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


.. py:class:: SynchronousMotorSaturated(n_p, R_s, current, psi_s0=0j)

   Bases: :py:obj:`SynchronousMotor`

   
   Model of a saturated synchronous motor.

   This overrides the linear magnetics model of the SynchronousMotor class
   with a generic saturation model::

       i_s = i_s(psi_s)

   The saturation model could be an analytical function or a look-up table.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ohm).
   :type R_s: float
   :param current: Function that computes the stator current `i_s` as a function of the
                   stator flux linkage `psi_s`.
   :type current: callable
   :param psi_s0: Initial value of the stator flux linkage (Vs). For PM motors, this
                  should be solved from the the saturation model. The default is 0j.
   :type psi_s0: complex, optional















   ..
       !! processed by numpydoc !!

