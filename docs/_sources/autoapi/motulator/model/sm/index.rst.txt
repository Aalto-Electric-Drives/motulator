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
   motulator.model.sm.SynchronousMotorSaturatedLUT




.. py:class:: SynchronousMotor(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545, mech=None)

   
   Synchronous motor model.

   This models a synchronous motor in rotor coordinates. The stator flux
   linkage is the state variable. The default values correspond to a 2.2-kW
   permanent-magnet synchronous motor.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param L_d: d-axis inductance.
   :type L_d: float
   :param L_q: q-axis inductance.
   :type L_q: float
   :param psi_f: PM-flux linkage.
   :type psi_f: float
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Compute the stator current.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex

      :returns: **i_s** -- Stator current.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_s)

      
      Magnetic model.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex

      :returns: * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_s, u_s, w_M)

      
      Compute the state derivative.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex
      :param u_s: Stator voltage.
      :type u_s: complex
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float

      :returns: * **dpsi_s** (*complex list*) -- Time derivative of the stator flux linkage.
                * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.

      .. rubric:: Notes

      In addition to the state derivative, this method also returns the
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


.. py:class:: SynchronousMotorSaturated(n_p=2, R_s=0.54, i_f=0, a_d0=17.4, a_q0=52.1, a_dd=373.0, a_qq=658.0, a_dq=1120.0, S=5, T=1, U=1, V=0, mech=None)

   Bases: :py:obj:`SynchronousMotor`

   
   Model of a saturated synchronous motor.

   This extends the SynchronousMotor class with an analytical saturation
   model [R83c7849ec2d6-1]_, [R83c7849ec2d6-2]_. The permanent magnets (PMs) are assumed to be along the
   d-axis. The default values correspond to a 6.7-kW synchronous reluctance
   motor.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param i_f: Constant current corresponding to the magnetomotive force (MMF) of PMs.
               In the magnetically linear case, `i_f = psi_f/L_d`.
   :type i_f: float
   :param a_d0: Nonnegative parameter of the saturation model. In the magnetically
                linear case, `a_d0 = 1/L_d`.
   :type a_d0: float
   :param a_q0: Nonnegative parameter of the saturation model. In the magnetically
                linear case, `a_q0 = 1/L_q`.
   :type a_q0: float
   :param a_dd: Nonnegative constant defining the d-axis self-saturation together with
                `S`. In the magnetically linear case, `a_dd = 0`.
   :type a_dd: float
   :param a_qq: Nonnegative constant defining the q-axis self-saturation together with
                `T`. In the magnetically linear case, `a_qq = 0`.
   :type a_qq: float
   :param a_dq: Nonnegative constant defining the cross-saturation together with `U`
                and `V`. In the magnetically linear case, `a_dq = 0`.
   :type a_dq: float
   :param S: Nonnegative constant defining the d-axis self-saturation.
   :type S: float
   :param T: Nonnegative constant defining the q-axis self-saturation.
   :type T: float
   :param U: Nonnegative constant defining the cross-saturation.
   :type U: float
   :param V: Nonnegative constant defining the cross-saturation.
   :type V: float
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics

   .. rubric:: Notes

   The magnetomotive force (MMF) of the PMs is modeled using constant current
   source `i_f` on the d-axis [R83c7849ec2d6-3]_. Correspondingly, this approach assumes
   that the MMFs of the d-axis current and of the PMs are in series. This
   model cannot capture the desaturation phenomenon of thin iron ribs [R83c7849ec2d6-4]_.
   For such motors, look-up tables can be used.

   .. rubric:: References

   .. [R83c7849ec2d6-1] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi,
      “Sensorless self-commissioning of synchronous reluctance motors at
      standstill without rotor locking, ”IEEE Trans. Ind. Appl., 2017,
      https://doi.org/10.1109/TIA.2016.2644624

   .. [R83c7849ec2d6-2] Awan, Song, Saarakkala, Hinkkanen, "Optimal torque control of
      saturated synchronous motors: plug-and-play method," IEEE Trans. Ind.
      Appl., 2018, https://doi.org/10.1109/TIA.2018.2862410

   .. [R83c7849ec2d6-3] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous
      motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986,
      https://doi.org/10.1109/TIA.1986.4504786

   .. [R83c7849ec2d6-4] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate
      modeling and performance analysis of IPM-PMASR motors," IEEE Trans. Ind.
      Appl., 2009, https://doi.org/10.1109/TIA.2008.2009493















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorSaturatedLUT(n_p=2, R_s=0.2, psi_s_data=None, i_s_data=None, mech=None)

   Bases: :py:obj:`SynchronousMotor`

   
   Look-up-table-based model of a saturated synchronous motor.

   This extends the SynchronousMotor class with a saturation model, where the
   stator current depends on the stator flux linkage. The coordinates assume
   the PMSM convention, i.e., that the PM flux is along the d-axis.
   Unstructured flux map data can be used.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param psi_s_data: Stator flux data points for creating the interpolant.
   :type psi_s_data: ndarray of complex
   :param i_s_data: Stator current data values for creating the interpolant.
   :type i_s_data: ndarray of complex
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


