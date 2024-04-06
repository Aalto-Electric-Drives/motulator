:orphan:

:py:mod:`motulator.control.sm._vector`
======================================

.. py:module:: motulator.control.sm._vector

.. autoapi-nested-parse::

   Current vector control methods for synchronous machine drives.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._vector.ModelPars
   motulator.control.sm._vector.VectorCtrl
   motulator.control.sm._vector.CurrentCtrl
   motulator.control.sm._vector.CurrentReferencePars
   motulator.control.sm._vector.CurrentReference




.. py:class:: ModelPars


   
   Model parameters of a synchronous machine.

   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: PM flux linkage (Vs).
   :type psi_f: float
   :param n_p: Number of pole pairs.
   :type n_p: int
   :param J: Moment of inertia (kgm²).
   :type J: float















   ..
       !! processed by numpydoc !!

.. py:class:: VectorCtrl(par, ref, T_s=0.00025, sensorless=True)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Vector control for synchronous machine drives.

   This class interconnects the subsystems of the control system and provides
   the interface to the solver.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ref: Reference generation parameters.
   :type ref: ReferencePars
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: current_ref

      Current reference generator.

      :type: CurrentReference

   .. attribute:: observer

      Flux and rotor position observer, used in the sensorless mode only.

      :type: Observer

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentCtrl

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentCtrl(par, alpha_c)


   Bases: :py:obj:`motulator.control._common.ComplexPICtrl`

   
   Current controller for synchronous machines.

   This provides an interface of a current controller for synchronous machines
   [#Awa2019a]_. The gains are initialized based on the desired closed-loop
   bandwidth and the inductances.

   :param par: Synchronous machine parameters, should contain `L_d` and `L_q` (H).
   :type par: ModelPars
   :param alpha_c: Closed-loop bandwidth (rad/s).
   :type alpha_c: float

   .. rubric:: References

   .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current
      control of saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















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


.. py:class:: CurrentReferencePars


   
   Parameters for reference generation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param psi_s_min: Minimum stator flux (Vs). The default is `psi_f`.
   :type psi_s_min: float, optional
   :param w_m_nom: Nominal rotor angular speed (electrical rad/s). Needed if `k_fw` is not
                   directly provided.
   :type w_m_nom: float, optional
   :param alpha_fw: Field-weakening bandwidth (rad/s). The default is 2*pi*20.
   :type alpha_fw: float, optional
   :param k_fw: Field-weakening gain. The default is `alpha_fw/(w_m_nom*par.L_d)`.
   :type k_fw: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional

   .. attribute:: i_sd_mtpa

      MTPA d-axis current (A) as a function of the torque (Nm).

      :type: callable

   .. attribute:: tau_M_lim

      Torque limit (Nm) as a function of the stator flux linkage (Vs). This
      limit merges the MTPV and current limits.

      :type: callable

   .. attribute:: i_sd_lim

      d-axis current limit (A) as a function of the stator flux linkage (Vs).
      This limit merges the MTPV and current limits.

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReference(par, ref)


   
   Current reference calculation.

   This method includes the MTPA locus and field-weakening operation based on
   the unlimited voltage reference feedback. The MTPV and current limits are
   taken into account. This resembles the method presented [#Bed2020]_.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ref: Reference generation parameters.
   :type ref: CurrentReferencePars

   .. rubric:: Notes

   Instead of the PI controller used in [#Bed2020]_, we use a simpler integral
   controller with a constant gain. The resulting operating-point-dependent
   closed-loop pole could be derived using (12) of the paper. Unlike in
   [#Bed2020]_, the MTPV limit is also included here by means of limiting the
   reference torque and the d-axis current reference.

   .. rubric:: References

   .. [#Bed2020] Bedetti, Calligaro, Petrella, "Analytical design and
      autotuning of adaptive flux-weakening voltage regulation loop in IPMSM
      drives with accurate torque regulation," IEEE Trans. Ind. Appl., 2020,
      https://doi.org/10.1109/TIA.2019.2942807















   ..
       !! processed by numpydoc !!
   .. py:method:: output(tau_M_ref, w_m, u_dc)

      
      Compute the stator current reference.

      :param tau_M_ref: Torque reference (Nm).
      :type tau_M_ref: float
      :param w_m: Rotor speed (electrical rad/s)
      :type w_m: float
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: * **i_s_ref** (*complex*) -- Stator current reference (A).
                * **tau_M_ref_lim** (*float*) -- Limited torque reference (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: update(T_s, tau_M_ref_lim, u_s_ref, u_dc)

      
      Field-weakening based on the unlimited reference voltage.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param tau_M_ref_lim: Limited torque reference (Nm).
      :type tau_M_ref_lim: float
      :param u_s_ref: Unlimited stator voltage reference (V).
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float















      ..
          !! processed by numpydoc !!


