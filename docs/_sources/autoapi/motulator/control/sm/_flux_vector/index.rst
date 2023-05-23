:orphan:

:py:mod:`motulator.control.sm._flux_vector`
===========================================

.. py:module:: motulator.control.sm._flux_vector

.. autoapi-nested-parse::

   Flux-vector control of synchronous machine drives.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._flux_vector.FluxVectorCtrl
   motulator.control.sm._flux_vector.FluxTorqueReferencePars
   motulator.control.sm._flux_vector.FluxTorqueReference




.. py:class:: FluxVectorCtrl(par, ref, alpha_psi=2 * np.pi * 100, alpha_tau=2 * np.pi * 200, T_s=0.00025, sensorless=True)

   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Flux-vector control of synchronous machine drives.

   This class implements a variant of stator-flux-vector control [#Pel2009]_.
   Rotor coordinates as well as decoupling between the stator flux and torque
   channels are used according to [#Awa2019]_. Here, the stator flux magnitude
   and the electromagnetic torque are selected as controllable variables.

   .. rubric:: Notes

   Proportional controllers are used for simplicity. The magnetic saturation is
   not considered in this implementation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ref: Reference generation parameters.
   :type ref: FluxTorqueReferencePars
   :param alpha_psi: Bandwidth of the flux controller (rad/s). The default is `2*pi*100`.
   :type alpha_psi: float, optional
   :param alpha_tau: Bandwidth of the torque controller (rad/s). The default is `2*pi*200`.
   :type alpha_tau: float, optional
   :param T_s: Sampling period (s). The default is `250e-6`.
   :type T_s: float
   :param sensorless: If `True`, sensorless control is used. The default is `True`.
   :type sensorless: bool, optional

   .. attribute:: observer

      Observer.

      :type: Observer | SensorlessObserver

   .. attribute:: flux_torque_ref

      Flux and torque reference generator.

      :type: FluxTorqueReference

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: float

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. rubric:: References

   .. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented
      control of IPM drives with variable DC link in the field-weakening
      region,” IEEE Trans.Ind. Appl., 2009,
      https://doi.org/10.1109/TIA.2009.2027167

   .. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented
      control of synchronous motors: A systematic design procedure," IEEE Trans.
      Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316















   ..
       !! processed by numpydoc !!

.. py:class:: FluxTorqueReferencePars

   
   Parameters for reference generation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param psi_s_min: Minimum stator flux (Vs). The default is `psi_f`.
   :type psi_s_min: float, optional
   :param psi_s_max: Maximum stator flux (Vs). The default is `inf`.
   :type psi_s_max: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional

   .. attribute:: psi_s_mtpa

      MTPA stator flux linkage (Vs) as a function of the torque (Nm).

      :type: callable

   .. attribute:: tau_M_lim

      Torque limit (Nm) as a function of the stator flux linkage (Vs). This
      limit merges the MTPV and current limits.

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: FluxTorqueReference(ref)

   
   Flux and torque references.

   The current and MTPV limits as well as the MTPA locus are implemented as
   look-up tables, which are generated based on the constant machine model
   parameters.

   :param ref: Reference generation parameters.
   :type ref: FluxTorqueReferencePars















   ..
       !! processed by numpydoc !!

