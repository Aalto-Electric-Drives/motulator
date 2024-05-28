motulator.control.sm
====================

.. py:module:: motulator.control.sm

.. autoapi-nested-parse::

   
   This package contains example controllers for synchronous machines.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.control.sm.CurrentReferencePars
   motulator.control.sm.CurrentReference
   motulator.control.sm.CurrentCtrl
   motulator.control.sm.ModelPars
   motulator.control.sm.VectorCtrl
   motulator.control.sm.Observer
   motulator.control.sm.FluxObserver
   motulator.control.sm.FluxVectorCtrl
   motulator.control.sm.FluxTorqueReference
   motulator.control.sm.FluxTorqueReferencePars
   motulator.control.sm.ObserverBasedVHzCtrl
   motulator.control.sm.ObserverBasedVHzCtrlPars
   motulator.control.sm.TorqueCharacteristics
   motulator.control.sm.SignalInjectionCtrl
   motulator.control.sm.SignalInjection


Package Contents
----------------

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

      
      Field-weakening control based on the unlimited reference voltage.

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

.. py:class:: Observer(par, alpha_o=2 * np.pi * 40, k=None, k_f=None, sensorless=True)

   
   Observer for synchronous machines in estimated rotor coordinates.

   This observer estimates the stator flux linkage, the rotor angle, the rotor
   speed, and (optionally) the PM-flux linkage. The design is based on
   [#Hin2018]_ and [#Tuo2018]. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the corresponding
   linearized estimation error dynamics. The PM-flux linkage can also be
   estimated [#Tuo2018]_. This implementation operates in estimated rotor
   coordinates. The observer can  also be used in the sensored mode by
   providing the measured rotor speed as an input.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer bandwidth (electrical rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
             `sensorless` else ``lambda w_m: 2*pi*15``.
   :type k: callable, optional
   :param k_f: PM-flux estimation gain (V) as a function of the rotor angular speed.
               The default is zero, ``lambda w_m: 0``. A typical nonzero gain is of
               the form ``lambda w_m: max(k*(abs(w_m) - w_min), 0)``, i.e., zero below
               the speed `w_min` (rad/s) and linearly increasing above that with the
               slope `k` (Vs).
   :type k_f: callable, optional
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: theta_m

      Rotor angle estimate (electrical rad).

      :type: float

   .. attribute:: w_m

      Rotor speed estimate (electrical rad/s).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex

   .. attribute:: psi_f

      PM-flux estimate (Vs). The PM-flux estimate lumps various disturbances.
      Hence, it can become slightly negative in the case of SyRMs.

      :type: float

   .. rubric:: Notes

   The sensored mode assumes that the control system operates in the measured
   rotor coordinates, i.e., the rotor-angle tracking is disabled.

   .. rubric:: References

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis,"
      IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

   .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-
      magnet flux adaptation for sensorless synchronous motor drives," Proc.
      IEEE SLED, 2018, https://doi.org/10.1109/SLED.2018.8485899















   ..
       !! processed by numpydoc !!

   .. py:method:: update(T_s, u_s, i_s, w_m=None)

      
      Update the states for the next sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V) in estimated rotor coordinates.
      :type u_s: complex
      :param i_s: Stator current (A) in estimated rotor coordinates.
      :type i_s: complex
      :param w_m: Rotor angular speed (electrical rad/s). Needed only in the sensored
                  mode. The default is None.
      :type w_m: float, optional















      ..
          !! processed by numpydoc !!


.. py:class:: FluxObserver(par, alpha_o=2 * np.pi * 20, zeta_inf=0.2)

   
   Sensorless stator flux observer in external coordinates.

   This observer estimates the stator flux linkage and the angle of the
   coordinate system with respect to the d-axis of the rotor. Speed-estimation
   is omitted. The observer gain decouples the electrical and mechanical
   dynamics and allows placing the poles of the corresponding linearized
   estimation error dynamics. This implementation operates in external
   coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer gain (rad/s). The default is 2*pi*20.
   :type alpha_o: float, optional
   :param zeta_inf: Damping ratio at infinite speed. The default is 0.2.
   :type zeta_inf: float, optional

   .. attribute:: delta

      Angle estimate of the coordinate system with respect to the d-axis of
      the rotor (electrical rad).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex















   ..
       !! processed by numpydoc !!

   .. py:method:: update(T_s, u_s, i_s, w_s)

      
      Update the states for the next sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V).
      :type u_s: complex
      :param i_s: Stator current (A).
      :type i_s: complex
      :param w_s: Stator angular frequency (rad/s).
      :type w_s: float















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorCtrl(par, ref, alpha_psi=2 * np.pi * 100, alpha_tau=2 * np.pi * 200, T_s=0.00025, sensorless=True)

   Bases: :py:obj:`motulator.control._common.Ctrl`


   
   Flux-vector control of synchronous machine drives.

   This class implements a variant of stator-flux-vector control [#Pel2009]_.
   Rotor coordinates as well as decoupling between the stator flux and torque
   channels are used according to [#Awa2019b]_. Here, the stator flux
   magnitude and the electromagnetic torque are selected as controllable
   variables.

   .. rubric:: Notes

   Proportional controllers are used for simplicity. The magnetic saturation
   is not considered in this implementation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ref: Reference generation parameters.
   :type ref: FluxTorqueReferencePars
   :param alpha_psi: Bandwidth of the flux controller (rad/s). The default is 2*pi*100.
   :type alpha_psi: float, optional
   :param alpha_tau: Bandwidth of the torque controller (rad/s). The default is 2*pi*200.
   :type alpha_tau: float, optional
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: observer

      Flux observer, having both sensorless and sensored modes.

      :type: Observer

   .. attribute:: flux_torque_ref

      Flux and torque reference generator.

      :type: FluxTorqueReference

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: callable

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. rubric:: References

   .. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented
      control of IPM drives with variable DC link in the field-weakening
      region,” IEEE Trans.Ind. Appl., 2009,
      https://doi.org/10.1109/TIA.2009.2027167

   .. [#Awa2019b] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented
      control of synchronous motors: A systematic design procedure," IEEE
      Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316















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

.. py:class:: FluxTorqueReferencePars

   
   Parameters for reference generation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param psi_s_min: Minimum stator flux (Vs). The default is `psi_f`.
   :type psi_s_min: float, optional
   :param psi_s_max: Maximum stator flux (Vs). The default is inf.
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

.. py:class:: ObserverBasedVHzCtrl(par, ctrl_par, T_s=0.00025)

   Bases: :py:obj:`motulator.control._common.Ctrl`


   
   Observer-based V/Hz control for synchronous motors.

   This observer-based V/Hz control control method is based on [#Tii2022]_.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ctrl_par: Control system parameters.
   :type ctrl_par: ObserverBasedVHzCtrlPars
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional

   .. attribute:: w_m_ref

      Rotor speed reference (electrical rad/s).

      :type: callable

   .. rubric:: References

   .. [#Tii2022] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors,
      "Stable and passive observer-based V/Hz control for synchronous Motors,"
      Proc. IEEE ECCE, 2022, https://doi.org/10.1109/ECCE50734.2022.9947858















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverBasedVHzCtrlPars

   Bases: :py:obj:`motulator.control.sm._flux_vector.FluxTorqueReferencePars`


   
   Parameters for the control system.

   This class extends FluxTorqueReferencePars with the parameters needed for
   the observer-based V/Hz control.

   :param alpha_psi: Flux control bandwidth (rad/s). The default is 2*pi*50.
   :type alpha_psi: float, optional
   :param alpha_tau: Torque control bandwidth (rad/s). The default is 2*pi*50.
   :type alpha_tau: float
   :param alpha_f: Bandwidth of the high-pass filter (rad/s). The default is 2*pi*1.
   :type alpha_f: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: TorqueCharacteristics(par)

   
   Compute MTPA and MTPV loci based on the machine parameters.

   The magnetic saturation is omitted.

   :param par: Machine model parameters.
   :type par: ModelPars















   ..
       !! processed by numpydoc !!

   .. py:method:: torque(psi_s)

      
      Compute the torque as a function of the stator flux linkage.

      :param psi_s: Stator flux (Vs).
      :type psi_s: complex

      :returns: **tau_M** -- Electromagnetic torque (Nm).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: current(psi_s)

      
      Compute the stator current as a function of the stator flux linkage.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: **i_s** -- Stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: flux(i_s)

      
      Compute the stator flux linkage as a function of the current.

      :param i_s: Stator current (A).
      :type i_s: complex

      :returns: **psi_s** -- Stator flux linkage (Vs).
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpa(abs_i_s)

      
      Compute the MTPA stator current angle.

      :param abs_i_s: Stator current magnitude (A).
      :type abs_i_s: float

      :returns: **beta** -- MTPA angle of the stator current vector (electrical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpv(abs_psi_s)

      
      Compute the MTPV stator flux angle.

      :param abs_psi_s: Stator flux magnitude (Vs).
      :type abs_psi_s: float

      :returns: **delta** -- MTPV angle of the stator flux vector (electrical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpv_current(abs_i_s)

      
      Compute the MTPV based on the current magnitude.

      This computes the MTPV based on the current magnitude, i.e., the
      intersection of the MTPV current locus and the current limit circle.
      This method is not necessary for computing the control look-up tables.
      It is used here to "cut" the MTPV characteristics at the desired
      current. Alternatively just a large enough maximum flux magnitude could
      be used.

      :param abs_i_s: Stator current magnitude (A).
      :type abs_i_s: float

      :returns: **i_s** -- MTPV stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpa_locus(i_s_max, psi_s_min=None, N=20)

      
      Compute the MTPA locus.

      :param i_s_max: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type i_s_max: float
      :param psi_s_min: Minimum stator flux magnitude (Vs) at which the locus is computed.
      :type psi_s_min: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **abs_psi_s_vs_tau_M** (*callable*) -- Stator flux magnitude (Vs) as a function of the torque (Nm).
                * **i_sd_vs_tau_M** (*callable*) -- d-axis current (A) as a function of the torque (Nm).















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpv_locus(psi_s_max=None, i_s_max=None, N=20)

      
      Compute the MTPV locus.

      :param psi_s_max: Maximum stator flux magnitude (Vs) at which the locus is computed.
                        Either `psi_s_max` or `i_s_max` must be given.
      :type psi_s_max: float, optional
      :param i_s_max: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type i_s_max: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).















      ..
          !! processed by numpydoc !!


   .. py:method:: current_limit(i_s_max, gamma1=np.pi, gamma2=0, N=20)

      
      Compute the current limit.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param gamma1: Starting angle (electrical rad). The default is 0.
      :type gamma1: float, optional
      :param gamma2: End angle (electrical rad). The defauls in pi.
      :type gamma2: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).















      ..
          !! processed by numpydoc !!


   .. py:method:: mtpv_and_current_limits(i_s_max, N=20)

      
      Merge the MTPV and current limits into a single interpolant.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).
                * **i_sd_vs_tau_M** (*interp1d object*) -- d-axis current (A) as a function of the torque (Nm).















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_loci(i_s_max, base, N=20)

      
      Plot the stator flux linkage loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_current_loci(i_s_max, base, N=20)

      
      Plot the current loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_torque_current(i_s_max, base, N=20)

      
      Plot torque vs. current characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_torque_flux(i_s_max, base, N=20)

      
      Plot torque vs. flux magnitude characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


.. py:class:: SignalInjectionCtrl(par, ref, T_s=0.00025)

   Bases: :py:obj:`motulator.control._common.Ctrl`


   
   Sensorless control with signal injection for synchronous machine drives.

   This class implements a square-wave signal injection for low-speed
   operation according to [#Kim2012]_. A phase-locked loop is used to track
   the rotor position.

   .. rubric:: Notes

   For a wider speed range, signal injection could be combined to a
   model-based observer. The effects of magnetic saturation are not
   compensated for in this version.

   .. rubric:: References

   .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection
      sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2210175

   :param T_s: Sampling period (s).
   :type T_s: float
   :param pars: Machine model parameters.
   :type pars: ModelPars
   :param U_inj: Amplitude of the injected voltage (V).
   :type U_inj: float
   :param w_o: PLL natural frequency (rad/s).
   :type w_o: float

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentCtrl

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: current_ref

      Current reference generator.

      :type: CurrentReference

   .. attribute:: pll

      Phase-locked loop.

      :type: PhaseLockedLoop

   .. attribute:: signal_inj

      Signal injection.

      :type: SignalInjection

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s).

      :type: callable

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM















   ..
       !! processed by numpydoc !!

.. py:class:: SignalInjection(par, U_inj)

   
   Estimate the rotor position error based on signal injection.

   This signal injection method estimates the rotor position error based on
   the injected switching frequency signal. The estimate can be used in a
   phase-locked loop or in a state observer to robustify low-speed sensorless
   operation.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param U_inj: Injected voltage amplitude (V).
   :type U_inj: float















   ..
       !! processed by numpydoc !!

   .. py:method:: output(T_s, i_sq)

      
      Compute the rotor position estimation error.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param i_sq: q-axis stator current (A) in estimated rotor coordinates.
      :type i_sq: float

      :returns: **err** -- Rotor position estimation error (electrical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(i_s)

      
      Store the old current values for the next sampling period.

      :param i_s: Stator current in estimated rotor coordinates.
      :type i_s: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: filter_current(i_s)

      
      Filter the stator current using the previously measured value.

      :param i_s: Unfiltered stator current (A) in estimated rotor coordinates.
      :type i_s: complex

      :returns: **i_s_filt** -- Filtered stator current (A) in estimated rotor coordinates.
      :rtype: complex















      ..
          !! processed by numpydoc !!


