motulator.drive.control.sm
==========================

.. py:module:: motulator.drive.control.sm

.. autoapi-nested-parse::

   
   Controls for synchronous machines.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.control.sm.CurrentController
   motulator.drive.control.sm.CurrentReference
   motulator.drive.control.sm.CurrentReferenceCfg
   motulator.drive.control.sm.CurrentVectorControl
   motulator.drive.control.sm.FluxTorqueReference
   motulator.drive.control.sm.FluxTorqueReferenceCfg
   motulator.drive.control.sm.FluxVectorControl
   motulator.drive.control.sm.Observer
   motulator.drive.control.sm.ObserverBasedVHzControl
   motulator.drive.control.sm.ObserverBasedVHzControlCfg
   motulator.drive.control.sm.ObserverCfg
   motulator.drive.control.sm.SignalInjection
   motulator.drive.control.sm.SignalInjectionControl
   motulator.drive.control.sm.SpeedController
   motulator.drive.control.sm.TorqueCharacteristics


Package Contents
----------------

.. py:class:: CurrentController(par, alpha_c)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   Current controller for synchronous machines.

   This provides an interface of a current controller for synchronous machines
   [#Awa2019a]_. The gains are initialized based on the desired closed-loop
   bandwidth and the inductances.

   :param par: Synchronous machine parameters, should contain `L_d` and `L_q` (H).
   :type par: SynchronousMachinePars
   :param alpha_c: Closed-loop bandwidth (rad/s).
   :type alpha_c: float

   .. rubric:: References

   .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current
      control of saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















   ..
       !! processed by numpydoc !!

   .. py:method:: output(ref_i, i)

      
      Compute the controller output.

      :param ref_i: Reference signal.
      :type ref_i: complex
      :param i: Feedback signal.
      :type i: complex
      :param u_ff: Feedforward signal. The default is 0.
      :type u_ff: complex, optional

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentReference(par, cfg)

   
   Current reference calculation.

   This method includes the MTPA locus and field-weakening operation based on
   the unlimited voltage reference feedback. The MTPV and current limits are
   taken into account. This resembles the method presented [#Bed2020]_.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param cfg: Reference generation configuration.
   :type cfg: CurrentReferenceCfg

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

   .. py:method:: output(fbk, ref)

      
      Compute the stator current reference.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Field-weakening control based on the unlimited reference voltage.
















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentReferenceCfg

   
   Reference generation configuration.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param max_i_s: Maximum stator current (A).
   :type max_i_s: float
   :param min_psi_s: Minimum stator flux (Vs). The default is `psi_f`.
   :type min_psi_s: float, optional
   :param nom_w_m: Nominal rotor angular speed (electrical rad/s). Needed if `k_fw` is not
                   directly provided.
   :type nom_w_m: float, optional
   :param alpha_fw: Field-weakening bandwidth (rad/s). The default is 2*pi*20.
   :type alpha_fw: float, optional
   :param k_fw: Field-weakening gain. The default is `alpha_fw/(w_m_nom*par.L_d)`.
   :type k_fw: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional

   .. attribute:: mtpa_i_sd

      MTPA d-axis current (A) as a function of the torque (Nm).

      :type: callable

   .. attribute:: lim_tau_M

      Torque limit (Nm) as a function of the stator flux linkage (Vs). This
      limit merges the MTPV and current limits.

      :type: callable

   .. attribute:: lim_i_sd

      d-axis current limit (A) as a function of the stator flux linkage (Vs).
      This limit merges the MTPV and current limits.

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentVectorControl(par, cfg, T_s=0.00025, J=None, alpha_c=2 * np.pi * 200, alpha_o=2 * np.pi * 100, sensorless=True)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   Current vector control for synchronous machine drives.

   This class interconnects the subsystems of the control system and provides
   the interface to the solver.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param cfg: Reference generation configuration.
   :type cfg: CurrentReferenceCfg
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional
   :param J: Moment of inertia (kgm²). Needed only for the speed controller.
   :type J: float, optional
   :param alpha_c: Current controller bandwidth (rad/s). The default is 2*pi*200.
   :type alpha_c: float, optional
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*100.
   :type alpha_o: float, optional
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: current_reference

      Current reference generator.

      :type: CurrentReference

   .. attribute:: observer

      Flux and rotor position observer, used in the sensorless mode only.

      :type: Observer | None

   .. attribute:: current_ctrl

      Current controller. The default is CurrentController(par, 2*np.pi*200).

      :type: CurrentController

   .. attribute:: speed_ctrl

      Speed controller. The default is SpeedController(par.J, 2*np.pi*4).

      :type: SpeedController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Output
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Update
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxTorqueReference(cfg)

   
   Flux and torque references.

   The current and MTPV limits as well as the MTPA locus are implemented as
   look-up tables, which are generated based on the constant machine model
   parameters.

   :param cfg: Reference generation configuration.
   :type cfg: FluxTorqueReferenceCfg















   ..
       !! processed by numpydoc !!

.. py:class:: FluxTorqueReferenceCfg

   
   Reference generation configuration.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param max_i_s: Maximum stator current (A).
   :type max_i_s: float
   :param min_psi_s: Minimum stator flux (Vs). The default is `par.psi_f`.
   :type min_psi_s: float, optional
   :param max_psi_s: Maximum stator flux (Vs). The default is inf.
   :type max_psi_s: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxVectorControl(par, cfg, alpha_psi=2 * np.pi * 100, alpha_tau=2 * np.pi * 200, alpha_o=2 * np.pi * 100, J=None, T_s=0.00025, sensorless=True)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   Flux-vector control of synchronous machine drives.

   This class implements a variant of flux-vector control [#Pel2009]_. Rotor
   coordinates as well as decoupling between the stator flux and torque
   channels are used according to [#Awa2019b]_. Here, the stator flux
   magnitude and the electromagnetic torque are selected as controllable
   variables, and proportional controllers are used for simplicity
   [#Tii2024]_. The magnetic saturation is not considered in this
   implementation.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param cfg: Reference generation configuration.
   :type cfg: FluxTorqueReferenceCfg
   :param alpha_psi: Flux-control bandwidth (rad/s). The default is 2*pi*100.
   :type alpha_psi: float, optional
   :param alpha_tau: Torque-control bandwidth (rad/s). The default is 2*pi*200.
   :type alpha_tau: float, optional
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*100.
   :type alpha_o: float, optional
   :param J: Moment of inertia (kgm²). Needed only for the speed controller.
   :type J: float, optional
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. rubric:: References

   .. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented
      control of IPM drives with variable DC link in the field-weakening
      region,” IEEE Trans.Ind. Appl., 2009,
      https://doi.org/10.1109/TIA.2009.2027167

   .. [#Awa2019b] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented
      control of synchronous motors: A systematic design procedure," IEEE
      Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

   .. [#Tii2024] Tiitinen, Hinkkanen, Harnefors, "Design framework for
      sensorless control of synchronous machine drives," IEEE Trans. Ind.
      Electron., 2024, https://doi.org/10.1109/TIE.2024.3429650















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Calculate references.
















      ..
          !! processed by numpydoc !!


.. py:class:: Observer(cfg)

   
   Observer for synchronous machines in estimated rotor coordinates.

   This observer estimates the stator flux linkage, the rotor angle, the rotor
   speed, and (optionally) the PM-flux linkage. The design is based on
   [#Hin2018]_ and [#Tuo2018]. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the corresponding
   linearized estimation error dynamics. The PM-flux linkage can also be
   estimated [#Tuo2018]_. The observer can also be used in the sensored mode,
   in which case the control system is fixed to the measured rotor angle.

   :param cfg: Observer configuration.
   :type cfg: ObserverCfg

   .. rubric:: References

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis,"
      IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

   .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-
      magnet flux adaptation for sensorless synchronous motor drives," Proc.
      IEEE SLED, 2018, https://doi.org/10.1109/SLED.2018.8485899















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Compute the feedback signals for the control system.

      :param fbk:
                  Measured signals, which should contain the following fields:

                      u_ss : complex
                          Stator voltage (V) in stator coordinates.
                      i_ss : complex
                          Stator current (A) in stator coordinates.
                      w_m : float, optional
                          Rotor angular speed (electrical rad/s). This is only needed
                          in the sensored mode.
                      theta_m : float, optional
                          Rotor angle (electrical rad). This is only needed in the
                          sensored mode.
      :type fbk: SimpleNamespace

      :returns: **fbk** -- Measured and estimated feedback signals for the control system,
                containing at least the following fields:

                    u_s : complex
                        Stator voltage (V) in estimated rotor coordinates.
                    i_s : complex
                        Stator current (A) in estimated rotor coordinates.
                    psi_f : float
                        PM-flux magnitude estimate (Vs).
                    theta_m : float
                        Rotor angle estimate (electrical rad).
                    w_s : float
                        Angular frequency (rad/s) of the coordinate system.
                    w_m : float
                        Rotor speed estimate (electrical rad/s).
                    psi_s : complex
                        Stator flux estimate (Vs).
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, fbk)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzControl(par, cfg, T_s=0.00025)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   Observer-based V/Hz control for synchronous motors.

   This observer-based V/Hz control control method is based on [#Tii2022]_.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param cfg: Control system configuration.
   :type cfg: ObserverBasedVHzControlCfg
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional

   .. rubric:: References

   .. [#Tii2022] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors,
      "Stable and passive observer-based V/Hz control for synchronous Motors,"
      Proc. IEEE ECCE, 2022, https://doi.org/10.1109/ECCE50734.2022.9947858















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Output.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Update the states.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzControlCfg

   Bases: :py:obj:`motulator.drive.control.sm._flux_vector.FluxTorqueReferenceCfg`


   
   Control system configuration.

   :param alpha_psi: Flux control bandwidth (rad/s). The default is 2*pi*50.
   :type alpha_psi: float, optional
   :param alpha_tau: Torque control bandwidth (rad/s). The default is 2*pi*50.
   :type alpha_tau: float
   :param alpha_f: Bandwidth of the high-pass filter (rad/s). The default is 2*pi*1.
   :type alpha_f: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverCfg

   
   Observer configuration.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param sensorless: If True, sensorless mode is used.
   :type sensorless: bool
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed. The default is
               ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
               `sensorless` else ``lambda w_m: 2*pi*15``.
   :type k_o: callable, optional
   :param k_f: PM-flux estimation gain (V) as a function of the rotor angular speed.
               The default is zero, ``lambda w_m: 0``. A typical nonzero gain is of
               the form ``lambda w_m: max(k*(abs(w_m) - w_min), 0)``, i.e., zero below
               the speed `w_min` (rad/s) and linearly increasing above that with the
               slope `k` (Vs).
   :type k_f: callable, optional















   ..
       !! processed by numpydoc !!

.. py:class:: SignalInjection(par, U_inj)

   
   Estimate the rotor position error based on signal injection.

   This signal-injection method estimates the rotor position error based on
   the injected switching frequency signal. The estimate can be used in a
   phase-locked loop or in a state observer to robustify low-speed sensorless
   operation.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param U_inj: Injected voltage amplitude (V).
   :type U_inj: float















   ..
       !! processed by numpydoc !!

   .. py:method:: filter_current(i_s)

      
      Filter the stator current using the previously measured value.

      :param i_s: Unfiltered stator current (A) in estimated rotor coordinates.
      :type i_s: complex

      :returns: **i_s_flt** -- Filtered stator current (A) in estimated rotor coordinates.
      :rtype: complex















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


.. py:class:: SignalInjectionControl(par, cfg, J=None, T_s=0.00025)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
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

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars
   :param cfg: Reference generation configuration.
   :type cfg: CurrentReferenceCfg
   :param J: Moment of inertia (kgm²). Needed only for the speed controller.
   :type J: float, optional
   :param T_s: Sampling period (s).
   :type T_s: float















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Compute outputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedController(J, alpha_s, max_tau_M=np.inf)

   Bases: :py:obj:`motulator.common.control.PIController`


   
   2DOF PI speed controller.

   This is an interface for a speed controller. The gains are initialized
   based on the desired closed-loop bandwidth and the rotor inertia estimate.

   :param J: Total inertia of the rotor (kgm²).
   :type J: float
   :param alpha_s: Closed-loop bandwidth (rad/s).
   :type alpha_s: float
   :param max_tau_M: Maximum motor torque (Nm). The default is `inf`.
   :type max_tau_M: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: TorqueCharacteristics(par)

   
   Compute MTPA and MTPV loci based on the machine parameters.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars















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


   .. py:method:: current_limit(max_i_s, gamma1=np.pi, gamma2=0, N=20)

      
      Compute the current limit.

      :param max_i_s: Current limit (A).
      :type max_i_s: float
      :param gamma1: Starting angle (electrical rad). The default is 0.
      :type gamma1: float, optional
      :param gamma2: End angle (electrical rad). The default is pi.
      :type gamma2: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: **Object with the following fields defined** --

                psi_s : complex
                    Stator flux (Vs).
                i_s : complex
                    Stator current (A).
                tau_M : float
                    Electromagnetic torque (Nm).
                tau_M_vs_abs_psi_s : interp1d object
                    Torque (Nm) as a function of the flux magnitude (Vs).
      :rtype: SimpleNamespace















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


   .. py:method:: mtpa_locus(max_i_s, min_psi_s=None, N=20)

      
      Compute the MTPA locus.

      :param max_i_s: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type max_i_s: float
      :param min_psi_s: Minimum stator flux magnitude (Vs) at which the locus is computed.
      :type min_psi_s: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: **Object with the following fields defined** --

                psi_s : complex
                    Stator flux (Vs).
                i_s : complex
                    Stator current (A).
                tau_M : float
                    Electromagnetic torque (Nm).
                abs_psi_s_vs_tau_M : callable
                    Stator flux magnitude (Vs) as a function of the torque (Nm).
                i_sd_vs_tau_M : callable
                    d-axis current (A) as a function of the torque (Nm).
      :rtype: SimpleNamespace















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


   .. py:method:: mtpv_and_current_limits(max_i_s, N=20)

      
      Merge the MTPV and current limits into a single interpolant.

      :param max_i_s: Current limit (A).
      :type max_i_s: float
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: **Object with the following fields defined** --

                tau_M_vs_abs_psi_s : interp1d object
                    Torque (Nm) as a function of the flux magnitude (Vs).
                i_sd_vs_tau_M : interp1d object
                    d-axis current (A) as a function of the torque (Nm).
      :rtype: SimpleNamespace















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


   .. py:method:: mtpv_locus(max_psi_s=None, max_i_s=None, N=20)

      
      Compute the MTPV locus.

      :param max_psi_s: Maximum stator flux magnitude (Vs) at which the locus is computed.
                        Either `max_psi_s` or `max_i_s` must be given.
      :type max_psi_s: float, optional
      :param max_i_s: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type max_i_s: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: **Object with the following fields defined** --

                psi_s : complex
                    Stator flux (Vs).
                i_s : complex
                    Stator current (A).
                tau_M : float
                    Electromagnetic torque (Nm).
                tau_M_vs_abs_psi_s : interp1d object
                    Torque (Nm) as a function of the flux magnitude (Vs).
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_current_loci(max_i_s, base, N=20)

      
      Plot the current loci.

      :param max_i_s: Maximum current (A) at which the loci are evaluated.
      :type max_i_s: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_loci(max_i_s, base, N=20)

      
      Plot the stator flux linkage loci.

      :param max_i_s: Maximum current (A) at which the loci are evaluated.
      :type max_i_s: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_torque_current(max_i_s, base, N=20)

      
      Plot torque vs. current characteristics.

      :param max_i_s: Maximum current (A) at which the loci are evaluated.
      :type max_i_s: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_torque_flux(max_i_s, base, N=20)

      
      Plot torque vs. flux magnitude characteristics.

      :param max_i_s: Maximum current (A) at which the loci are evaluated.
      :type max_i_s: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















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


