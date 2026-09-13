motulator.drive.control.sm
==========================

.. py:module:: motulator.drive.control.sm

.. autoapi-nested-parse::

   Controls for synchronous machine drives.

   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.control.sm.CurrentController
   motulator.drive.control.sm.CurrentVectorController
   motulator.drive.control.sm.CurrentVectorControllerCfg
   motulator.drive.control.sm.FluxObserver
   motulator.drive.control.sm.FluxVectorController
   motulator.drive.control.sm.FluxVectorControllerCfg
   motulator.drive.control.sm.ObserverBasedVHzController
   motulator.drive.control.sm.ObserverBasedVHzControllerCfg
   motulator.drive.control.sm.ObserverOutputs
   motulator.drive.control.sm.PIController
   motulator.drive.control.sm.ReferenceGenerator
   motulator.drive.control.sm.SaturatedSynchronousMachinePars
   motulator.drive.control.sm.SignalInjectionController
   motulator.drive.control.sm.SpeedController
   motulator.drive.control.sm.SpeedFluxObserver
   motulator.drive.control.sm.SpeedObserver
   motulator.drive.control.sm.SynchronousMachinePars
   motulator.drive.control.sm.VHzControlSystem
   motulator.drive.control.sm.VectorControlSystem


Module Contents
---------------

.. py:class:: CurrentController(par, alpha_c, alpha_i = None)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   Current controller for synchronous machines.

   This provides an interface of a current controller for synchronous machines
   [#Awa2019a]_. The gains are initialized based on the desired closed-loop bandwidth
   and the inductances (or nonlinear flux linkage maps).

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param alpha_c: Reference-tracking bandwidth (rad/s).
   :type alpha_c: float
   :param alpha_i: Integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional

   .. rubric:: References

   .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
      saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
      https://doi.org/10.1109/TIA.2019.2919258















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(i_ref, i, u_ff = 0j)

      
      Compute the controller output.

      :param i_ref: Reference signal.
      :type i_ref: complex
      :param i: Feedback signal.
      :type i: complex
      :param u_ff: Feedforward signal, defaults to 0.
      :type u_ff: complex, optional

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentVectorController(par, cfg)

   
   Current vector controller for synchronous machine drives.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param cfg: Current-vector control configuration.
   :type cfg: CurrentVectorControllerCfg















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_meas, theta_M_meas)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentVectorControllerCfg

   
   Current-vector controller configuration.

   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_c: Current-control bandwidth (rad/s), defaults to 2*pi*200.
   :type alpha_c: float, optional
   :param alpha_i: Current-control integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*50 if `J` is None, otherwise
                   2*pi*50/3, keeping the default speed observer gain the same.
   :type alpha_o: float, optional
   :param alpha_cur: Current-reference tracking bandwidth (rad/s), defaults to 2*pi*200.
   :type alpha_cur: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], float], optional
   :param k_f: PM-flux estimation gain as a function of the rotor angular speed.
   :type k_f: Callable[[float], float], optional
   :param psi_s_min: Minimum stator flux (Vs), defaults to `par.psi_f` elsewhere.
   :type psi_s_min: float, optional
   :param psi_s_max: Maximum stator flux (Vs), defaults to `inf`.
   :type psi_s_max: float, optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_mtpv: MTPV margin, defaults to 0.9.
   :type k_mtpv: float, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float | None, optional
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxObserver(par, k_theta, k_o, k_f)

   
   Observer for synchronous machines in estimated rotor coordinates.

   This observer estimates the stator flux linkage, the rotor angle, and (optionally)
   the PM-flux linkage. The design is based on [#Hin2018]_ and [#Tuo2018]. The observer
   gain decouples the electrical and mechanical dynamics and allows placing the poles
   of the corresponding linearized estimation error dynamics. The rotor angle is
   tracked using a position error signal, which is either computed internally from the
   back-EMF-based flux estimation error or supplied externally, e.g., from the measured
   rotor angle. The weight `h` of the external signal, given to `compute_output`,
   selects between these: `h = 0` gives purely model-based (sensorless) operation,
   `h = 1` relies on the external signal alone, and intermediate values blend the two.
   The magnetic saturation is taken into account based on the provided machine model.
   The PM-flux linkage can also be estimated [#Tuo2018]_.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param k_theta: Rotor angle estimation gain (rad/s).
   :type k_theta: float
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], float]
   :param k_f: PM-flux estimation gain (V) as a function of the rotor angular speed.
   :type k_f: Callable[[float], float], optional

   .. rubric:: References

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis," IEEE
      Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

   .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-magnet flux
      adaptation for sensorless synchronous motor drives," Proc. IEEE SLED, 2018,
      https://doi.org/10.1109/SLED.2018.8485899















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, w_M, eps_ext = 0.0, h = 0.0)

      
      Compute the feedback signals for the control system.

      :param u_s_ab: Stator voltage (V) in stator coordinates.
      :type u_s_ab: complex
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex
      :param w_M: Mechanical rotor speed (rad/s), typically from the speed observer.
      :type w_M: float
      :param eps_ext: External mechanical position error signal (rad), defaults to 0.
      :type eps_ext: float, optional
      :param h: Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
                based error signal is used exclusively.
      :type h: float, optional

      :returns: **out** -- Estimated feedback signals for the control system.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorController(par, cfg)

   
   Flux-vector controller of synchronous machine drives.

   This class implements a variant of flux-vector control. Rotor coordinates and
   decoupling between the stator flux and torque channels are used according to
   [#Awa2019b]_. Here, the stator flux magnitude and the electromagnetic torque are
   selected as controllable variables [#Tii2025a]_. The magnetic saturation is taken
   into account [#Var2022]_.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param cfg: Flux-vector control configuration.
   :type cfg: FluxVectorControllerCfg

   .. rubric:: References

   .. [#Awa2019b] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of
      synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019,
      https://doi.org/10.1109/TIA.2019.2927316

   .. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless
      control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025,
      https://doi.org/10.1109/TIE.2024.3429650

   .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
      synchronous motor drives: Accurate decoupled control with online adaptive maximum
      torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
      Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_meas, theta_M_meas)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorControllerCfg

   
   Flux-vector controller configuration.

   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_tau: Torque-control bandwidth (rad/s), defaults to 2*pi*100.
   :type alpha_tau: float, optional
   :param alpha_psi: Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
   :type alpha_psi: float | None, optional
   :param alpha_i: Integral-action bandwidth (rad/s), defaults to `alpha_tau`.
   :type alpha_i: float | None, optional
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*50 if `J` is None, otherwise
                   2*pi*50/3, keeping the default speed observer gain the same.
   :type alpha_o: float | None, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], float] | None, optional
   :param k_f: PM-flux estimation gain as a function of the rotor angular speed.
   :type k_f: Callable[[float], float] | None, optional
   :param psi_s_min: Minimum stator flux (Vs). If None, defaults to `par.psi_f` elsewhere.
   :type psi_s_min: float | None, optional
   :param psi_s_max: Maximum stator flux (Vs), defaults to `inf`.
   :type psi_s_max: float, optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_mtpv: MTPV margin, defaults to 0.85.
   :type k_mtpv: float, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float | None, optional
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverBasedVHzController(par, cfg)

   
   Observer-based V/Hz controller for synchronous machine drives.

   This class implements sensorless observer-based V/Hz control. Rotor coordinates and
   decoupling between the stator flux and torque channels are used [#Tii2025a]_.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param cfg: Observer-based V/Hz control configuration.
   :type cfg: ObserverBasedVHzControllerCfg















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Calculate references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_ref)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzControllerCfg

   
   Observer-based V/Hz controller configuration.

   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_psi: Flux-control bandwidth (rad/s), defaults to 2*pi*100.
   :type alpha_psi: float, optional
   :param alpha_tau: Torque-control bandwidth (rad/s), defaults to 2*pi*20.
   :type alpha_tau: float, optional
   :param alpha_f: Filter bandwidth (rad/s), defaults to 2*pi*1.
   :type alpha_f: float, optional
   :param alpha_o: Angle estimation pole (rad/s), defaults to 2*pi*200.
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], complex], optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_mtpv: MTPV margin, defaults to 0.9.
   :type k_mtpv: float, optional
   :param psi_s_min: Minimum stator flux (Vs), defaults to `par.psi_f` elsewhere.
   :type psi_s_min: float | None, optional
   :param psi_s_max: Maximum stator flux (Vs), defaults to `inf`.
   :type psi_s_max: float, optional
   :param T_s: Sampling period (s), defaults to 250e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverOutputs

   
   Feedback signals for the control system.
















   ..
       !! processed by numpydoc !!

.. py:class:: PIController(k_p, k_i, k_t = None, u_max = inf)

   
   2DOF PI controller.

   This implements a discrete-time 2DOF PI controller, whose continuous-time
   counterpart is::

       u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y) + u_ff

   where `u` is the controller output, `y_ref` is the reference signal, `y` is the
   feedback signal, `u_ff` is the feedforward signal, and `1/s` refers to integration.
   The standard PI controller is obtained by choosing ``k_t = k_p``. The integrator
   anti-windup is implemented based on the realized controller output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain, defaults to `k_p`.
   :type k_t: float, optional
   :param u_max: Maximum controller output, defaults to `inf`.
   :type u_max: float, optional

   .. rubric:: Notes

   This controller can be used, e.g., as a speed controller. In this case, `y`
   corresponds to the rotor angular speed `w_M` and `u` to the torque reference
   `tau_M_ref`.















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(y_ref, y, u_ff = 0.0)

      
      Compute the controller output.

      :param y_ref: Reference signal.
      :type y_ref: float
      :param y: Feedback signal.
      :type y: float
      :param u_ff: Feedforward signal, defaults to 0.
      :type u_ff: float, optional

      :returns: **u** -- Controller output.
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u: Realized (limited) controller output.
      :type u: float















      ..
          !! processed by numpydoc !!


.. py:class:: ReferenceGenerator(par, i_s_max, psi_s_min = None, psi_s_max = inf, k_u = 1.0, k_mtpv = 1.0, alpha_cur = 2 * pi * 200)

   
   Optimal reference generator for synchronous machines.

   This class computes the optimal flux, limited torque, and current references from a
   given torque reference. The MTPA locus as well as the current, voltage and MTPV
   limits are taken into account. This class can be used also for a saturated machine
   model. The flux and torque references are computed using pre-computed lookup
   tables [#Mey2006]_, [#Awa2018]_. The current reference is generated dynamically
   using a tracking law [#Sar2026]_, needed only for current-vector control.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param psi_s_min: Minimum stator flux (Vs), defaults to `par.psi_f`.
   :type psi_s_min: float, optional
   :param psi_s_max: Maximum stator flux (Vs), defaults to `inf`.
   :type psi_s_max: float, optional
   :param k_u: Voltage utilization factor, defaults to 1.
   :type k_u: float, optional
   :param k_mtpv: MTPV margin, defaults to 1.
   :type k_mtpv: float, optional
   :param alpha_cur: Bandwidth of the current-reference tracking (rad/s), defaults to 2*pi*200. It
                     should be well below the sampling frequency to maintain a numerical margin.
   :type alpha_cur: float, optional

   .. rubric:: References

   .. [#Mey2006] Meyer, Böcker, “Optimum control for interior permanent magnet
      synchronous motors (IPMSM) in constant torque and flux weakening range,” Proc.
      EPE-PEMC, 2006, https://doi.org/10.1109/EPEPEMC.2006.4778413

   .. [#Awa2018] Awan, Song, Saarakkala, Hinkkanen, “Optimal torque control of
      saturated  synchronous motors: Plug-and-play method,” IEEE Trans. Ind. Appl.,
      2018, https://doi.org/10.1109/TIA.2018.2862410

   .. [#Sar2026] Sarén, Hartikainen, Piippo, Hinkkanen, "Decoupled online feedforward
      generation of optimal references for saturated synchronous machine drives,"
      2026, https://arxiv.org/abs/2607.08528















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_current_ref(tau_M_ref)

      
      Compute the current reference.

      This method is needed only for current-vector control. It returns the current
      reference state. The state is updated in the `update` method.

      :param tau_M_ref: Torque reference (Nm).
      :type tau_M_ref: float

      :returns: Stator current reference (A) in rotor coordinates.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_flux_and_torque_refs(tau_M_ref, w_m, u_dc)

      
      Compute the flux and torque reference signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, psi_s_abs_ref, tau_M_ref)

      
      Update the current-reference state.

      The current reference is a state variable, driven toward the given flux and
      torque references by the tracking law with the bandwidth `alpha_cur`
      [#Sar2026]_.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param psi_s_abs_ref: Stator flux reference (Vs).
      :type psi_s_abs_ref: float
      :param tau_M_ref: Torque reference (Nm).
      :type tau_M_ref: float















      ..
          !! processed by numpydoc !!


.. py:class:: SaturatedSynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Parameters of a saturated synchronous machine.

   The saturation model is specified as a current map (current as a function of the
   flux linkage). Optionally, to be used only in control systems, a flux map (flux
   linkage as a function of the current) can be provided. For convenience, this class
   also provides the incremental inductance matrix, which can be used in control and
   optimal reference generation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param i_s_dq_fcn: Stator current (A) as a function of the stator flux linkage (Vs). Needed in the
                      system model and in some control methods.
   :type i_s_dq_fcn: Callable[[complex], complex], optional
   :param psi_s_dq_fcn: Stator flux linkage (Vs) as a function of the stator current (A). This function
                        should be differentiable, if incremental inductances are used. Needed only for
                        control methods and optimal reference loci, not used in the system model.
   :type psi_s_dq_fcn: Callable[[complex], complex], optional
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix at given current.
















      ..
          !! processed by numpydoc !!


   .. py:method:: iterate_i_s_dq(psi_s_dq)

      
      Compute the current from the flux linkage using root finding.

      The current is computed iteratively from the flux map using a root-finding
      algorithm. This is less efficient, but may be convenient in some special cases.















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage as a function of the stator current.
















      ..
          !! processed by numpydoc !!


.. py:class:: SignalInjectionController(par, cfg, U_inj = 250, N_inj = 1)

   Bases: :py:obj:`motulator.drive.control._sm_current_vector.CurrentVectorController`


   
   Sensorless controller with signal injection for synchronous machine drives.

   This class implements a square-wave signal injection for low-speed operation
   according to [#Kim2012]_. Cross-saturation errors are compensated for using flux
   maps [#You2018]_. A related adjustable-frequency method is presented in [#Yu2022]_.
   If the inertia of the mechanical system is provided, the speed is estimated using
   the speed observer based on the mechanical model [#Kim2003]_, otherwise the phase-
   locked loop is used.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param cfg: Current-vector control configuration.
   :type cfg: CurrentVectorControllerCfg
   :param U_inj: Injected voltage amplitude (V), defaults to 250.
   :type U_inj: float, optional
   :param N_inj: Number of sampling periods per injection voltage half-period, defaults to 1. The
                 injection frequency is `1 / (2 * N_inj * cfg.T_s)`. Reducing this frequency
                 requires reducing `cfg.alpha_o` accordingly.
   :type N_inj: int, optional

   .. rubric:: References

   .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless
      method in IPMSM," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2210175

   .. [#You2018] Yousefi-Talouki, Pescetto, Pellegrino, Boldea, "Combined active flux
      and high-frequency injection methods for sensorless direct-flux vector control of
      synchronous reluctance machines," IEEE Trans. Power Electron., 2018,
      https://doi.org/10.1109/TPEL.2017.2697209

   .. [#Yu2022] Yu, Wang, "Position sensorless control of IPMSM using adjustable
      frequency setting square-wave voltage injection," IEEE Trans. Power Electron.,
      2022, https://doi.org/10.1109/TPEL.2022.3179611

   .. [#Kim2003] Kim, Harke, Lorenz, "Sensorless control of interior permanent-magnet
      machine drives with zero-phase lag position estimation," IEEE Trans. Ind. Appl.,
      2003, https://doi.org/10.1109/TIA.2003.818966















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedController(J, alpha_s, alpha_i = None, tau_M_max = inf)

   Bases: :py:obj:`motulator.common.control._controllers.PIController`


   
   2DOF PI speed controller.

   This is an interface for a speed controller. The gains are initialized based on the
   desired closed-loop bandwidth and the rotor inertia estimate.

   :param J: Total inertia of the rotor (kgm²).
   :type J: float
   :param alpha_s: Reference-tracking bandwidth (rad/s).
   :type alpha_s: float
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_s`.
   :type alpha_i: float, optional
   :param tau_M_max: Maximum motor torque (Nm), defaults to `inf`.
   :type tau_M_max: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: SpeedFluxObserver(par, alpha_o, k_o, k_f, J = None)

   
   Flux observer with speed estimation.

   This observer estimates the stator flux linkage, rotor angle, and rotor speed. The
   observer gain decouples the electrical and mechanical dynamics and allows placing
   the poles of the corresponding linearized estimation error dynamics. If the inertia
   of the mechanical system is provided, the observer also estimates the load torque,
   to avoid the lag in the speed estimate. The rotor angle is tracked using a position
   error signal, see {class}`FluxObserver`.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars
   :param alpha_o: Speed-estimation pole (rad/s).
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], float], optional
   :param k_f: PM-flux estimation gain (V) as a function of the rotor angular speed.
   :type k_f: Callable[[float], float], optional
   :param J: Inertia of the mechanical system (kgm²). Defaults to None, which means the
             mechanical system model is not used.
   :type J: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, eps_ext = 0.0, h = 0.0)

      
      Compute the feedback signals for the control system.

      :param u_s_ab: Stator voltage (V) in stator coordinates.
      :type u_s_ab: complex
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex
      :param eps_ext: External mechanical position error signal (rad), defaults to 0.
      :type eps_ext: float, optional
      :param h: Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
                based error signal is used exclusively.
      :type h: float, optional

      :returns: **out** -- Estimated feedback signals for the control system.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:property:: theta_m
      :type: float

      
      Electrical rotor angle estimate (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedObserver(k_w, k_tau, J)

   
   Speed observer.

   This observer estimates the mechanical rotor speed based on the mechanical system
   model and the error signal. If the inertia of the mechanical system is provided, the
   load torque is als estimated, avoiding lag in the speed estimate [#Lor1991]_.

   :param k_w: Speed-estimation gain (rad/s or (rad/s)² for induction machines or for
               synchronous machines, respectively).
   :type k_w: float
   :param k_tau: Load-torque estimation gain (Nm or Nm/s for induction machines or for
                 synchronous machines, respectively).
   :type k_tau: float
   :param J: Inertia of the mechanical system (kgm²). Defaults to None, which means the load
             torque estimation is not used.
   :type J: float | None, optional

   .. rubric:: References

   .. [#Lor1991] Lorenz, Van Patten, "High-resolution velocity estimation for
      all-digital, AC servo drives," IEEE Trans. Ind. Appl., 1991,
      https://doi.org/10.1109/28.85485















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output()

      
      Compute outputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, eps, tau_M = 0.0)

      
      Update mechanical state estimates.

      :param T_s: Sample time (s).
      :type T_s: float
      :param eps: Estimation error signal: mechanical speed (rad/s) or mechanical position
                  (rad) for induction machines or synchronous machines, respectively.
      :type eps: float
      :param tau_M: Electromagnetic torque estimate (Nm).
      :type tau_M: float, optional















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Synchronous machine parameters, without saturation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: Permanent-magnet flux linkage (Vs).
   :type psi_f: float
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix (H).
















      ..
          !! processed by numpydoc !!


   .. py:method:: iterate_i_s_dq(psi_s_dq)

      
      Compute the current from the flux linkage using root finding.
















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage (Vs) as a function of the stator current (A).
















      ..
          !! processed by numpydoc !!


.. py:class:: VHzControlSystem(vhz_ctrl, slew_rate = inf)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   V/Hz control system.

   :param vhz_ctrl: V/Hz controller to be used in the drive control system.
   :type vhz_ctrl: VHzController
   :param slew_rate: Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.
   :type slew_rate: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference.

      :param ref_fcn: Speed reference (mechanical rad/s) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


.. py:class:: VectorControlSystem(vector_ctrl, speed_ctrl = None)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Vector control system.

   This class defines the interface for drive control systems. It is a generic class
   that can be used with different inner controllers (such as current-vector control
   and flux-vector control).

   :param vector_ctrl: Vector controller whose input is the torque reference.
   :type vector_ctrl: VectorController
   :param speed_ctrl: Speed controller. If not given or None, torque-control mode is used.
   :type speed_ctrl: SpeedController | PIController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements from sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference for speed-control mode.

      :param ref_fcn: Speed reference (mechanical rad/s) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_torque_ref(ref_fcn)

      
      Set the external torque reference for torque-control mode.

      :param ref_fcn: Torque reference (Nm) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


