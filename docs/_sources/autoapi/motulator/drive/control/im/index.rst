motulator.drive.control.im
==========================

.. py:module:: motulator.drive.control.im

.. autoapi-nested-parse::

   
   Controls for induction machines.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.control.im.FullOrderObserver
   motulator.drive.control.im.FullOrderObserverCfg
   motulator.drive.control.im.Observer
   motulator.drive.control.im.ObserverCfg
   motulator.drive.control.im.CurrentController
   motulator.drive.control.im.CurrentReference
   motulator.drive.control.im.CurrentReferenceCfg
   motulator.drive.control.im.CurrentVectorControl
   motulator.drive.control.im.ObserverBasedVHzControl
   motulator.drive.control.im.ObserverBasedVHzControlCfg
   motulator.drive.control.im.VHzControl
   motulator.drive.control.im.VHzControlCfg
   motulator.drive.control.im.SpeedController


Package Contents
----------------

.. py:class:: FullOrderObserver(cfg)

   
   Full-order flux observer operating in estimated rotor flux coordinates.

   This class implements a full-order flux observer for induction machines.
   The observer structure is similar to [#Tii2023]_. The observer operates in
   estimated rotor flux coordinates.

   :param cfg: Observer parameters.
   :type cfg: ObserverCfg

   .. rubric:: References

   .. [#Tii2023] Tiitinen, Hinkkanen, Harnefors, "Speed-adaptive full-order
      observer revisited: Closed-form design for induction motor drives,"
      Proc. IEEE SLED, 2023, https://doi.org/10.1109/SLED57582.2023.10261359















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Output.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, fbk)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: FullOrderObserverCfg

   Bases: :py:obj:`ObserverCfg`


   
   Full-order observer configuration.

   :param alpha_i: Current estimation bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_i: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Observer(cfg)

   
   Reduced-order flux observer operating in estimated rotor flux coordinates.

   This class implements a reduced-order flux observer for induction machines.
   Both sensored and sensorless operation are supported. The observer
   structure is similar to [#Hin2010]_. The observer operates in estimated
   rotor flux coordinates.

   :param cfg: Observer configuration.
   :type cfg: ObserverCfg

   .. rubric:: References

   .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers
      with stator-resistance adaptation for speed-sensorless induction motor
      drives," IEEE Trans. Power Electron., 2010,
      https://doi.org/10.1109/TPEL.2009.2039650















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
                          Rotor angular speed (electrical rad/s). This signal is only
                          needed in the sensored mode.
      :type fbk: SimpleNamespace

      :returns: **fbk** -- Measured and estimated feedback signals for the control system,
                containing at least the following fields:

                    u_s : complex
                        Stator voltage (V) in estimated rotor flux coordinates.
                    i_s : complex
                        Stator current (A) in estimated rotor flux coordinates.
                    psi_R : float
                        Rotor flux magnitude estimate (Vs).
                    theta_s : float
                        Rotor flux angle estimate (rad).
                    w_s : float
                        Angular frequency (rad/s) of the coordinate system.
                    w_m : float
                        Rotor speed estimate (electrical rad/s).
                    w_r : float
                        Slip angular frequency (rad/s).
                    psi_s : complex
                        Stator flux estimate (Vs).
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, fbk)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverCfg

   
   Reduced-order flux observer configuration.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars
   :param T_s: Sampling period (s).
   :type T_s: float
   :param sensorless: If True, sensorless mode is used.
   :type sensorless: bool
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed. The default is
               ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` if
               `sensorless` else ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.
   :type k_o: callable, optional

   .. rubric:: Notes

   The pure voltage model corresponds to ``k_o = lambda w_m: 0``, resulting in
   the marginally stable estimation-error dynamics. The current model is
   obtained by setting ``k_o = lambda w_m: 1``.















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentController(par, alpha_c)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   2DOF PI current controller for induction machines.

   This class provides an interface for a current controller for induction
   machines. The gains are initialized based on the desired closed-loop
   bandwidth and the leakage inductance.

   :param par: Machine parameters, contains the leakage inductance `L_sgm` (H).
   :type par: InductionMachineInvGammaPars
   :param alpha_c: Closed-loop bandwidth (rad/s).
   :type alpha_c: float















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReference(par, cfg)

   
   Current reference generation.

   In the base-speed region, the current reference in rotor-flux coordinates
   is given by::

       ref_i_s = nom_psi_R/L_M + 1j*ref_tau_M/(1.5*n_p*abs(psi_R))

   where `nom_psi_R` is the nominal rotor flux magnitude and `psi_R` is the
   estimated rotor flux. The field-weakening operation is based on adjusting
   the flux-producing current component::

       ref_i_s.real = (k_fw/s)*(max_u_s - abs(ref_u_s))

   where `1/s` refers to integration, ``max_u_s = k_u*u_dc/sqrt(3)`` is the
   maximum stator voltage in the linear modulation region, `ref_u_s` is the
   (unlimited) stator voltage reference, and `k_fw` is the field-weakening
   gain. The field-weakening method and its tuning corresponds roughly to
   [#Hin2006]_. Furthermore, the torque-producing current component
   `ref_i_s.imag` is limited based on the maximum stator current and the
   breakdown slip.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars
   :param cfg: Reference generation configuration.
   :type cfg: CurrentReferenceCfg

   .. rubric:: References

   .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled
      induction motor drives equipped with diode rectifier without braking
      resistor," IEEE Trans. Ind. Appl., 2006,
      https://doi.org/10.1109/TIA.2006.880852















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk, ref)

      
      Compute the stator current reference.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Field-weakening based on the unlimited reference voltage.
















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentReferenceCfg

   
   Reference generation configuration.

   This dataclass stores the nominal and limit values needed for reference
   generation. For calculating the rotor flux reference, the machine
   parameters are also required.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars
   :param max_i_s: Maximum stator current (A).
   :type max_i_s: float
   :param nom_u_s: Nominal stator voltage (V). The default is sqrt(2/3)*400.
   :type nom_u_s: float, optional
   :param nom_w_s: Nominal stator angular frequency (rad/s). The default is 2*pi*50.
   :type nom_w_s: float, optional
   :param nom_psi_R: Nominal rotor flux linkage (Vs). The default is
                     `(nom_u_s/nom_w_s)/(1 + L_sgm/L_M)`.
   :type nom_psi_R: float, optional
   :param k_fw: Field-weakening gain (1/H). The default is `2*R_R/(nom_w_s*L_sgm**2)`.
   :type k_fw: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentVectorControl(par, cfg, J=None, T_s=0.00025, sensorless=True)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   Current-vector control for induction machine drives.

   This class provides an interface for current-vector control of induction
   machines. The control system consists of a current reference generator, a
   current controller, a flux observer, and speed controller (optional).

   :param par: Machine parameters.
   :type par: InductionMachineInvGammaPars
   :param cfg: Current reference generator configuration.
   :type cfg: CurrentReferenceCfg
   :param J: Moment of inertia (kgm²). Needed only for the speed controller.
   :type J: float, optional
   :param T_s: Sampling time (s). The default is 250e-6.
   :type T_s: float, optional
   :param sensorless: Enable sensorless control. The default is True.
   :type sensorless: bool, optional

   .. attribute:: observer

      Flux observer.

      :type: Observer

   .. attribute:: current_reference

      Current reference generator.

      :type: CurrentReference

   .. attribute:: current_ctrl

      Current controller. The default is CurrentController(par, 2*np.pi*200).

      :type: CurrentController

   .. attribute:: speed_ctrl

      Speed controller. The default is SpeedController(J, 2*np.pi*4)

      :type: SpeedController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Compute the controller outputs.

      :param fbk: Feedback signals.
      :type fbk: SimpleNamespace

      :returns: **ref** --

                References, containing at least the following fields:

                    T_s : float
                        Next sampling period (s).
                    d_abc : ndarray, shape (3,)
                        Duty ratios.
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzControl(par, cfg, T_s=0.00025)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   Observer-based V/Hz control for induction machines.

   This implements the observer-based V/Hz control method [#Tii2022]_. The
   state-feedback control law is in the alternative form which uses an
   intermediate stator current reference.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param cfg: Control system configuration.
   :type cfg: ObserverBasedVHzControlCfg
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional

   .. rubric:: References

   .. [#Tii2022] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-
      based V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI,
      Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9948057















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

   
   Control system configuration.

   :param nom_psi_s: Nominal stator flux linkage (Vs).
   :type nom_psi_s: float
   :param max_i_s: Maximum stator current (A). The default is inf.
   :type max_i_s: float, optional
   :param k_tau: Torque controller gain. The default is 3.
   :type k_tau: float, optional
   :param alpha_psi: Stator flux control bandwidth (rad/s). The default is 2*pi*20.
   :type alpha_psi: float, optional
   :param alpha_f: Torque high-pass filter bandwidth (rad/s). The default is 2*pi*1.
   :type alpha_f: float, optional
   :param alpha_r: Low-pass-filter bandwidth (rad/s) for slip angular frequency. The
                   default is 2*pi*1.
   :type alpha_r: float, optional
   :param slip_compensation: Enable slip compensation. The default is False.
   :type slip_compensation: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: VHzControl(cfg)

   Bases: :py:obj:`motulator.drive.control.DriveControlSystem`


   
   V/Hz control with the stator current feedback.

   The method is similar to [#Hin2022]_. Open-loop V/Hz control can be
   obtained as a special case by choosing::

       R_s, R_R = 0, 0
       k_u, k_w = 0, 0

   .. rubric:: References

   .. [#Hin2022] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
      volts-per-hertz control for induction motors," IEEE J. Emerg. Sel.
      Topics Power Electron., 2022,
      https://doi.org/10.1109/JESTPE.2021.3060583















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: VHzControlCfg

   
   V/Hz control configuration.
















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

