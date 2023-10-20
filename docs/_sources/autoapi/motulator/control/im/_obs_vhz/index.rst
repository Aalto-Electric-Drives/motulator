:orphan:

:py:mod:`motulator.control.im._obs_vhz`
=======================================

.. py:module:: motulator.control.im._obs_vhz

.. autoapi-nested-parse::

   Observer-based V/Hz control for induction machine drives.

   This implements the observer-based V/Hz control method described in [#Tii2022]_.
   The state-feedback control law is in the alternative form which uses an
   intermediate stator current reference.

   .. rubric:: References

   .. [#Tii2022] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-based
      V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022,
      https://doi.org/10.1109/ECCE50734.2022.9948057

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im._obs_vhz.ObserverBasedVHzCtrlPars
   motulator.control.im._obs_vhz.ObserverBasedVHzCtrl




.. py:class:: ObserverBasedVHzCtrlPars


   
   Parameters for the control system.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A). The default is inf.
   :type i_s_max: float, optional
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

.. py:class:: ObserverBasedVHzCtrl(par, ctrl_par, T_s=0.00025)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Observer-based V/Hz control for induction machines.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ctrl_par: Control system parameters.
   :type ctrl_par: ObserverBasedVHzCtrlPars
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional

   .. attribute:: observer

      Sensorless reduced-order flux observer in external coordinates.

      :type: SensorlessObserverExtCoord

   .. attribute:: rate_limiter

      Rate limiter for the speed reference.

      :type: RateLimiter

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: callable















   ..
       !! processed by numpydoc !!

