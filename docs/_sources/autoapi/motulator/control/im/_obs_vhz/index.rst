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
   motulator.control.im._obs_vhz.FluxObserver




.. py:class:: ObserverBasedVHzCtrlPars

   
   Parameters for the control system.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A). The default is `inf`.
   :type i_s_max: float, optional
   :param k_tau: Torque controller gain. The default is `3`.
   :type k_tau: float, optional
   :param alpha_psi: Stator flux control bandwidth (rad/s). The default is `2*pi*20`.
   :type alpha_psi: float, optional
   :param alpha_f: Torque high-pass filter bandwidth (rad/s). The default is `2*pi*1`.
   :type alpha_f: float, optional
   :param alpha_r: Low-pass-filter bandwidth (rad/s) for slip angular frequency. The
                   default is `2*pi*1`.
   :type alpha_r: float, optional
   :param slip_compensation: Enable slip compensation. The default is `False`.
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

.. py:class:: FluxObserver(par, alpha_o, b=None)

   
   Sensorless reduced-order flux observer in external coordinates.

   This is a sensorless reduced-order flux observer in synchronous coordinates
   for an induction machine. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the linearized
   estimation error dynamics. This implementation operates in external
   coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Speed-estimation bandwidth (rad/s).
   :type alpha_o: float
   :param b: Coefficient (rad/s) of the characteristic polynomial as a function of
             the rotor angular speed estimate. The default is
             ``lambda w_m: R_R/L_M + .4*abs(w_m)``.
   :type b: callable, optional

   .. attribute:: psi_R

      Rotor flux estimate (Vs).

      :type: complex

   .. attribute:: w_m

      Rotor angular speed estimate (electrical rad/s).

      :type: float

   .. rubric:: Notes

   The characteristic polynomial of the observer in synchronous coordinates is
   ``s**2 + b*s + w_s**2``.















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, u_s, i_s, w_s)

      
      Update the states.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V) in synchronous coordinates.
      :type u_s: complex
      :param i_s: Stator current (A) in synchronous coordinates.
      :type i_s: complex
      :param w_s: Angular frequency (rad/s) of the coordinate system.
      :type w_s: float















      ..
          !! processed by numpydoc !!


