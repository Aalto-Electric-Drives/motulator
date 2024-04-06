:orphan:

:py:mod:`motulator.control.sm._obs_vhz`
=======================================

.. py:module:: motulator.control.sm._obs_vhz

.. autoapi-nested-parse::

   Observer-based V/Hz control of synchronous motor drives.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._obs_vhz.ObserverBasedVHzCtrlPars
   motulator.control.sm._obs_vhz.ObserverBasedVHzCtrl




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

