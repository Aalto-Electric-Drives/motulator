:orphan:

:py:mod:`motulator.model._mechanics`
====================================

.. py:module:: motulator.model._mechanics

.. autoapi-nested-parse::

   Continuous-time models for mechanical subsystems.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model._mechanics.Mechanics
   motulator.model._mechanics.MechanicsTwoMass




.. py:class:: Mechanics(J, tau_L_w=lambda w_M: 0 * w_M, tau_L_t=lambda t: 0 * t)

   
   Mechanics subsystem.

   This models an equation of motion for stiff mechanics.

   :param J: Total moment of inertia (kgm²).
   :type J: float
   :param tau_L_w: Load torque (Nm) as a function of speed, `tau_L_w(w_M)`. For example,
                   ``tau_L_w = b*w_M``, where `b` is the viscous friction coefficient. The
                   default is zero, ``lambda w_M: 0*w_M``.
   :type tau_L_w: callable
   :param tau_L_t: Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
                   zero, ``lambda t: 0*t``.
   :type tau_L_t: callable















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, tau_M)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float
      :param tau_M: Electromagnetic torque (Nm).
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      This returns the rotor speed at the end of the sampling period.

      :returns: **w_M0** -- Rotor angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_position()

      
      Measure the rotor angle.

      This returns the rotor angle at the end of the sampling period.

      :returns: **theta_M0** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: MechanicsTwoMass(J_M, J_L, K_S, C_S, tau_L_w=None, tau_L_t=None)

   Bases: :py:obj:`Mechanics`

   
   Two-mass mechanics subsystem.

   This models an equation of motion for two-mass mechanics.

   :param J_M: Motor moment of inertia (kgm²).
   :type J_M: float
   :param J_L: Load moment of inertia (kgm²).
   :type J_L: float
   :param K_S: Shaft torsional stiffness (Nm).
   :type K_S: float
   :param C_S: Shaft torsional damping (Nms).
   :type C_S: float
   :param tau_L_w: Load torque (Nm) as a function of the load speed, `tau_L_w(w_L)`, e.g.,
                   ``tau_L_w = B*w_L``, where `B` is the viscous friction coefficient. The
                   default is zero, ``lambda w_L: 0*w_L``.
   :type tau_L_w: callable
   :param tau_L_t: Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
                   zero, ``lambda t: 0*t``.
   :type tau_L_t: callable















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, w_L, theta_ML, tau_M)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float
      :param w_L: Load angular speed (mechanical rad/s).
      :type w_L: float
      :param theta_ML: Twist angle, theta_M - theta_L (mechanical rad).
      :type theta_ML: float
      :param tau_M: Electromagnetic torque (Nm).
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 4















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_speed()

      
      Measure the load speed.

      This returns the load speed at the end of the sampling period.

      :returns: **w_L0** -- Load angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_position()

      
      Measure the load angle.

      This returns the load angle at the end of the sampling period.

      :returns: **theta_L0** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


