:py:mod:`motulator.model.mech`
==============================

.. py:module:: motulator.model.mech

.. autoapi-nested-parse::

   Continuous-time models for mechanical subsystems.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.mech.Mechanics
   motulator.model.mech.MechanicsTwoMass




.. py:class:: Mechanics(J=0.015, tau_L_w=lambda w_M: 0, tau_L_t=lambda t: 0)

   
   Mechanics subsystem.

   This models an equation of motion for stiff mechanics.

   :param J: Total moment of inertia.
   :type J: float
   :param tau_L_w: Load torque as function of speed, `tau_L_w(w_M)`. For example,
                   tau_L_w = b*w_M, where b is the viscous friction coefficient.
   :type tau_L_w: function
   :param tau_L_t: Load torque as a function of time, `tau_L_t(t)`.
   :type tau_L_t: function















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, tau_M)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float
      :param tau_M: Electromagnetic torque.
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      This returns the rotor speed at the end of the sampling period.

      :returns: **w_M0** -- Rotor angular speed (in mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_position()

      
      Measure the rotor angle.

      This returns the rotor angle at the end of the sampling period.

      :returns: **theta_M0** -- Rotor angle (in mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: MechanicsTwoMass(J_M=0.005, J_L=0.005, K_S=700.0, C_S=0.13, tau_L_w=lambda w_M: 0, tau_L_t=lambda t: 0)

   Bases: :py:obj:`Mechanics`

   
   Two-mass mechanics subsystem.

   This models an equation of motion for two-mass mechanics.

   :param J_M: Moment of inertia of the motor.
   :type J_M: float
   :param J_L: Moment of inertia of the load.
   :type J_L: float
   :param K_S: Torsional stiffness of the shaft.
   :type K_S: float
   :param C_S: Torsional damping of the shaft.
   :type C_S: float
   :param tau_L_w: Load torque as function of the load speed, `tau_L_w(w_L)`. For example,
                   tau_L_w = b*w_L, where b is the viscous friction coefficient.
   :type tau_L_w: function
   :param tau_L_t: Load torque as a function of time, `tau_L_t(t)`.
   :type tau_L_t: function















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, w_L, theta_ML, tau_M)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float
      :param w_L: Load angular speed (in mechanical rad/s).
      :type w_L: float
      :param theta_ML: Twist angle, theta_M - theta_L (in mechanical rad).
      :type theta_ML: float
      :param tau_M: Electromagnetic torque.
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 4















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_speed()

      
      Measure the load speed.

      This returns the load speed at the end of the sampling period.

      :returns: **w_L0** -- Load angular speed (in mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_position()

      
      Measure the load angle.

      This returns the load angle at the end of the sampling period.

      :returns: **theta_L0** -- Rotor angle (in mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


