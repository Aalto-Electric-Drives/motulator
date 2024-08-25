motulator.drive.control
=======================

.. py:module:: motulator.drive.control

.. autoapi-nested-parse::

   
   Controllers for machine drives.
















   ..
       !! processed by numpydoc !!


Subpackages
-----------

.. toctree::
   :maxdepth: 1

   /autoapi/motulator/drive/control/im/index
   /autoapi/motulator/drive/control/sm/index


Classes
-------

.. autoapisummary::

   motulator.drive.control.DriveControlSystem
   motulator.drive.control.SpeedController


Package Contents
----------------

.. py:class:: DriveControlSystem(par, T_s, sensorless)

   Bases: :py:obj:`motulator.common.control.ControlSystem`, :py:obj:`abc.ABC`


   
   Base class for drive control systems.

   This base class provides typical functionalities for control of electric
   machine drives. This can be used both in speed-control and torque-control
   modes.

   :param par: Machine model parameters.
   :type par: motulator.drive.control.im.ModelPars |          motulator.drive.control.sm.ModelPars
   :param T_s: Sampling period (s).
   :type T_s: float
   :param sensorless: If True, sensorless control mode is used.
   :type sensorless: bool

   .. attribute:: ref

      References, possibly containing either of the following fields:

          w_m : callable
              Speed reference (electrical rad/s) as a function of time (s).
              This signal is needed in speed-control mode.
          tau_M : callable
              Torque reference (Nm) as a function of time (s). This signal
              is needed in torque-control mode.

      :type: SimpleNamespace

   .. attribute:: observer

      State observer can be None or an instance of either
      `motulator.drive.control.im.Observer` or
      `motulator.drive.control.sm.Observer`
      depending on the machine type. The default is None.

      :type: motulator.drive.control.im.Observer |                motulator.drive.control.sm.Observer | None

   .. attribute:: speed_ctrl

      Speed controller. The default is None.

      :type: SpeedController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: get_electrical_measurements(fbk, mdl)

      
      Measure the currents and voltages.

      :param fbk: Measured signals are added to this object.
      :type fbk: SimpleNamespace
      :param mdl: Continuous-time system model.
      :type mdl: Model

      :returns: **fbk** --

                Measured signals, containing the following fields:

                    u_dc : float
                        DC-bus voltage (V).
                    i_ss : complex
                        Stator current (A) in stator coordinates.
                    u_ss : complex
                        Realized stator voltage (V) in stator coordinates. This
                        signal is obtained from the PWM.
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_mechanical_measurements(fbk, mdl)

      
      Measure the speed and position.

      :param fbk: Measured signals are added to this object.
      :type fbk: SimpleNamespace
      :param mdl: Continuous-time system model.
      :type mdl: Model

      :returns: **fbk** --

                Measured signals, containing the following fields:

                    w_m : float
                        Rotor speed (electrical rad/s).
                    theta_m : float
                        Rotor position (electrical rad).
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: get_torque_reference(fbk, ref)

      
      Get the torque reference in vector control.

      This method can be used in vector control to get the torque reference
      from the speed controller. If the speed controller method `speed_ctrl`
      is None, the torque reference is obtained directly from the reference.

      :param fbk: Feedback signals. In speed-control mode, the measured or estimated
                  rotor speed `w_m` is used to compute the torque reference.
      :type fbk: SimpleNamespace
      :param ref: Reference signals, containing the digital time `t`. The speed and
                  torque references are added to this object.
      :type ref: SimpleNamespace

      :returns: **ref** --

                Reference signals, containing the following fields:

                    w_m : float
                        Speed reference (electrical rad/s).
                    tau_M : float
                        Torque reference (Nm).
      :rtype: SimpleNamespace















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

