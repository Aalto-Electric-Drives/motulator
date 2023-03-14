:py:mod:`motulator.model.sm_drive`
==================================

.. py:module:: motulator.model.sm_drive

.. autoapi-nested-parse::

   Continuous-time models for synchronous motor drives.

   Peak-valued complex space vectors are used. The default values correspond to a
   2.2-kW permanent-magnet synchronous motor.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.sm_drive.SynchronousMotorDrive
   motulator.model.sm_drive.SynchronousMotorDriveTwoMass




.. py:class:: SynchronousMotorDrive(motor=None, mech=None, conv=None)

   
   Continuous-time model for a synchronous motor drive.

   This interconnects the subsystems of a synchronous motor drive and provides
   an interface to the solver. More complicated systems could be modeled using
   a similar template.

   :param motor: Synchronous motor model.
   :type motor: SynchronousMotor
   :param mech: Mechanics model.
   :type mech: Mechanics
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Get the initial values.

      :returns: **x0** -- Initial values of the state variables.
      :rtype: complex list, length 3















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Set the initial values.

      :param x0: Initial values of the state variables.
      :type x0: complex ndarray















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Compute the complete state derivative list for the solver.

      :param t: Time.
      :type t: float
      :param x: State vector.
      :type x: complex ndarray

      :returns: State derivatives.
      :rtype: complex list















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Save the solution.

      :param sol: Solution from the solver.
      :type sol: Bunch object















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorDriveTwoMass(motor=None, mech=None, conv=None)

   Bases: :py:obj:`SynchronousMotorDrive`

   
   Synchronous motor drive with two-mass mechanics.

   This interconnects the subsystems of a synchronous motor drive and provides
   an interface to the solver.

   :param motor: Synchronous motor model.
   :type motor: SynchronousMotor
   :param mech: Mechanics model.
   :type mech: MechanicsTwoMass
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Override the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!


