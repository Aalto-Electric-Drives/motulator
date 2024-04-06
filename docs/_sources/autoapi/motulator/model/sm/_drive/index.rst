:orphan:

:py:mod:`motulator.model.sm._drive`
===================================

.. py:module:: motulator.model.sm._drive

.. autoapi-nested-parse::

   Continuous-time models for synchronous motor drives.

   Peak-valued complex space vectors are used.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.sm._drive.SynchronousMachine
   motulator.model.sm._drive.SynchronousMachineSaturated
   motulator.model.sm._drive.Drive
   motulator.model.sm._drive.DriveTwoMassMechanics




.. py:class:: SynchronousMachine(n_p, R_s, L_d, L_q, psi_f)


   
   Synchronous machine model.

   This models a synchronous machine in rotor coordinates. The stator flux
   linkage and the electrical angle of the rotor are the state variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: PM-flux linkage (Vs).
   :type psi_f: float















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Compute the stator current.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: **i_s** -- Stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_s)

      
      Magnetic model.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_s, u_s, w_M)

      
      Compute the state derivative.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex
      :param u_s: Stator voltage (V).
      :type u_s: complex
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_s, dtheta_m0]
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).

      .. rubric:: Notes

      In addition to the state derivative, this method also returns the
      output signals (stator current `i_ss` and torque `tau_M`) needed for
      interconnection with other subsystems. This avoids overlapping
      computation in simulation.















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the phase currents at the end of the sampling period.

      :returns: **i_s_abc** -- Phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachineSaturated(n_p, R_s, current, psi_s0=0j)


   Bases: :py:obj:`SynchronousMachine`

   
   Model of a saturated synchronous machine.

   This overrides the linear magnetics model of the SynchronousMachine class
   with a generic saturation model::

       i_s = i_s(psi_s)

   The saturation model could be an analytical function or a look-up table.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param current: Function that computes the stator current `i_s` as a function of the
                   stator flux linkage `psi_s`.
   :type current: callable
   :param psi_s0: Initial value of the stator flux linkage (Vs). For PM machines, this
                  should be solved from the the saturation model. The default is 0j.
   :type psi_s0: complex, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Drive(machine=None, mechanics=None, converter=None)


   
   Continuous-time model for a synchronous machine drive.

   This interconnects the subsystems of a synchronous machine drive and
   provides an interface to the solver. More complicated systems could be
   modeled using a similar template.

   :param machine: Synchronous machine model.
   :type machine: SynchronousMachine
   :param mechanics: Mechanics model.
   :type mechanics: Mechanics
   :param converter: Inverter model.
   :type converter: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: clear()

      
      Clear the simulation data of the system model.

      This method is automatically run when the instance for the system model
      is created. It can also be used in the case of repeated simulations to
      clear the data from the previous simulation run.















      ..
          !! processed by numpydoc !!

   .. py:method:: get_initial_values()

      
      Get the initial values.

      :returns: **x0** -- Initial values of the state variables.
      :rtype: complex list, length 4















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Set the initial values.

      :param t0: Initial time (s).
      :type t0: float
      :param x0: Initial values of the state variables.
      :type x0: complex ndarray















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Compute the complete state derivative list for the solver.

      :param t: Time (s).
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
      :type sol: Bunch















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


.. py:class:: DriveTwoMassMechanics(machine=None, mechanics=None, converter=None)


   Bases: :py:obj:`Drive`

   
   Synchronous machine drive with two-mass mechanics.

   This interconnects the subsystems of a synchronous machine drive and
   provides an interface to the solver.

   :param machine: Synchronous machine model.
   :type machine: SynchronousMachine
   :param mechanics: Mechanics model.
   :type mechanics: MechanicsTwoMass
   :param converter: Inverter model.
   :type converter: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: clear()

      
      Extend the base class.
















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


