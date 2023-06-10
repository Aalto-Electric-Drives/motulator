:py:mod:`motulator.model.sm`
============================

.. py:module:: motulator.model.sm

.. autoapi-nested-parse::

   
   Continuous-time synchronous machine models.
















   ..
       !! processed by numpydoc !!


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.sm.Drive
   motulator.model.sm.DriveTwoMassMechanics
   motulator.model.sm.SynchronousMachine
   motulator.model.sm.SynchronousMachineSaturated



Functions
~~~~~~~~~

.. autoapisummary::

   motulator.model.sm.import_syre_data
   motulator.model.sm.plot_flux_map
   motulator.model.sm.plot_flux_vs_current



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

.. py:function:: import_syre_data(fname, add_negative_q_axis=True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format,
   please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which
   the PM flux is along the d axis.

   :param fname: MATLAB file name.
   :type fname: str
   :param add_negative_q_axis: Adds the negative q-axis data based on the symmetry.
   :type add_negative_q_axis: bool, optional

   :returns: * *Bunch object with the following fields defined*
             * **i_s** (*complex ndarray*) -- Stator current data (A).
             * **psi_s** (*complex ndarray*) -- Stator flux linkage data (Vs).
             * **tau_M** (*ndarray*) -- Torque data (Nm).

   .. rubric:: Notes

   Some example data files (including THOR.mat) are available in the SyR-e
   repository, licensed under the Apache License, Version 2.0.















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_map(data)

   
   Plot the flux linkage as function of the current.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_vs_current(data)

   
   Plot the flux vs. current characteristics.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

