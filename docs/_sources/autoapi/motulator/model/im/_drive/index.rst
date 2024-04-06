:orphan:

:py:mod:`motulator.model.im._drive`
===================================

.. py:module:: motulator.model.im._drive

.. autoapi-nested-parse::

   Continuous-time models for induction machine drives.

   Peak-valued complex space vectors are used. The space vector models are
   implemented in stator coordinates.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.im._drive.InductionMachine
   motulator.model.im._drive.InductionMachineSaturated
   motulator.model.im._drive.InductionMachineInvGamma
   motulator.model.im._drive.Drive
   motulator.model.im._drive.DriveWithDiodeBridge
   motulator.model.im._drive.DriveTwoMassMechanics




.. py:class:: InductionMachine(n_p, R_s, R_r, L_ell, L_s)


   
   Γ-equivalent model of an induction machine.

   An induction machine is modeled using the Γ-equivalent model [#Sle1989]_.
   The model is implemented in stator coordinates. The flux linkages are used
   as state variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H).
   :type L_s: float

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic
   saturation model in a straightforward manner. If the magnetic saturation is
   omitted, the Γ model is mathematically identical to the inverse-Γ and T
   models [#Sle1989]_.

   .. rubric:: References

   .. [#Sle1989] Slemon, "Modelling of induction machines for electric drives,"
      IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Compute the stator and rotor currents.

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current (A).
                * **i_rs** (*complex*) -- Rotor current (A).















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_ss, psi_rs)

      
      Magnetic model.

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current (A).
                * **i_rs** (*complex*) -- Rotor current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_ss, psi_rs, u_ss, w_M)

      
      Compute the state derivatives.

      :param psi_ss: Stator flux linkage (Vs).
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage (Vs).
      :type psi_rs: complex
      :param u_ss: Stator voltage (V).
      :type u_ss: complex
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_ss, dpsi_rs]
                * **i_ss** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).

      .. rubric:: Notes

      In addition to the state derivatives, this method also returns the
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


.. py:class:: InductionMachineSaturated(n_p, R_s, R_r, L_ell, L_s)


   Bases: :py:obj:`InductionMachine`

   
   Γ-equivalent model of an induction machine model with main-flux saturation.

   This extends the InductionMachine class with a main-flux magnetic saturation
   model::

       L_s = L_s(abs(psi_ss))

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H) as a function of the stator-flux magnitude.
   :type L_s: callable















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachineInvGamma(n_p, R_s, R_R, L_sgm, L_M)


   Bases: :py:obj:`InductionMachine`

   
   Inverse-Γ model of an induction machine.

   This extends the InductionMachine class (based on the Γ model) by providing
   an interface for the inverse-Γ model parameters. Linear magnetics are
   assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_R: Rotor resistance (Ω).
   :type R_R: float
   :param L_sgm: Leakage inductance (H).
   :type L_sgm: float
   :param L_M: Magnetizing inductance (H).
   :type L_M: float















   ..
       !! processed by numpydoc !!

.. py:class:: Drive(machine=None, mechanics=None, converter=None)


   
   Continuous-time model for an induction machine drive.

   This interconnects the subsystems of an induction machine drive and provides
   an interface to the solver. More complicated systems could be modeled using
   a similar template.

   :param machine: Induction machine model.
   :type machine: InductionMachine | InductionMachineSaturated
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
      :type sol: Bunch object















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


.. py:class:: DriveWithDiodeBridge(machine=None, mechanics=None, converter=None)


   Bases: :py:obj:`Drive`

   
   Induction machine drive equipped with a diode bridge.

   This model extends the Drive class with a model for a three-phase diode
   bridge fed from stiff supply voltages. The DC bus is modeled as an inductor
   and a capacitor.

   :param machine: Induction machine model.
   :type machine: InductionMachine | InductionMachineSaturated
   :param mechanics: Mechanics model.
   :type mechanics: Mechanics
   :param converter: Frequency converter model.
   :type converter: FrequencyConverter















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


.. py:class:: DriveTwoMassMechanics(machine=None, mechanics=None, converter=None)


   Bases: :py:obj:`Drive`

   
   Induction machine drive with two-mass mechanics.

   This interconnects the subsystems of an induction machine drive and provides
   an interface to the solver.

   :param machine: Induction machine model.
   :type machine: InductionMachine | InductionMachineSaturated
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


