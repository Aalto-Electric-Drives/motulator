motulator.drive.model
=====================

.. py:module:: motulator.drive.model

.. autoapi-nested-parse::

   
   Continuous-time machine drive models.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.model.CarrierComparison
   motulator.drive.model.Drive
   motulator.drive.model.DriveWithLCFilter
   motulator.drive.model.ExternalRotorSpeed
   motulator.drive.model.FrequencyConverter
   motulator.drive.model.InductionMachine
   motulator.drive.model.LCFilter
   motulator.drive.model.Simulation
   motulator.drive.model.StiffMechanicalSystem
   motulator.drive.model.SynchronousMachine
   motulator.drive.model.TwoMassMechanicalSystem
   motulator.drive.model.VoltageSourceConverter


Package Contents
----------------

.. py:class:: CarrierComparison(N=2**12, return_complex=True)

   
   Carrier comparison.

   This computes the the switching states and their durations based on the
   duty ratios. Instead of searching for zero crossings, the switching
   instants are explicitly computed in the beginning of each sampling period,
   allowing faster simulations.

   :param N: Amount of the counter quantization levels. The default is 2**12.
   :type N: int, optional
   :param return_complex: Complex switching state space vectors are returned if True. Otherwise
                          phase switching states are returned. The default is True.
   :type return_complex: bool, optional

   .. rubric:: Examples

   >>> from motulator.common.model import CarrierComparison
   >>> carrier_cmp = CarrierComparison(return_complex=False)
   >>> # First call gives rising edges
   >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.4, .2, .8])
   >>> # Durations of the switching states
   >>> t_steps
   array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
   >>> # Switching states
   >>> q_c_abc
   array([[0, 0, 0],
          [0, 0, 1],
          [1, 0, 1],
          [1, 1, 1]])
   >>> # Second call gives falling edges
   >>> t_steps, q_c_abc = carrier_cmp(.001, [.4, .2, .8])
   >>> t_steps
   array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
   >>> q_c_abc
   array([[1, 1, 1],
          [1, 0, 1],
          [0, 0, 1],
          [0, 0, 0]])
   >>> # Sum of the step times equals T_s
   >>> np.sum(t_steps)
   0.001
   >>> # 50% duty ratios in all phases
   >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.5, .5, .5])
   >>> t_steps
   array([0.0005, 0.    , 0.    , 0.0005])
   >>> q_c_abc
   array([[0, 0, 0],
          [0, 0, 0],
          [0, 0, 0],
          [1, 1, 1]])















   ..
       !! processed by numpydoc !!

.. py:class:: Drive(converter=None, machine=None, mechanics=None)

   Bases: :py:obj:`motulator.common.model.Model`


   
   Continuous-time model for machine drives.

   This interconnects the subsystems of a machine drive and provides an
   interface to the solver.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter | FrequencyConverter
   :param machine: Machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanical subsystem model.
   :type mechanics: ExternalRotorSpeed | StiffMechanicalSystem |                TwoMassMechanicalSystem















   ..
       !! processed by numpydoc !!

   .. py:method:: interconnect(_)

      
      Interconnect the subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


.. py:class:: DriveWithLCFilter(converter=None, machine=None, mechanics=None, lc_filter=None)

   Bases: :py:obj:`motulator.common.model.Model`


   
   Machine drive with an output LC filter.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter | FrequencyConverter
   :param machine: Machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanical subsystem model.
   :type mechanics: ExternalRotorSpeed | StiffMechanicalSystem |                TwoMassMechanicalSystem
   :param lc_filter: LC-filter model.
   :type lc_filter: LCFilter















   ..
       !! processed by numpydoc !!

   .. py:method:: interconnect(_)

      
      Interconnect the subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


.. py:class:: ExternalRotorSpeed(w_M=lambda t: 0 * t)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Integrate the rotor angle from the externally given rotor speed.

   :param w_M: Rotor speed (rad/s) as a function of time, `w_M(t)`. The default is
               zero, ``lambda t: 0*t``.
   :type w_M: callable















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_position()

      
      Measure the rotor angle.

      :returns: **theta_M** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      :returns: **w_M** -- Rotor angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(C_dc, L_dc, U_g, f_g)

   Bases: :py:obj:`VoltageSourceConverter`


   
   Frequency converter with a six-pulse diode bridge.

   A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The
   diode bridge is connected to the voltage-source inverter. The inductance of
   the grid is omitted.

   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float
   :param L_dc: DC-bus inductance (H).
   :type L_dc: float
   :param U_g: Grid voltage (V, line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency (Hz).
   :type f_g: float















   ..
       !! processed by numpydoc !!

   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachine(par)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Γ-equivalent model of an induction machine.

   An induction machine is modeled using the Γ-equivalent model [#Sle1989]_.
   The stator inductance `L_s` can either be constant or a function of the
   stator flux magnitude::

       L_s = L_s(abs(psi_ss))

   :param par:
   :type par: InductionMachinePars

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic
   saturation model in a straightforward manner. If the magnetic saturation is
   omitted, the Γ model is mathematically identical to the inverse-Γ and T
   models [#Sle1989]_.

   .. rubric:: References

   .. [#Sle1989] Slemon, "Modelling of induction machines for electric
      drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251















   ..
       !! processed by numpydoc !!

   .. py:property:: L_s
      
      Stator inductance (H).
















      ..
          !! processed by numpydoc !!


   .. py:property:: i_rs
      
      Rotor current (A).
















      ..
          !! processed by numpydoc !!


   .. py:property:: i_ss
      
      Stator current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the phase currents.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:property:: tau_M
      
      Electromagnetic torque (Nm).
















      ..
          !! processed by numpydoc !!


.. py:class:: LCFilter(L_f, C_f, R_f=0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   LC-filter model.

   :param L_f: Filter inductance (H).
   :type L_f: float
   :param C_f: Filter capacitance (F).
   :type C_f: float
   :param R_f: Series resistance (Ω) of the inductor. The default is 0.
   :type R_f: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor phase voltages.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: Simulation(mdl=None, ctrl=None)

   
   Simulation environment.

   Each simulation object has a system model object and a control system
   object.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time control system.
   :type ctrl: ControlSystem















   ..
       !! processed by numpydoc !!

   .. py:method:: save_mat(name='sim')

      
      Save the simulation data into MATLAB .mat files.

      :param name: Name for the simulation instance. The default is `sim`.
      :type name: str, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: simulate(t_stop=1, max_step=np.inf)

      
      Solve the continuous-time system model and call the control system.

      :param t_stop: Simulation stop time. The default is 1.
      :type t_stop: float, optional
      :param max_step: Max step size of the solver. The default is inf.
      :type max_step: float, optional

      .. rubric:: Notes

      Other options of `solve_ivp` could be easily used if needed, but, for
      simplicity, only `max_step` is included as an option of this method.















      ..
          !! processed by numpydoc !!


.. py:class:: StiffMechanicalSystem(J, B_L=0, tau_L=lambda t: 0 * t)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Stiff mechanical system.

   :param J: Total moment of inertia (kgm²).
   :type J: float
   :param B_L: Friction coefficient (Nm/(rad/s)) that can be constant, corresponding
               to viscous friction, or an arbitrary function of the rotor speed. For
               example, choosing ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic
               load torque ``k*w_M**2``. The default is ``B_L = 0``.
   :type B_L: float | callable
   :param tau_L: External load torque (Nm) as a function of time, `tau_L_t(t)`. The
                 default is zero, ``lambda t: 0*t``.
   :type tau_L: callable















   ..
       !! processed by numpydoc !!

   .. py:property:: B_L
      
      Friction coefficient (Nm/(rad/s)).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure the rotor angle.

      :returns: **theta_M** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      :returns: **w_M** -- Rotor angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachine(par, i_s=None, psi_s0=None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Synchronous machine model.

   :param par: Machine parameters.
   :type par: SynchronousMachinePars
   :param i_s: Stator current (A) as a function of the stator flux linkage (A) in
               order to model the magnetic saturation. If this function is given, the
               stator current is computed using this function instead of constants
               `par.L_d`, `par.L_q`, and `par.psi_f`.
   :type i_s: callable, optional
   :param psi_s0: Initial stator flux linkage (Vs). If not given, `par.psi_f` is used.
   :type psi_s0: float, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: i_s
      
      Stator current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the phase currents.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:property:: tau_M
      
      Electromagnetic torque (Nm).
















      ..
          !! processed by numpydoc !!


.. py:class:: TwoMassMechanicalSystem(par, tau_L=lambda t: 0 * t)

   Bases: :py:obj:`StiffMechanicalSystem`


   
   Two-mass mechanical subsystem.

   :param par: Two-mass mechanical system parameters.
   :type par: TwoMassMechanicalSystemPars
   :param tau_L: Load torque (Nm) as a function of time, `tau_L(t)`. The default is
                 zero, ``lambda t: 0*t``.
   :type tau_L: callable















   ..
       !! processed by numpydoc !!

   .. py:property:: B_L
      
      Friction coefficient (Nm/(rad/s)).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_position()

      
      Measure the load angle.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_speed()

      
      Measure the load speed.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: VoltageSourceConverter(u_dc, C_dc=None, i_dc=lambda t: None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Lossless three-phase voltage-source converter.

   :param u_dc: DC-bus voltage (V). If the DC-bus capacitor is modeled, this value is
                used as the initial condition.
   :type u_dc: float
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional
   :param i_dc: External current (A) fed to the DC bus. Needed if `C_dc` is not None.
   :type i_dc: callable, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: i_dc_int
      
      Converter-side DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure the converter DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Set input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:property:: u_cs
      
      AC-side voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:property:: u_dc
      
      DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


