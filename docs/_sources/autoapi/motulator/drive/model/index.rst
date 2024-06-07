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
   motulator.drive.model.Delay
   motulator.drive.model.Simulation
   motulator.drive.model.Drive
   motulator.drive.model.DriveWithLCFilter
   motulator.drive.model.FrequencyConverter
   motulator.drive.model.Inverter
   motulator.drive.model.InductionMachine
   motulator.drive.model.InductionMachineInvGamma
   motulator.drive.model.SynchronousMachine
   motulator.drive.model.Mechanics
   motulator.drive.model.TwoMassMechanics
   motulator.drive.model.LCFilter


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

   >>> from motulator.model import CarrierComparison
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

.. py:class:: Delay(length=1, elem=3)

   
   Computational delay modeled as a ring buffer.

   :param length: Length of the buffer in samples. The default is 1.
   :type length: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Simulation(mdl=None, ctrl=None)

   
   Simulation environment.

   Each simulation object has a system model object and a controller object.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time controller.
   :type ctrl: Ctrl















   ..
       !! processed by numpydoc !!

   .. py:method:: simulate(t_stop=1, max_step=np.inf)

      
      Solve the continuous-time model and call the discrete-time controller.

      :param t_stop: Simulation stop time. The default is 1.
      :type t_stop: float, optional
      :param max_step: Max step size of the solver. The default is inf.
      :type max_step: float, optional

      .. rubric:: Notes

      Other options of `solve_ivp` could be easily used if needed, but, for
      simplicity, only `max_step` is included as an option of this method.















      ..
          !! processed by numpydoc !!


   .. py:method:: save_mat(name='sim')

      
      Save the simulation data into MATLAB .mat files.

      :param name: Name for the simulation instance. The default is `sim`.
      :type name: str, optional















      ..
          !! processed by numpydoc !!


.. py:class:: Drive(converter=None, machine=None, mechanics=None)

   Bases: :py:obj:`motulator.common.model.Model`


   
   Continuous-time model for machine drives.

   This interconnects the subsystems of a machine drive and provides an
   interface to the solver.

   :param converter: Converter model.
   :type converter: Inverter | FrequencyConverter
   :param machine: Machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanics model.
   :type mechanics: Mechanics | TwoMassMechanics















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
   :type converter: Inverter | FrequencyConverter
   :param machine: Machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanics model.
   :type mechanics: Mechanics | TwoMassMechanics
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


.. py:class:: FrequencyConverter(L, C, U_g, f_g)

   Bases: :py:obj:`Inverter`


   
   Frequency converter.

   This extends the Inverter class with models for a strong grid, a
   three-phase diode-bridge rectifier, an LC filter.

   :param L: DC-bus inductance (H).
   :type L: float
   :param C: DC-bus capacitance (F).
   :type C: float
   :param U_g: Grid voltage (V, line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency (Hz).
   :type f_g: float















   ..
       !! processed by numpydoc !!

   .. py:property:: u_dc
      
      DC-bus voltage.
















      ..
          !! processed by numpydoc !!


   .. py:method:: grid_voltages(t)

      
      Compute three-phase grid voltages.

      :param t: Time (s).
      :type t: float

      :returns: **u_g_abc** -- Phase voltages (V).
      :rtype: ndarray of floats, shape (3,)















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















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


.. py:class:: Inverter(u_dc)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Lossless three-phase inverter with constant DC-bus voltage.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float















   ..
       !! processed by numpydoc !!

   .. py:property:: u_dc
      
      DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:property:: u_cs
      
      AC-side voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:property:: i_dc
      
      DC-side current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure the DC-bus voltage.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachine(n_p, R_s, R_r, L_ell, L_s)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Γ-equivalent model of an induction machine.

   An induction machine is modeled using the Γ-equivalent model [#Sle1989]_.
   The model is implemented in stator coordinates. The flux linkages are used
   as state variables. The stator inductance `L_s` can either be constant or
   a function of the stator flux magnitude::

       L_s = L_s(abs(psi_ss))

   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H) or a callable L_s = L_s(abs(psi_ss)).
   :type L_s: float | callable
   :param n_p: Number of pole pairs.
   :type n_p: int

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


   .. py:property:: tau_M
      
      Electromagnetic torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















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

.. py:class:: SynchronousMachine(n_p, R_s, L_d=None, L_q=None, psi_f=None, i_s=None, psi_s0=None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Synchronous machine model.

   This models a synchronous machine in rotor coordinates. The stator flux
   linkage and the electrical angle of the rotor are the state variables.

   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: PM-flux linkage (Vs).
   :type psi_f: float
   :param n_p: Number of pole pairs.
   :type n_p: int















   ..
       !! processed by numpydoc !!

   .. py:property:: i_s
      
      Stator current (A).
















      ..
          !! processed by numpydoc !!


   .. py:property:: tau_M
      
      Electromagnetic torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















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


.. py:class:: Mechanics(J, tau_L_w=lambda w_M: 0 * w_M, tau_L_t=lambda t: 0 * t)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
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

   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      :returns: **w_M** -- Rotor angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure the rotor angle.

      :returns: **theta_M** -- Rotor angle (mechanical rad).
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


.. py:class:: TwoMassMechanics(J_M, J_L, K_S, C_S, tau_L_w=None, tau_L_t=None)

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

   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_speed()

      
      Measure the load speed.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_position()

      
      Measure the load angle.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


.. py:class:: LCFilter(L, C, R=0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   LC-filter model.

   :param L: Inductance (H).
   :type L: float
   :param C: Capacitance (F).
   :type C: float
   :param R: Series resistance (Ω) of the inductor. The default is 0.
   :type R: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_voltages()

      
      Measure the capacitor phase voltages.
















      ..
          !! processed by numpydoc !!


