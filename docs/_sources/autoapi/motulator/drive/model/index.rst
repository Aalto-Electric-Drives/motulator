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
   motulator.drive.model.CompositeModel
   motulator.drive.model.DCBus
   motulator.drive.model.Drive
   motulator.drive.model.ExternalRotorSpeed
   motulator.drive.model.GridConnectedMachine
   motulator.drive.model.InductionMachine
   motulator.drive.model.InductionMachineInvGammaPars
   motulator.drive.model.InductionMachinePars
   motulator.drive.model.LCFilter
   motulator.drive.model.MechanicalSystem
   motulator.drive.model.SaturatedSynchronousMachinePars
   motulator.drive.model.Sensor
   motulator.drive.model.Simulation
   motulator.drive.model.SimulationResults
   motulator.drive.model.SpatialSaturatedSynchronousMachinePars
   motulator.drive.model.SynchronousMachine
   motulator.drive.model.SynchronousMachinePars
   motulator.drive.model.ThreePhaseSource
   motulator.drive.model.TwoMassMechanicalSystem
   motulator.drive.model.VoltageSourceConverter
   motulator.drive.model.ZOH


Package Contents
----------------

.. py:class:: CarrierComparison(t_d = 0.0, delay = None, single_update = False, N = 2**12)

   Bases: :py:obj:`TwoLevelModulator`\ [\ :py:obj:`collections.abc.Sequence`\ [\ :py:obj:`float`\ ]\ ]


   
   Carrier comparison of a voltage-source converter.

   This computes the gate signals and their durations based on the duty ratios. Instead
   of searching for zero crossings, the switching instants are explicitly computed in
   the beginning of each half carrier period, allowing faster simulations. The dead
   time delays the turn-on of the switches, i.e., each leg is blanked for `t_d` after
   its switching instant.

   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
   :param delay: Computational delay in half carrier periods, defaults to one sampling period,
                 see `Modulator`.
   :type delay: int, optional
   :param single_update: Sample and update the duty ratios once per carrier period, defaults to False,
                         see `Modulator`.
   :type single_update: bool, optional
   :param N: Amount of the counter quantization levels, defaults to 2**12.
   :type N: int, optional

   .. rubric:: Examples

   >>> from motulator.common.model import CarrierComparison
   >>> carrier_cmp = CarrierComparison(delay=0)
   >>> # First call gives rising edges
   >>> t_steps, gates = carrier_cmp(1e-3, [.4, .2, .8])
   >>> # Durations of the switching states
   >>> t_steps
   array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
   >>> # Gate signals of the upper switches
   >>> gates[:, 0]
   array([[0, 0, 0],
          [0, 0, 1],
          [1, 0, 1],
          [1, 1, 1]])
   >>> # Second call gives falling edges
   >>> t_steps, gates = carrier_cmp(1e-3, [.4, .2, .8])
   >>> t_steps
   array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
   >>> gates[:, 0]
   array([[1, 1, 1],
          [1, 0, 1],
          [0, 0, 1],
          [0, 0, 0]])
   >>> # Sum of the step times equals T_s
   >>> float(np.sum(t_steps))
   0.001
   >>> # Dead time blanks each leg after its switching instant
   >>> carrier_cmp = CarrierComparison(t_d=1e-4, delay=0)
   >>> t_steps, gates = carrier_cmp(1e-3, [.5, .5, .5])
   >>> np.round(t_steps / 1e-3, 3)  # In ms
   array([0.5, 0.1, 0.4])
   >>> gates[:, 0]  # Gate signals
   array([[0, 0, 0],
          [0, 0, 0],
          [1, 1, 1]])
   >>> gates[:, 1]  # Blanking signals
   array([[0, 0, 0],
          [1, 1, 1],
          [0, 0, 0]])















   ..
       !! processed by numpydoc !!

   .. py:method:: schedule(T_half, command)

      
      Compute the gate signals and their durations.

      :param T_half: Half carrier period (s).
      :type T_half: float
      :param command: Duty ratios in the range [0, 1].
      :type command: Sequence[float], shape (n_legs,)

      :returns: * **t_steps** (*SwitchingTimes, shape (N,)*) -- Durations of the intervals (s).
                * **gates** (*GateStates, shape (N, 2, n_legs)*) -- Gate signals of the upper switches and blanking signals in each interval.

      .. rubric:: Notes

      Simultaneous switching instants are merged, so the number of intervals varies.
      Blanking continues across half carrier periods, including changes in their
      length. A reversed command cancels a pending turn-on and starts a new blanking
      interval. On the first call, the initial commanded state is assumed to
      already conduct.















      ..
          !! processed by numpydoc !!


.. py:class:: CompositeModel(models)

   Bases: :py:obj:`motulator.common.model._base.Model`\ [\ :py:obj:`None`\ ]


   
   Composite of models sharing subsystems.

   The models are coupled through shared subsystem objects, e.g., a DC bus given to
   the models of a grid converter and a drive. Each model keeps its converter, whose
   modulator includes the PWM model and the computational delay, and the model of a
   controlled converter is driven by its own control system in the simulation, see
   `Simulation`. The subsystems are
   evaluated in the order of the models, a shared subsystem at its first occurrence.
   The order is not resolved automatically: a shared subsystem whose outputs other
   subsystems depend on directly must come first in each model, as `Model` places the
   DC bus.

   :param models: Models by name. The names are used for the control systems and the results, so
                  they must be valid attribute names (not starting with an underscore, and not
                  `t`).
   :type models: dict[str, Model]















   ..
       !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Save the solution and the time histories of the models.
















      ..
          !! processed by numpydoc !!


.. py:class:: DCBus(C_dc, u_dc)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Capacitive DC bus feeding converters.

   The DC bus feeds the converters of the models it is given to, see `Model`. The
   models connect the DC-bus voltage to the input `u_dc` of their converters and
   register the converters in `converters`, whose DC currents the DC bus computes from
   their switching states and AC currents. The DC bus can be shared by the converters
   of several models, see `CompositeModel`. An external current, e.g., from a battery
   or a braking chopper, can also be fed to the DC bus.

   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float
   :param u_dc: Initial DC-bus voltage (V).
   :type u_dc: float















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_dc_current(t)

      
      Compute the net current (A) fed to the DC-bus capacitor.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_dc_current(i_dc)

      
      Set external DC current (A), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables for interconnection.
















      ..
          !! processed by numpydoc !!


.. py:class:: Drive(machine, mechanics, converter, lc_filter = None, dc_bus = None)

   Bases: :py:obj:`motulator.common.model._base.Model`\ [\ :py:obj:`motulator.common.model._converter.VoltageSourceConverter`\ ]


   
   Continuous-time system model for a machine drive.

   :param machine: Electric machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanical system model.
   :type mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
   :param converter: Converter model.
   :type converter: VoltageSourceConverter
   :param lc_filter: LC filter model. If not given, a direct connection between the converter and
                     machine is used.
   :type lc_filter: LCFilter, optional
   :param dc_bus: DC bus feeding the converter, which can be shared with other models (e.g., with
                  a grid converter system with a diode bridge or an active front end), see
                  `CompositeModel`. Defaults to None, in which case the converter has a constant
                  DC-bus voltage.
   :type dc_bus: DCBus, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ExternalRotorSpeed

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Integrate rotor angle from externally given rotor speed.

   This class maintains the same interface as other mechanical systems but the speed is
   determined by an external function rather than by torque dynamics.















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)
      :abstractmethod:


      
      Set external load torque (Nm), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(w_M)

      
      Set external rotor speed (rad/s), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list, index)

      
      Set states, keeping the rotor angle within [-pi, pi).
















      ..
          !! processed by numpydoc !!


.. py:class:: GridConnectedMachine(machine, mechanics, ac_source)

   Bases: :py:obj:`motulator.common.model._base.Model`\ [\ :py:obj:`None`\ ]


   
   Continuous-time system model for a machine connected directly to the grid.

   The stator terminals of the machine are connected to a three-phase voltage source,
   e.g., to model the direct-on-line start of an induction machine. A series grid
   impedance can be lumped into the machine parameters, e.g., into the stator
   resistance and the leakage inductance of the inverse-Γ model of an induction
   machine.

   :param machine: Electric machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanical system model.
   :type mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
   :param ac_source: Three-phase voltage source.
   :type ac_source: ThreePhaseSource















   ..
       !! processed by numpydoc !!

.. py:class:: InductionMachine(par, i_s_sensor = None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Γ-equivalent model of an induction machine.

   An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. The stator
   inductance `L_s` can either be constant or a function of the stator flux magnitude::

       L_s = L_s(abs(psi_s_ab))

   Optionally, eddy-current core losses can be modeled by means of a constant core-loss
   conductance `G_c`, connected in parallel with the magnetizing branch.

   :param par: Machine parameters. Core losses are modeled if `par.G_c` is nonzero.
   :type par: InductionMachinePars | InductionMachineInvGammaPars
   :param i_s_sensor: Phase-current sensors, defaults to an ideal measurement.
   :type i_s_sensor: Sensor, optional

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic saturation
   model in a straightforward manner. If the magnetic saturation is omitted, the Γ
   model is mathematically identical to the inverse-Γ and T models [#Sle1989]_.

   The core-loss branch is located between the stator resistance and the magnetizing
   inductance, i.e., the voltage across `G_c` is `u_s_ab - R_s*i_s_ab`. Consequently,
   the stator current depends directly on the stator voltage. This algebraic loop is
   solved in a closed form. The electromagnetic torque is produced by the current
   flowing into the magnetic circuit, i.e., the core-loss current is excluded from the
   torque.

   .. rubric:: References

   .. [#Sle1989] Slemon, "Modelling of induction machines for electric
      drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_ac_port()

      
      Compute the AC port for a converter with blocked gates, see `ACPort`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_outputs(state, inp)

      
      Compute output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachineInvGammaPars

   
   Constant inverse-Γ model parameters of an induction machine.

   This contains constant inverse-Γ model parameters of an induction machine. To model
   the main-flux saturation, use the `InductionMachinePars` class instead.

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
   :param G_c: Core-loss conductance (S), defaults to 0 (no core losses). It is connected
               across the stator EMF `u_s - R_s*i_s`, i.e., between the stator resistance and
               the leakage inductance, which corresponds to the core-loss branch in parallel
               with the magnetizing branch of the Γ model.
   :type G_c: float, optional

   .. attribute:: R_sgm

      Inverse-Γ total resistance `R_s` plus `R_R` (Ω).

      :type: float

   .. attribute:: alpha

      Inverse rotor time constant (rad/s).

      :type: float

   .. attribute:: w_rb

      Breakdown slip angular frequency (rad/s).

      :type: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_gamma_pars(par)
      :classmethod:


      
      Compute inverse-Γ model parameters from Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Γ-model parameters.
      :type par: InductionMachinePars

      :returns: Inverse-Γ model parameters.
      :rtype: InductionMachineInvGammaPars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachinePars

   
   Γ-model parameters of an induction machine.

   This contains Γ-model parameters of an induction machine. The main-flux saturation
   can also be modeled by providing a callable `L_s` parameter. For convenience, the
   class also provides the corresponding inverse-Γ model parameters, which can be used
   in control systems. If the saturation is modeled, these inverse-Γ parameters depend
   on the stator flux linkage magnitude `psi_s` that should be updated by calling the
   `update_psi_s` method.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H). If callable, it should be a function of the stator flux
               linkage magnitude (Vs).
   :type L_s: float | Callable[[float], float]
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional
   :param psi_s: Stator flux linkage magnitude (Vs), at which the callable `L_s` is evaluated,
                 defaults to 0. It can be updated by calling the `update_psi_s` method.
   :type psi_s: float, optional

   .. attribute:: gamma

      Magnetic coupling factor.

      :type: float

   .. attribute:: L_M

      Inverse-Γ magnetizing inductance (H).

      :type: float

   .. attribute:: L_sgm

      Inverse-Γ leakage inductance (H).

      :type: float

   .. attribute:: R_R

      Inverse-Γ rotor resistance (Ω).

      :type: float

   .. attribute:: R_sgm

      Inverse-Γ total resistance `R_s` plus `R_R` (Ω).

      :type: float

   .. attribute:: alpha

      Inverse rotor time constant (rad/s).

      :type: float

   .. attribute:: w_rb

      Breakdown slip angular frequency (rad/s).

      :type: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_inv_gamma_pars(par)
      :classmethod:


      
      Compute Γ-model parameters from inverse-Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Inverse-Γ model parameters.
      :type par: InductionMachineInvGammaPars

      :returns: Γ model parameters.
      :rtype: InductionMachinePars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: LCFilter(L_f, C_f, R_f = 0.0, i_c_sensor = None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   LC-filter model.

   :param L_f: Filter inductance (H).
   :type L_f: float
   :param C_f: Filter capacitance (F).
   :type C_f: float
   :param R_f: Series resistance (Ω) of the inductor, defaults to 0.
   :type R_f: float, optional
   :param i_c_sensor: Converter phase-current sensors, defaults to an ideal measurement.
   :type i_c_sensor: Sensor, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_ac_port()

      
      Compute the AC port for a converter with blocked gates, see `ACPort`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor line-to-line voltages u_ab and u_bc (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: MechanicalSystem(J, B_L = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Stiff mechanical system.

   :param J: Total moment of inertia (kgm²).
   :type J: float
   :param B_L: Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
               friction, or an arbitrary function of the rotor speed. For example, choosing
               ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic load torque ``k*w_M**2``.
               Defaults to 0.
   :type B_L: float | Callable[[float], float], optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_total_load_torque(t, w_M)

      
      Total load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)

      
      Set external load torque (Nm), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(_)
      :abstractmethod:


      
      Set external rotor speed (rad/s), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list, index)

      
      Set states, keeping the rotor angle within [-pi, pi).
















      ..
          !! processed by numpydoc !!


.. py:class:: SaturatedSynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Parameters of a saturated synchronous machine.

   The saturation model is specified as a current map (current as a function of the
   flux linkage). Optionally, to be used only in control systems, a flux map (flux
   linkage as a function of the current) can be provided. For convenience, this class
   also provides the incremental inductance matrix, which can be used in control and
   optimal reference generation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param i_s_dq_fcn: Stator current (A) as a function of the stator flux linkage (Vs). Needed in the
                      system model and in some control methods.
   :type i_s_dq_fcn: Callable[[complex], complex], optional
   :param psi_s_dq_fcn: Stator flux linkage (Vs) as a function of the stator current (A). This function
                        should be differentiable, if incremental inductances are used. Needed only for
                        control methods and optimal reference loci, not used in the system model.
   :type psi_s_dq_fcn: Callable[[complex], complex], optional
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix at given current.
















      ..
          !! processed by numpydoc !!


   .. py:method:: iterate_i_s_dq(psi_s_dq)

      
      Compute the current from the flux linkage.

      The current map is used if available. Otherwise, the current is computed
      iteratively from the flux map using a root-finding algorithm, which is less
      efficient but may be convenient in some special cases.















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage as a function of the stator current.
















      ..
          !! processed by numpydoc !!


.. py:class:: Sensor

   
   Sensor with a gain error, an offset, and additive Gaussian noise.

   The measured signal is ``y = gain*x + offset + std*n``, where `x` is the actual
   signal and `n` is zero-mean white Gaussian noise with unit variance, drawn
   independently for each channel at each sample. The parameters are scalars or arrays
   with one element per channel (e.g., per phase), in the units of the measured
   signal. The noise represents the sampled signal, i.e., `std` is the standard
   deviation of the samples after the anti-aliasing filter and the A/D conversion.

   :param gain: Gain, defaults to 1.
   :type gain: array_like, optional
   :param offset: Offset, defaults to 0.
   :type offset: array_like, optional
   :param std: Standard deviation of the noise, defaults to 0.
   :type std: array_like, optional
   :param rng: Random number generator, defaults to an unseeded generator. For reproducible
               results, pass the same seeded generator to all sensors. Sensors with separate
               generators of the same seed would produce identical noise.
   :type rng: numpy.random.Generator, optional

   .. rubric:: Examples

   >>> from motulator.common.model import Sensor
   >>> sensor = Sensor(gain=[1.02, 1, 1], offset=[0, 0.1, 0])
   >>> sensor([1.0, -0.5, -0.5])
   array([ 1.02, -0.4 , -0.5 ])















   ..
       !! processed by numpydoc !!

.. py:class:: Simulation(mdl, ctrl = None, show_progress = True, cfg = None, t_start = None)

   
   Simulation environment.

   Each control system is called at its own sampling instants, which gives the
   sampling period of its next control cycle. The control systems of a composite
   model can have different sampling periods, and their sampling instants need not
   coincide. The switching states of the converters are held constant between the
   sampling and switching instants of all the converters.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time control system of a model with a controlled converter. For a
                composite model, the control systems of the models with controlled converters
                by the model names. Defaults to None, i.e., no control systems.
   :type ctrl: ControlSystem | Mapping[str, ControlSystem], optional
   :param show_progress: Show progress during simulation, defaults to True.
   :type show_progress: bool, optional
   :param cfg: Solver configuration parameters.
   :type cfg: SolverCfg, optional
   :param t_start: First sampling instant (s) of the control system or, for a composite model,
                   of the control systems by the model names. The time `ctrl.t` of the control
                   system is set to this value. Defaults to the initial time of the model, as do
                   the control systems not included in the mapping, in which case `ctrl.t` is
                   not changed.
   :type t_start: float | Mapping[str, float], optional















   ..
       !! processed by numpydoc !!

   .. py:method:: simulate(t_stop = 1.0, N_eval = 0)

      
      Solve continuous-time system model and call control system.

      The simulation runs until the first sampling instant after `t_stop`, or
      until `t_stop` without control systems. A subsequent call continues the
      simulation.

      :param t_stop: Simulation stop time, defaults to 1.
      :type t_stop: float, optional
      :param N_eval: Number of evenly spaced data points to be returned by the solver for each
                     integration interval, i.e., for each interval between the sampling and
                     switching instants, or for the whole simulation without control systems.
                     Defaults to 0, in which case the number and spacing of points is selected
                     by the solver.
      :type N_eval: int, optional















      ..
          !! processed by numpydoc !!


.. py:class:: SimulationResults

   
   Container for simulation results.

   .. attribute:: mdl

      Results from the continuous-time model. For a composite model, the time series
      of each model is an attribute by the model name.

      :type: ModelTimeSeries | CompositeModelTimeSeries

   .. attribute:: ctrl

      Results from the digital control system, None for a model without a control
      system. For a composite model, the results of each control system are an
      attribute by the model name (None for a model without a control system).

      :type: Any

   .. attribute:: success

      False if the simulation stopped before the stop time due to a numerical
      failure, in which case the results cover the time until the failure.

      :type: bool















   ..
       !! processed by numpydoc !!

.. py:class:: SpatialSaturatedSynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Parameters of a saturated synchronous machine with spatial harmonics.

   This saturation model contains spatial harmonics in addition to the saturation
   effects. This version is intended to parametrize high-fidelity machine models for
   simulation purposes, while control methods do not support spatial harmonics.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param magnetic_map_fcn: Stator current (A) and electromagnetic torque (Nm) per pole pair as functions of
                            the stator flux linkage (Vs) and the complex exponential of the electrical rotor
                            angle.
   :type magnetic_map_fcn: Callable[[complex, complex], Tuple[complex, float]]
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m = None)

      
      Current (A) as a function of flux linkage (Vs) and rotor angle.
















      ..
          !! processed by numpydoc !!


   .. py:method:: magnetic_map(psi_s_dq, exp_j_theta_m = None)

      
      Magnetic map.

      :param psi_s_dq: Stator flux linkage (Vs) in rotor coordinates.
      :type psi_s_dq: complex | ndarray
      :param exp_j_theta_m: Complex exponential of electrical rotor angle.
      :type exp_j_theta_m: complex | ndarray

      :returns: Stator current (A) in rotor coordinates and electromagnetic torque (Nm) per
                pole pair.
      :rtype: Tuple [complex | ndarray, float | ndarray]















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachine(par, i_s_sensor = None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Synchronous machine model.

   This model is internally represented in rotor coordinates, which results in the
   simplest implementation. The interfaces are in stator coordinates. The magnetic
   saturation can be modeled by providing a nonlinear current map, either as
   `i_s_dq_fcn` in `SaturatedSynchronousMachinePars` or, including spatial harmonics,
   as `magnetic_map_fcn` in `SpatialSaturatedSynchronousMachinePars`.
   Optionally, eddy-current core losses can be modeled by means of a constant core-loss
   conductance `G_c`, connected in parallel with the magnetizing branch.

   :param par: Machine parameters. Core losses are modeled if `par.G_c` is nonzero.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars         | SpatialSaturatedSynchronousMachinePars
   :param i_s_sensor: Phase-current sensors, defaults to an ideal measurement.
   :type i_s_sensor: Sensor, optional

   .. rubric:: Notes

   The core-loss branch is located between the stator resistance and the magnetizing
   branch, i.e., the voltage across `G_c` is `u_s - R_s*i_s`. Consequently, the stator
   current depends directly on the stator voltage. This algebraic loop is solved in a
   closed form. The electromagnetic torque is produced by the current flowing into the
   magnetic circuit, i.e., the core-loss current is excluded from the torque.















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_ac_port()

      
      Compute the AC port for a converter with blocked gates, see `ACPort`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_outputs(state, inp)

      
      Compute output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list, index)

      
      Set states, keeping the rotor angle within [-pi, pi).
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Synchronous machine parameters, without saturation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: Permanent-magnet flux linkage (Vs).
   :type psi_f: float
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix (H).
















      ..
          !! processed by numpydoc !!


   .. py:method:: iterate_i_s_dq(psi_s_dq)

      
      Compute the current from the flux linkage in closed form.
















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage (Vs) as a function of the stator current (A).
















      ..
          !! processed by numpydoc !!


.. py:class:: ThreePhaseSource(w_g = 2 * pi * 50, e_g = sqrt(2 / 3) * 400, phi = 0.0, e_g_neg = 0.0, phi_neg = 0.0)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Three-phase source model.

   The frequency, phase shift, and magnitude can be given either as constants or
   functions of time. An unbalanced source can be modeled by specifying a negative-
   sequence component. The zero-sequence component is not included in this model.

   :param w_g: Angular frequency (rad/s), defaults to 2*pi*50.
   :type w_g: float | Callable[[float], float], optional
   :param e_g: Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
   :type e_g: float | Callable[[float], float], optional
   :param phi: Phase shift (rad) of positive-sequence component, defaults to 0.
   :type phi: float | Callable[[float], float], optional
   :param e_g_neg: Peak-valued magnitude of negative-sequence component, defaults to 0.
   :type e_g_neg: float | Callable[[float], float], optional
   :param phi_neg: Phase shift (rad) of negative-sequence component, defaults to 0.
   :type phi_neg: float | Callable[[float], float], optional

   .. rubric:: Notes

   This model is typically used to represent a voltage source, but it can be configured
   to represent, e.g., a current source as well.















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: generate_space_vector(t, exp_j_theta_g)

      
      Generate the space vector in stationary coordinates.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivative.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list, index)

      
      Set the state, keeping the angle within [-pi, pi).
















      ..
          !! processed by numpydoc !!


.. py:class:: TwoMassMechanicalSystem(J_M, J_L, K_S, C_S, B_L = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Two-mass mechanical subsystem.

   :param J_M: Motor moment of inertia (kgm²).
   :type J_M: float
   :param J_L: Load moment of inertia (kgm²).
   :type J_L: float
   :param K_S: Shaft torsional stiffness (Nm/rad).
   :type K_S: float
   :param C_S: Shaft torsional damping (Nm/(rad/s)).
   :type C_S: float
   :param B_L: Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
               friction, or an arbitrary function of the load speed. For example, choosing
               ``B_L = lambda w_L: k*abs(w_L)`` leads to quadratic load torque ``k*w_L**2``.
               Defaults to 0.
   :type B_L: float | Callable[[float], float], optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_shaft_torque(theta_ML, w_M, w_L)

      
      Shaft torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_total_load_torque(t, w_L)

      
      Total load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















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


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)

      
      Set external load torque (Nm), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(_)
      :abstractmethod:


      
      Set external rotor speed (rad/s), constant or a function of time.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list, index)

      
      Set states, keeping the rotor angle within [-pi, pi).
















      ..
          !! processed by numpydoc !!


.. py:class:: VoltageSourceConverter(u_dc = None, modulator = None, u_th = 0.0, sign = np.sign, u_dc_sensor = None)

   Bases: :py:obj:`DiodeBridge`


   
   Three-phase voltage-source converter.

   The modulator converts the duty ratios of the control system into the gate states
   of the legs, i.e., the gate signals of the upper switches and the blanking signals,
   including the computational delay and the dead time, see `ZOH` and
   `CarrierComparison`. The switches are ideal, except for the dead time and the
   threshold voltage. When a leg is blanked, i.e., neither of its switches conducts,
   the current direction determines which diode conducts. The leg state is then
   `(1 - sign(i))/2`, where `i` is the phase current in the beginning of the blanking
   interval. The threshold voltage of the conducting device, assumed equal for the
   switches and the diodes, reduces each phase voltage by `u_th*sign(i)`, where the
   current direction is also determined at the latest switching instant. A current
   within the tolerance of the diode-state transitions has no direction, see
   `current_direction`. The DC-bus voltage is the input `u_dc`, which is constant
   unless a DC-bus model is connected to it (see `DCBus` and `Model`).

   When the gates are blocked (`inp.enabled` is False), all the switches are open, and
   the converter operates as a diode bridge, see `DiodeBridge`. The diode states are
   latched from the current directions when the gates are blocked.

   :param u_dc: Constant DC-bus voltage (V). Defaults to None, in which case the converter must
                be fed from a DC-bus model.
   :type u_dc: float, optional
   :param modulator: Modulator converting the command of the control system into the gate states,
                     see `TwoLevelModulator`. Defaults to `ZOH()`, i.e., the switching-cycle
                     averaged model of the duty ratios without the dead time and with the
                     computational delay of one sampling period. A modulator keeps its state, so
                     each converter needs its own.
   :type modulator: TwoLevelModulator, optional
   :param u_th: Threshold voltage (V) of the power devices, defaults to 0.
   :type u_th: float, optional
   :param sign: Function of the phase currents (A) determining the leg states during blanking,
                defaults to `np.sign`. A smooth function, such as `2/pi*arctan(i/i_d)`, can be
                used to model the effect of parasitic capacitances, for example.
   :type sign: Callable[[np.ndarray], np.ndarray], optional
   :param u_dc_sensor: DC-bus voltage sensor, defaults to an ideal measurement.
   :type u_dc_sensor: Sensor, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: blocked
      :type: bool

      
      Only the diodes conduct while the gates are blocked.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_internal_dc_current(inp)

      
      Compute the internal DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: extend_history(n, u_c_ab)

      
      Extend the history, the switching state held constant, see `DiodeBridge`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: init_state_history()

      
      Initialize the history of the switching state and the voltage.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure converter DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_voltages()

      
      Measure the converter line-to-line voltages u_ab and u_bc (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: modulate(T_s, command)

      
      Convert the command to the gate states and their durations.

      The command passes through the modulator also while the gates are blocked,
      but the gate states are then zero for the whole sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param command: Command computed at this sampling instant, the duty ratios for `ZOH` and
                      `CarrierComparison`.
      :type command: Any

      :returns: * **t_steps** (*SwitchingTimes, shape (N,)*) -- Durations of the intervals (s), summing to `T_s`.
                * **gates** (*GateStates, shape (N, 2, 3)*) -- Gate signals of the upper switches and blanking signals in each interval.















      ..
          !! processed by numpydoc !!


   .. py:method:: set_enable(enabled)

      
      Set the gate enable.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_gates(gates)

      
      Set the switching state based on the gate state of an interval.

      :param gates: Gate signals of the upper switches and blanking signals.
      :type gates: ndarray, shape (2, 3)















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: ZOH(t_d = 0.0, delay = None, single_update = False)

   Bases: :py:obj:`TwoLevelModulator`\ [\ :py:obj:`collections.abc.Sequence`\ [\ :py:obj:`float`\ ]\ ]


   
   Zero-order hold of the duty ratios of a voltage-source converter.

   The dead time reduces the gate signal duty ratios of both switches of a leg by
   `t_d/(2*T_half)`, leaving the leg blanked for the fraction `t_d/T_half` of the
   half carrier period `T_half`, unless the leg does not switch. Short pulses are
   suppressed, and the blanking fraction is computed from the remaining on-times of
   both switches. This is a switching-cycle averaged model, not a model of switching
   transients.

   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
   :param delay: Computational delay in half carrier periods, defaults to one sampling period,
                 see `Modulator`.
   :type delay: int, optional
   :param single_update: Sample and update the duty ratios once per carrier period, defaults to False,
                         see `Modulator`.
   :type single_update: bool, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: schedule(T_half, command)

      
      Compute the averaged gate states.

      :param T_half: Half carrier period (s).
      :type T_half: float
      :param command: Duty ratios in the range [0, 1].
      :type command: Sequence[float], shape (n_legs,)

      :returns: * **t_steps** (*SwitchingTimes, shape (1,)*) -- Half carrier period (s).
                * **gates** (*GateStates, shape (1, 2, n_legs)*) -- On-time fractions of the upper switches and blanking fractions.















      ..
          !! processed by numpydoc !!


