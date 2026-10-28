motulator.common.model
======================

.. py:module:: motulator.common.model

.. autoapi-nested-parse::

   
   Model package.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.common.model.ACPort
   motulator.common.model.CarrierComparison
   motulator.common.model.CompositeModel
   motulator.common.model.CompositeModelTimeSeries
   motulator.common.model.ControlledConverter
   motulator.common.model.Converter
   motulator.common.model.DCBus
   motulator.common.model.DiodeBridge
   motulator.common.model.Model
   motulator.common.model.ModelTimeSeries
   motulator.common.model.Modulator
   motulator.common.model.Sensor
   motulator.common.model.Simulation
   motulator.common.model.SimulationResults
   motulator.common.model.SolverCfg
   motulator.common.model.Subsystem
   motulator.common.model.SubsystemTimeSeries
   motulator.common.model.ThreePhaseSource
   motulator.common.model.TwoLevelModulator
   motulator.common.model.VoltageSourceConverter
   motulator.common.model.ZOH


Package Contents
----------------

.. py:class:: ACPort

   
   Relation between the terminal current and voltage of a load.

   The current derivative of the load at the converter terminals is ``di/dt = Y @ (u
   - e)`` in stationary coordinates, where `u` is the converter voltage. The matrix
   `Y` acts on the real and imaginary parts of the space vectors. The converter uses
   the port to compute its voltage when its gates are blocked.

   :param i: Current (A).
   :type i: complex
   :param e: Voltage (V) at which the current derivative is zero.
   :type e: complex
   :param Y: Inverse differential inductance matrix (1/H).
   :type Y: ndarray, shape (2, 2)















   ..
       !! processed by numpydoc !!

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


.. py:class:: CompositeModelTimeSeries

   
   Time series of the simulation results of a composite model.

   The time series of each model is stored as an attribute by the model name (e.g.,
   `drive`), see `ModelTimeSeries`.















   ..
       !! processed by numpydoc !!

.. py:class:: ControlledConverter

   Bases: :py:obj:`Protocol`


   
   Protocol defining the gate control of converters.

   A controlled converter is a converter (see `Converter`) whose switches are driven by
   a control system. Its modulator converts the command of the control system (e.g.,
   the duty ratios) into the gate states and their durations within the sampling
   period, see `Modulator`. The simulation sets the gate states at the switching
   instants, from which the converter resolves its switching state (e.g., from the
   gate signals and the phase currents during the blanking). The format of the
   command and the gate states is up to the converter and its modulator. The control
   system can block the gates through `set_enable` at its sampling instants, after
   which only the diodes conduct (see `Converter.blocked`). By default, blocking the
   gates is not supported.















   ..
       !! processed by numpydoc !!

   .. py:method:: modulate(T_s, command)

      
      Convert the command to the gate states and their durations.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_enable(enabled)

      
      Set the gate enable, blocking the gates not supported by default.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_gates(gates)

      
      Set the switching state based on the gate states.
















      ..
          !! processed by numpydoc !!


.. py:class:: Converter

   Bases: :py:obj:`Subsystem`, :py:obj:`Protocol`


   
   Protocol defining the interface for converters.

   In addition to the subsystem interface, a converter provides its internal DC
   current, which a DC bus draws (see `DCBus`), and the states of its diodes. While
   only the diodes conduct (`blocked` is True), the converter voltage `out.u_c_ab` is
   determined by the AC port of the load `ac_load` and the states of the diodes, see
   `ACPort`. The simulation then locates the transitions of the diode states as
   events, using the margin of the states, and updates the states at the transitions.
   Since the voltage varies within the integration interval, the model evaluates it
   at the solution points and passes it to `extend_history`, which also saves the
   other signals of the converter that are not states.

   The diodes and the DC bus are optional capabilities: by default, the gates are
   never blocked, so there are no diode-state transitions, and the converter cannot
   be fed from a DC bus. A converter subclassing this protocol implements only the
   capabilities it has.















   ..
       !! processed by numpydoc !!

   .. py:property:: blocked
      :type: bool

      
      Only the diodes conduct.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_internal_dc_current(inp)

      
      Compute the internal DC current (A) from the inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: extend_history(n, u_c_ab)

      
      Extend the history over `n` solution points, `u_c_ab` if blocked.
















      ..
          !! processed by numpydoc !!


   .. py:property:: margin
      :type: float

      
      Margin of the diode states, zero at their transition.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update_diodes()

      
      Update the diode states at a transition, nothing by default.
















      ..
          !! processed by numpydoc !!


.. py:class:: DCBus(C_dc, u_dc)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Capacitive DC bus feeding converters.

   The DC bus feeds the converters of the models it is given to, see `Model`. The
   model connects its converter to the DC bus through `connect`, which registers the
   converter in `converters` and gives the connections of its DC terminals, i.e., the
   DC-bus voltage to the input `u_dc` of the converter. The DC bus draws the DC
   currents of the registered converters, computed from their switching states and
   AC currents. The DC bus can be shared by the converters of several models, see
   `CompositeModel`. An external current, e.g., from a battery or a braking chopper,
   can also be fed to the DC bus.

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


   .. py:method:: connect(converter)

      
      Register a converter and get the connections of its DC terminals.

      :param converter: Converter fed from the DC bus.
      :type converter: Converter

      :returns: Connections from the DC bus to the converter, see `Model`.
      :rtype: dict[tuple[Subsystem, str], tuple[Subsystem, str]]















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


.. py:class:: DiodeBridge(u_dc = None, u_th = 0.0)

   Bases: :py:obj:`motulator.common.model._base.Converter`


   
   Three-phase six-pulse diode bridge.

   The diodes conduct according to the currents of the load `ac_load`: a leg carrying
   current is clamped to a rail, and a leg without current floats at the voltage
   keeping its current at zero. The voltage is computed from the AC port of the load,
   see `ACPort`. The diode states are updated at their transitions, which the
   simulation locates as events, see `Simulation`. Hence, the commutation of the
   current between the legs follows from the inductance of the load, e.g., an AC
   filter and the grid. The threshold voltage of the diodes is included in the diode
   drops. The DC-bus voltage is the input `u_dc`, which is constant unless a DC-bus
   model is connected to it (see `DCBus` and `Model`).

   :param u_dc: Constant DC-bus voltage (V). Defaults to None, in which case the bridge must be
                fed from a DC-bus model.
   :type u_dc: float, optional
   :param u_th: Threshold voltage (V) of the diodes, defaults to 0.
   :type u_th: float, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: blocked
      :type: bool

      
      Only the diodes conduct, as the bridge has no switches.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_internal_dc_current(inp)

      
      Compute the internal DC current (A), rectified by the diodes.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: extend_history(n, u_c_ab)

      
      Extend the history over the solution points of an integration interval.

      :param n: Number of solution points.
      :type n: int
      :param u_c_ab: Voltages (V) evaluated at the solution points if only the diodes conduct,
                     otherwise None.
      :type u_c_ab: Sequence[complex] | None















      ..
          !! processed by numpydoc !!


   .. py:method:: init_state_history()

      
      Initialize the history of the voltage, which is evaluated by the model.
















      ..
          !! processed by numpydoc !!


   .. py:property:: margin
      :type: float

      
      Margin of the diode states at the latest evaluation, see `diode_margin`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Default empty implementation.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update_diodes()

      
      Update the diode states at a transition, see `update_diodes`.
















      ..
          !! processed by numpydoc !!


.. py:class:: Model(converter, subsystems, connections, dc_bus = None)

   
   Base class for continuous-time system models.

   A model consists of subsystems and connections between them. The outputs are
   computed in the order of the `subsystems` list and passed to the connected inputs
   immediately. Hence, a subsystem whose outputs depend directly on its inputs must
   come after the subsystems providing these inputs. The voltage of a blocked
   converter depends on its load, so it is computed after the other subsystems.

   A model can have a converter, either a passive one (e.g., `DiodeBridge`) or a
   controlled one (see `ControlledConverter`), which is driven by a control system, see
   `Simulation`. The modulator of the controlled converter includes the computational
   delay and the PWM model, see `Modulator`. The control system can also block the
   gates of the converter, see `ControlSystem.set_enable`.

   :param converter: Converter, or None if the model has no converter.
   :type converter: Converter | None
   :param subsystems: All subsystems, including the converter.
   :type subsystems: list[Subsystem]
   :param connections: Connections as `{(target, input_name): (source, output_name)}`.
   :type connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]]
   :param dc_bus: DC bus feeding the converter. The DC bus is added to the subsystems, and the
                  converter is connected to it, see `DCBus.connect`. The DC bus can be shared
                  with other models, see `CompositeModel`. Defaults to None, in which case the
                  converter has a constant DC-bus voltage.
   :type dc_bus: DCBus, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: get_initial_values()

      
      Get initial values of all subsystems before the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t, state_list)

      
      Compute complete state derivative list for the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: save(sol)

      
      Save the solution and the signals of the converters.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Compute the outputs and pass them to the connected inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list)

      
      Set states in all subsystems.
















      ..
          !! processed by numpydoc !!


.. py:class:: ModelTimeSeries

   
   Time series of the simulation results.

   The time series of each subsystem is stored as an attribute (e.g., `machine`). It
   also contains the time series of the subsystem inputs, which are used for plotting
   and for computing the signals that depend directly on the inputs.















   ..
       !! processed by numpydoc !!

.. py:class:: Modulator(delay = None, single_update = False)

   
   Base class for modulators.

   The sampling period consists of one half carrier period with double update and of
   two with single update. The command passes through the computational delay once
   per half carrier period, and the gate states of each half carrier period are
   computed by `schedule`, which the subclasses implement. The type parameter `Cmd`
   is the type of the command, e.g., the duty ratios of a voltage-source converter.

   :param delay: Computational delay in half carrier periods from the sampling instant to the
                 loading of the command. Defaults to None, i.e., one sampling period, which the
                 control systems of this package compensate for: 1 with double update and 2
                 with single update. With single update, `delay=1` loads the command at the
                 carrier extremum between the sampling instants.
   :type delay: int, optional
   :param single_update: Sample and update the command once per carrier period, i.e., the sampling
                         period of the control system equals the carrier period, defaults to False
                         (double update, the sampling period is half the carrier period).
   :type single_update: bool, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: initial_command(command)

      
      Command assumed before the first command is loaded.

      The computational delay starts with this command, zeros of the shape of the
      command by default. A subclass whose command is not an array overrides this
      method.

      :param command: First command given to the modulator.
      :type command: Cmd

      :returns: Initial command.
      :rtype: Any















      ..
          !! processed by numpydoc !!


   .. py:method:: schedule(T_half, command)
      :abstractmethod:


      
      Convert the command to the gate states of a half carrier period.

      :param T_half: Half carrier period (s).
      :type T_half: float
      :param command: Command, delayed by the computational delay.
      :type command: Cmd

      :returns: * **t_steps** (*SwitchingTimes, shape (N,)*) -- Durations of the intervals (s), summing to `T_half`.
                * **gates** (*GateStates, shape (N, ...)*) -- Gate states in each interval.















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

.. py:class:: SolverCfg

   
   Solver configuration parameters.

   :param max_step: Maximum step size (s) for the integrator. Defaults to None, i.e., 1e-4 s while
                    the gates of a converter are blocked, so that the solver locates the
                    transitions of the diodes driven by the AC voltages, and without control
                    systems, so that the solver resolves the waveforms. Otherwise, the step size is
                    not limited, since the sampling and switching instants limit the integration
                    intervals.
   :type max_step: float, optional
   :param method: Integration method, defaults to "RK45".
   :type method: str, optional
   :param rtol: Relative tolerance, defaults to 1e-3.
   :type rtol: float, optional
   :param atol: Absolute tolerance, defaults to 1e-6.
   :type atol: float, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: solver
      :type: dict[str, Any]

      
      Return the solver configuration.
















      ..
          !! processed by numpydoc !!


.. py:class:: Subsystem

   Bases: :py:obj:`Protocol`


   
   Protocol defining the interface for all subsystems.

   This class defines the interface for all subsystems. It is a generic class that can
   be used with different input, output, and state types. The class provides methods
   for setting states, outputs, and computing state derivatives. It also provides
   methods for extending the state history and creating time-series representations
   of the subsystem. The state history is a dict of lists, keyed by the names of the
   state fields. The model initializes it by `init_state_history` and extends it by
   `extend_state_history`. A subclass can keep its other signals in the history as
   well, under other keys, as a converter does (see `Converter`).















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_ac_port()

      
      Compute the AC port for a blocked converter, see `ACPort`.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time-series representation of this subsystem.
















      ..
          !! processed by numpydoc !!


   .. py:method:: extend_state_history(sol_y, index)

      
      Extend the state history with values from solver output.
















      ..
          !! processed by numpydoc !!


   .. py:method:: init_state_history()

      
      Initialize the state history with an empty list for each state field.
















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

      
      Set states from the state list.
















      ..
          !! processed by numpydoc !!


.. py:class:: SubsystemTimeSeries

   Bases: :py:obj:`Protocol`


   
   Base class for subsystem time series.

   The outputs are computed when the time series is created or, if they depend
   directly on the inputs, in `set_outputs`. The signals computed in
   `compute_derived_signals` are not passed to other subsystems.















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_derived_signals(t, subsystem)

      
      Compute the other signals that depend on the inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t, subsystem)

      
      Compute the outputs that depend directly on the inputs.
















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


.. py:class:: TwoLevelModulator(delay = None, single_update = False)

   Bases: :py:obj:`Modulator`\ [\ :py:obj:`Cmd`\ ]


   
   Base class for the modulators of two-level voltage-source converters.

   The gate state of a leg consists of the gate signal of its upper switch and the
   blanking signal, see `VoltageSourceConverter`. Hence, the gate states of the N
   intervals form an array of shape (N, 2, n_legs), the gate signals in the first
   row and the blanking signals in the second row of each interval. In the averaged
   model, they are the on-time fractions of the upper switches and the blanking
   fractions.















   ..
       !! processed by numpydoc !!

.. py:class:: VoltageSourceConverter(u_dc = None, modulator = None, u_th = 0.0, sign = np.sign, u_dc_sensor = None)

   Bases: :py:obj:`DiodeBridge`, :py:obj:`motulator.common.model._base.ControlledConverter`


   
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


