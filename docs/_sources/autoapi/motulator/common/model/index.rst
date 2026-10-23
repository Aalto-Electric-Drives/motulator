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
   motulator.common.model.DCBus
   motulator.common.model.DiodeBridgeDCBus
   motulator.common.model.Model
   motulator.common.model.ModelTimeSeries
   motulator.common.model.Sensor
   motulator.common.model.Simulation
   motulator.common.model.SimulationResults
   motulator.common.model.SolverCfg
   motulator.common.model.Subsystem
   motulator.common.model.SubsystemTimeSeries
   motulator.common.model.VoltageSourceConverter


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

.. py:class:: CarrierComparison(N = 2**12, t_d = 0.0)

   Bases: :py:obj:`PWM`


   
   Carrier comparison.

   This computes the gate signals and their durations based on the duty ratios. Instead
   of searching for zero crossings, the switching instants are explicitly computed in
   the beginning of each sampling period, allowing faster simulations. The dead time
   delays the turn-on of the switches, i.e., each leg is blanked for `t_d` after its
   switching instant.

   :param N: Amount of the counter quantization levels, defaults to 2**12.
   :type N: int, optional
   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional

   .. rubric:: Examples

   >>> from motulator.common.model import CarrierComparison
   >>> carrier_cmp = CarrierComparison()
   >>> # First call gives rising edges
   >>> t_steps, q_abc, _ = carrier_cmp(1e-3, [.4, .2, .8])
   >>> # Durations of the switching states
   >>> t_steps
   array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
   >>> # Switching states
   >>> q_abc
   array([[0, 0, 0],
          [0, 0, 1],
          [1, 0, 1],
          [1, 1, 1]])
   >>> # Second call gives falling edges
   >>> t_steps, q_abc, _ = carrier_cmp(1e-3, [.4, .2, .8])
   >>> t_steps
   array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
   >>> q_abc
   array([[1, 1, 1],
          [1, 0, 1],
          [0, 0, 1],
          [0, 0, 0]])
   >>> # Sum of the step times equals T_s
   >>> float(np.sum(t_steps))
   0.001
   >>> # Dead time blanks each leg after its switching instant
   >>> carrier_cmp = CarrierComparison(t_d=1e-4)
   >>> t_steps, q_abc, b_abc = carrier_cmp(1e-3, [.5, .5, .5])
   >>> np.round(t_steps / 1e-3, 3)  # In ms
   array([0.5, 0.1, 0.4])
   >>> q_abc
   array([[0, 0, 0],
          [0, 0, 0],
          [1, 1, 1]])
   >>> b_abc
   array([[0, 0, 0],
          [1, 1, 1],
          [0, 0, 0]])















   ..
       !! processed by numpydoc !!

.. py:class:: CompositeModel(models)

   Bases: :py:obj:`motulator.common.model._base.System`


   
   Composite of models sharing subsystems.

   The models are coupled through shared subsystem objects, e.g., a DC bus given to
   the models of a grid converter and a drive. Each model keeps its converter, PWM
   model, and computational delay, and it is driven by its own control system in the
   simulation, see `Simulation`. The subsystems are evaluated in the order of the
   models, a shared subsystem at its first occurrence. The order is not resolved
   automatically: a shared subsystem whose outputs other subsystems depend on
   directly must come first in each model, as `Model` places the DC bus.

   :param models: Models by name, each with its own converter. The names are used for the
                  control systems and the results, so they must be valid attribute names (not
                  starting with an underscore, and not `t`).
   :type models: dict[str, Model]















   ..
       !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Save the solution and the switching states of the models.
















      ..
          !! processed by numpydoc !!


.. py:class:: CompositeModelTimeSeries

   
   Time series of the simulation results of a composite model.

   The time series of each model is stored as an attribute by the model name (e.g.,
   `drive`), see `ModelTimeSeries`.















   ..
       !! processed by numpydoc !!

.. py:class:: DCBus(C_dc, u_dc, converters)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Capacitive DC bus feeding converters.

   The DC bus feeds the given converters, whose DC currents are computed from their
   switching states and AC currents. The DC-bus voltage is connected to the input
   `u_dc` of each converter by the model (see `Model`). The DC bus can be shared by
   the converters of several models (see `CompositeModel`). An external current,
   e.g., from a battery or a braking chopper, can also be fed to the DC bus.

   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float
   :param u_dc: Initial DC-bus voltage (V).
   :type u_dc: float
   :param converters: Converters fed from the DC bus.
   :type converters: Sequence[VoltageSourceConverter]















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


.. py:class:: DiodeBridgeDCBus(C_dc, L_dc, U_g, f_g, converters)

   Bases: :py:obj:`DCBus`


   
   DC bus fed from a six-pulse diode bridge.

   A three-phase diode bridge rectifier with a DC-bus inductor is modeled, supplied
   from a stiff grid, i.e., the grid inductance is zero. The DC bus feeds the given
   converters, see `DCBus`.

   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float
   :param L_dc: DC-bus inductance (H).
   :type L_dc: float
   :param U_g: Grid voltage (V, line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency (Hz).
   :type f_g: float
   :param converters: Converters fed from the DC bus.
   :type converters: Sequence[VoltageSourceConverter]















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_derivatives(t, state)

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_voltages(state)

      
      Compute grid and rectified voltages.
















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


.. py:class:: Model(converter, subsystems, connections, dc_bus = None, pwm = False, delay = 0, single_update = False)

   Bases: :py:obj:`System`


   
   Base class for system models with a converter driven by a control system.

   The converter subsystem gets the switching state `q_c_ab` and the current-direction
   vector `s_c_ab`, which are held constant over each integration interval. The PWM
   model includes the dead time `converter.t_d`, and the computational delay is sized
   according to the number of legs `converter.n_legs`. The control systems of this
   package compensate for a computational delay of one sampling period, i.e.,
   `delay=1` with double update and `delay=2` with single update. The control system
   can also block the gates of the converter, see `ControlSystem.set_enable`, in
   which case the converter voltage is determined by the AC port of its load.

   :param converter: Converter model.
   :type converter: Converter
   :param subsystems: All subsystems, including the converter.
   :type subsystems: list[Subsystem]
   :param connections: Connections as `{(target, input_name): (source, output_name)}`.
   :type connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]]
   :param dc_bus: DC bus feeding the converter, whose input `u_dc` is connected to the DC-bus
                  voltage. The DC bus is added to the subsystems, and it can be shared with other
                  models, see `CompositeModel`. Defaults to None, in which case the converter has
                  its own DC-bus model (or a constant DC-bus voltage).
   :type dc_bus: DCBus, optional
   :param pwm: Enable PWM model, defaults to False.
   :type pwm: bool, optional
   :param delay: Computational delay in half carrier periods from the sampling instant to the
                 loading of the duty ratios, defaults to 0. With double update, this equals
                 the delay in sampling periods.
   :type delay: int, optional
   :param single_update: Sample and update the duty ratios once per carrier period, i.e., the sampling
                         period of the control system equals the carrier period, defaults to False
                         (double update, the sampling period is half the carrier period). With single
                         update, `delay=2` loads the duty ratios at the next sampling instant and
                         `delay=1` at the carrier extremum between the sampling instants.
   :type single_update: bool, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: modulate(T_s, d_abc)

      
      Delay the duty ratios and convert them to gate signals.

      The sampling period consists of one half carrier period with double update
      and of two with single update. The duty ratios pass through the
      computational delay once per half carrier period, and the PWM model is
      called for each half carrier period. If the gates of the converter are
      blocked (`converter.inp.enabled` is False), the duty ratios still pass
      through the delay and the PWM model, but the gate signals are zero for the
      whole sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param d_abc: Duty ratios computed at this sampling instant.
      :type d_abc: Sequence[float]

      :returns: * **t_steps** (*SwitchingTimes, shape (N,)*) -- Durations of the intervals (s), summing to `T_s`.
                * **q_abc** (*GateSignals, shape (N, n_legs)*) -- Gate signals of the upper switches in each interval.
                * **b_abc** (*BlankingSignals, shape (N, n_legs)*) -- Blanking signals in each interval.















      ..
          !! processed by numpydoc !!


   .. py:method:: save(sol)

      
      Save the solution and the switching state.
















      ..
          !! processed by numpydoc !!


.. py:class:: ModelTimeSeries

   
   Time series of the simulation results.

   The time series of each subsystem is stored as an attribute (e.g., `machine`). It
   also contains the time series of the subsystem inputs, which are used for plotting
   and for computing the signals that depend directly on the inputs.















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

.. py:class:: Simulation(mdl, ctrl, show_progress = True, cfg = None, t_start = None)

   
   Simulation environment.

   Each control system is called at its own sampling instants, which gives the
   sampling period of its next control cycle. The control systems of a composite
   model can have different sampling periods, and their sampling instants need not
   coincide. The switching states of the converters are held constant between the
   sampling and switching instants of all the converters.

   :param mdl: Continuous-time system model.
   :type mdl: Model | CompositeModel
   :param ctrl: Discrete-time control system. For a composite model, the control systems of
                the models by the model names.
   :type ctrl: ControlSystem | Mapping[str, ControlSystem]
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

      The simulation runs until the first sampling instant after `t_stop`. A
      subsequent call continues the simulation.

      :param t_stop: Simulation stop time, defaults to 1.
      :type t_stop: float, optional
      :param N_eval: Number of evenly spaced data points to be returned by the solver for each
                     integration interval, i.e., for each interval between the sampling and
                     switching instants. Defaults to 0, in which case the number and spacing of
                     points is selected by the solver.
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

      Results from the digital control system. For a composite model, the results of
      each control system is an attribute by the model name.

      :type: Any

   .. attribute:: success

      False if the simulation stopped before the stop time due to a numerical
      failure, in which case the results cover the time until the failure.

      :type: bool















   ..
       !! processed by numpydoc !!

.. py:class:: SolverCfg

   
   Solver configuration parameters.

   :param max_step: Maximum step size for the integrator, defaults to `inf`.
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
   be used with different input, output, state, and history types. The class provides
   methods for setting states, outputs, and computing state derivatives. It also
   provides methods for extending the state history and creating time-series
   representations of the subsystem. The state history is a dict of lists, keyed by
   the names of the state fields. The model initializes it by `init_state_history`
   and extends it by `extend_state_history`.















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_ac_port()

      
      Compute the AC port for a converter with blocked gates, see `ACPort`.
















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


.. py:class:: VoltageSourceConverter(u_dc = None, t_d = 0.0, u_th = 0.0, sign = np.sign, u_dc_sensor = None)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Three-phase voltage-source converter.

   The switches are ideal, except for the dead time and the threshold voltage. When a
   leg is blanked, i.e., neither of its switches conducts, the current direction
   determines which diode conducts. The leg state is then `(1 - sign(i))/2`, where
   `i` is the phase current in the beginning of the blanking interval. The threshold
   voltage of the conducting device, assumed equal for the switches and the diodes,
   reduces each phase voltage by `u_th*sign(i)`, where the current direction is also
   determined at the latest switching instant. The DC-bus voltage is either constant
   or given by a DC-bus model connected to the input `u_dc` (see `DCBus` and
   `Model`).

   When the gates are blocked (`inp.enabled` is False), all the switches are open, and
   the antiparallel diodes conduct according to the currents: a leg carrying current
   is clamped to a rail, and a leg without current floats at the voltage keeping its
   current at zero. The diode states are updated at their transitions, which the
   simulation locates as events, see `Simulation`. The voltage is computed from the
   AC port of the load `ac_load`, see `ACPort`, and the threshold voltage is included
   in the diode drops.

   :param u_dc: Constant DC-bus voltage (V). Defaults to None, in which case the converter must
                be fed from a DC-bus model.
   :type u_dc: float, optional
   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
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

   .. py:method:: compute_internal_dc_current(inp)

      
      Compute the internal DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series.
















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


   .. py:method:: rhs(t)

      
      Default empty implementation.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_enable(enabled)

      
      Set the gate enable, latching the diode states when the gates are blocked.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_gate_signals(q_abc, b_abc)

      
      Set the switching state based on the gate and blanking signals.
















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


