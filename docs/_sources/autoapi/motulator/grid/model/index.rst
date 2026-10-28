motulator.grid.model
====================

.. py:module:: motulator.grid.model

.. autoapi-nested-parse::

   
   Continuous-time grid converter models.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.model.CarrierComparison
   motulator.grid.model.CompositeModel
   motulator.grid.model.DCBus
   motulator.grid.model.DiodeBridge
   motulator.grid.model.GridConverterSystem
   motulator.grid.model.LCLFilter
   motulator.grid.model.LFilter
   motulator.grid.model.Sensor
   motulator.grid.model.Simulation
   motulator.grid.model.SimulationResults
   motulator.grid.model.ThreePhaseSource
   motulator.grid.model.ThreePhaseSourceWithSignalInjection
   motulator.grid.model.VoltageSourceConverter
   motulator.grid.model.ZOH


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


.. py:class:: DiodeBridge(u_dc = None, u_th = 0.0)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
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


.. py:class:: GridConverterSystem(converter, ac_filter, ac_source, dc_bus = None)

   Bases: :py:obj:`motulator.common.model._base.Model`\ [\ :py:obj:`C`\ ]


   
   Continuous-time model for grid-converter systems.

   The converter is connected to the grid through the AC filter.

   :param converter: Converter model.
   :type converter: DiodeBridge | VoltageSourceConverter
   :param ac_filter: AC filter model.
   :type ac_filter: LFilter | LCLFilter
   :param ac_source: Three-phase voltage source.
   :type ac_source: ThreePhaseSource
   :param dc_bus: DC bus feeding the converter, which can be shared with other models (e.g., with
                  a drive), see `CompositeModel`. Defaults to None, in which case the converter
                  has a constant DC-bus voltage.
   :type dc_bus: DCBus, optional















   ..
       !! processed by numpydoc !!

.. py:class:: LCLFilter(L_fc, L_fg, C_f, R_fc = 0.0, R_fg = 0.0, L_g = 0.0, R_g = 0.0, u_f0_ab = 0j, i_c_sensor = None, u_g_sensor = None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Model of an LCL filter and an inductive-resistive grid.

   An LCL filter and an inductive-resistive grid impedance, between the converter and
   grid voltage sources, are modeled. The point-of-common-coupling (PCC) voltage
   between the LCL filter and the grid impedance is also calculated.

   :param L_fc: Converter-side filter inductance (H).
   :type L_fc: float
   :param L_fg: Grid-side filter inductance (H).
   :type L_fg: float
   :param C_f: Filter capacitance (F).
   :type C_f: float
   :param R_fc: Series resistance (Ω) of the converter-side inductor, defaults to 0.
   :type R_fc: float, optional
   :param R_fg: Series resistance (Ω) of the grid-side inductor, defaults to 0.
   :type R_fg: float, optional
   :param L_g: Grid inductance (H), defaults to 0.
   :type L_g: float, optional
   :param R_g: Grid resistance (Ω), defaults to 0.
   :type R_g: float, optional
   :param u_f0_ab: Initial value of the filter capacitor voltage (V), defaults to 0.
   :type u_f0_ab: complex, optional
   :param i_c_sensor: Converter phase-current sensors, defaults to an ideal measurement.
   :type i_c_sensor: Sensor, optional
   :param u_g_sensor: Line-to-line PCC voltage sensors (`u_ab` and `u_bc`), defaults to an ideal
                      measurement.
   :type u_g_sensor: Sensor, optional















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


   .. py:method:: meas_grid_currents()

      
      Measure the grid phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the line-to-line voltages u_ab and u_bc (V) at the point of common
      coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: pcc_voltage(state, inp)

      
      Compute the voltage at the point of common coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: LFilter(L_f, R_f = 0.0, L_g = 0.0, R_g = 0.0, i_c_sensor = None, u_g_sensor = None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Model of an L filter and an inductive-resistive grid.

   An L filter and an inductive-resistive grid, between the converter and grid voltage
   sources, are modeled combining their inductances and series resistances. The point-
   of-common-coupling (PCC) voltage between the L filter and the grid impedance is
   calculated.

   :param L_f: Filter inductance (H).
   :type L_f: float
   :param R_f: Series resistance (Ω) of the filter inductor, defaults to 0.
   :type R_f: float, optional
   :param L_g: Grid inductance (H), defaults to 0.
   :type L_g: float, optional
   :param R_g: Grid resistance (Ω), defaults to 0.
   :type R_g: float, optional
   :param i_c_sensor: Converter phase-current sensors, defaults to an ideal measurement.
   :type i_c_sensor: Sensor, optional
   :param u_g_sensor: Line-to-line PCC voltage sensors (`u_ab` and `u_bc`), defaults to an ideal
                      measurement.
   :type u_g_sensor: Sensor, optional















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


   .. py:method:: meas_currents()

      
      Measure the converter phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the line-to-line voltages u_ab and u_bc (V) at the PCC.
















      ..
          !! processed by numpydoc !!


   .. py:method:: pcc_voltage(state, inp)

      
      Compute the voltage at the point of common coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















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


.. py:class:: ThreePhaseSourceWithSignalInjection(w_g = 2 * pi * 50, e_g = sqrt(2 / 3) * 400, phi = 0.0, u_e = 0j, f_e = 0.0)

   Bases: :py:obj:`motulator.common.model.ThreePhaseSource`


   
   Three-phase source model with signal injection in synchronous coordinates.

   The excitation ``u_e*cos(2*pi*f_e*t)`` is added to the positive-sequence component
   in the synchronous coordinates of the source. This model is intended for converter
   output admittance measurement.

   :param w_g: Angular frequency (rad/s), defaults to 2*pi*50.
   :type w_g: float | Callable[[float], float], optional
   :param e_g: Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
   :type e_g: float | Callable[[float], float], optional
   :param phi: Phase shift (rad) of positive-sequence component, defaults to 0.
   :type phi: float | Callable[[float], float], optional
   :param u_e: Excitation (V) in synchronous coordinates, defaults to 0.
   :type u_e: complex, optional
   :param f_e: Excitation frequency (Hz), defaults to 0.
   :type f_e: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: generate_space_vector(t, exp_j_theta_g)

      
      Generate the space vector in stationary coordinates.
















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
   current direction is also determined at the latest switching instant. The DC-bus
   voltage is the input `u_dc`, which is constant unless a DC-bus model is connected
   to it (see `DCBus` and `Model`).

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


