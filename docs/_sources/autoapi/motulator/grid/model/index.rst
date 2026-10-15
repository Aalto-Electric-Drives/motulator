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

   motulator.grid.model.CompositeModel
   motulator.grid.model.DCBus
   motulator.grid.model.GridConverterSystem
   motulator.grid.model.LCLFilter
   motulator.grid.model.LFilter
   motulator.grid.model.Sensor
   motulator.grid.model.Simulation
   motulator.grid.model.SimulationResults
   motulator.grid.model.ThreePhaseSource
   motulator.grid.model.ThreePhaseSourceWithSignalInjection
   motulator.grid.model.VoltageSourceConverter


Package Contents
----------------

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


.. py:class:: GridConverterSystem(converter, ac_filter, ac_source, dc_bus = None, pwm = False, delay = 1, single_update = False)

   Bases: :py:obj:`motulator.common.model._base.Model`\ [\ :py:obj:`motulator.common.model._converter.VoltageSourceConverter`\ ]


   
   Continuous-time model for grid-converter systems.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter
   :param ac_filter: AC filter model.
   :type ac_filter: LFilter | LCLFilter
   :param ac_source: Three-phase voltage source.
   :type ac_source: ThreePhaseSource
   :param dc_bus: DC bus feeding the converter, which can be shared with other models (e.g., with
                  a drive), see `CompositeModel`. Defaults to None, in which case the converter
                  has a constant DC-bus voltage.
   :type dc_bus: DCBus, optional
   :param pwm: Enable PWM model, defaults to False.
   :type pwm: bool, optional
   :param delay: Computational delay in half carrier periods (sampling periods with double
                 update), defaults to 1. With single update, use 2 for a delay of one
                 sampling period, see `Model`.
   :type delay: int, optional
   :param single_update: Sample and update the duty ratios once per carrier period instead of twice,
                         defaults to False, see `Model`.
   :type single_update: bool, optional















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

.. py:class:: ThreePhaseSource(w_g = 2 * pi * 50, e_g = sqrt(2 / 3) * 400, phi = 0.0, e_g_neg = 0.0, phi_neg = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
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

   Bases: :py:obj:`ThreePhaseSource`


   
   Three-phase source model with signal injection in synchronous coordinates.

   The excitation ``u_e*cos(2*pi*f_e*t)`` is added to the positive-sequence component
   in the synchronous coordinates of the source. This model is intended for converter
   output admittance identification.

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


   .. py:method:: rhs(t)

      
      Default empty implementation.
















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


