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

   motulator.common.model.CarrierComparison
   motulator.common.model.Model
   motulator.common.model.ModelTimeSeries
   motulator.common.model.Simulation
   motulator.common.model.SimulationResults
   motulator.common.model.SolverCfg
   motulator.common.model.Subsystem
   motulator.common.model.SubsystemTimeSeries


Package Contents
----------------

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

.. py:class:: Model(converter, subsystems, connections, pwm = False, delay = 0)

   
   Base class for continuous-time system models.

   A model consists of subsystems and connections between them. The converter
   subsystem gets the switching state `q_c_ab`, which is held constant over each
   integration interval. The PWM model includes the dead time `converter.t_d`, and the
   computational delay is sized according to the number of legs `converter.n_legs`. The
   outputs are computed in the order of the `subsystems` list and passed to the
   connected inputs immediately. Hence, a subsystem whose outputs depend directly on
   its inputs must come after the subsystems providing these inputs.

   :param converter: Converter model.
   :type converter: Converter
   :param subsystems: All subsystems, including the converter.
   :type subsystems: list[Subsystem]
   :param connections: Connections as `{(target, input_name): (source, output_name)}`.
   :type connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]]
   :param pwm: Enable PWM model, defaults to False.
   :type pwm: bool, optional
   :param delay: Computational delay (samples), defaults to 0.
   :type delay: int, optional















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

      
      Save the solution and the switching state.
















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

.. py:class:: Simulation(mdl, ctrl, show_progress = True, cfg = None)

   
   Simulation environment.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time control system.
   :type ctrl: ControlSystem
   :param show_progress: Show progress during simulation, defaults to True.
   :type show_progress: bool, optional
   :param cfg: Solver configuration parameters.
   :type cfg: SolverCfg, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: simulate(t_stop = 1.0, N_eval = 0)

      
      Solve continuous-time system model and call control system.

      :param t_stop: Simulation stop time, defaults to 1.
      :type t_stop: float, optional
      :param N_eval: Number of evenly spaced data points to be returned by the solver for each
                     sampling period. Defaults to 0, in which case the number and spacing of
                     points is selected by the solver.
      :type N_eval: int | None, optional















      ..
          !! processed by numpydoc !!


.. py:class:: SimulationResults

   
   Container for simulation results.

   .. attribute:: mdl

      Results from the continuous-time model.

      :type: ModelTimeSeries

   .. attribute:: ctrl

      Results from the digital control system.

      :type: Any















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
   representations of the subsystem.















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
















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_input_derived_signals(t, subsystem)

      
      Compute additional time series using subsystem's regular inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_zoh_input_derived_signals(t, subsystem)

      
      Compute additional time series using subsystem's ZOH inputs.
















      ..
          !! processed by numpydoc !!


