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

.. py:class:: CarrierComparison(N = 2**12, return_complex = True)

   Bases: :py:obj:`PWM`


   
   Carrier comparison.

   This computes the the switching states and their durations based on the duty ratios.
   Instead of searching for zero crossings, the switching instants are explicitly
   computed in the beginning of each sampling period, allowing faster simulations.

   :param N: Amount of the counter quantization levels, defaults to 2**12.
   :type N: int, optional
   :param return_complex: Complex switching state space vectors are returned if True. Otherwise phase
                          switching states are returned, defaults to True.
   :type return_complex: bool, optional

   .. rubric:: Examples

   >>> from motulator.common.model import CarrierComparison
   >>> carrier_cmp = CarrierComparison(return_complex=False)
   >>> # First call gives rising edges
   >>> t_steps, q_abc = carrier_cmp(1e-3, [.4, .2, .8])
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
   >>> t_steps, q_abc = carrier_cmp(.001, [.4, .2, .8])
   >>> t_steps
   array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
   >>> q_abc
   array([[1, 1, 1],
          [1, 0, 1],
          [0, 0, 1],
          [0, 0, 0]])
   >>> # Sum of the step times equals T_s
   >>> np.sum(t_steps)
   0.001
   >>> # 50% duty ratios in all phases
   >>> t_steps, q_abc = carrier_cmp(1e-3, [.5, .5, .5])
   >>> t_steps
   array([0.0005, 0.    , 0.    , 0.0005])
   >>> q_abc
   array([[0, 0, 0],
          [0, 0, 0],
          [0, 0, 0],
          [1, 1, 1]])















   ..
       !! processed by numpydoc !!

.. py:class:: Model(pwm = False, delay = 0)

   
   Base class for continuous-time system models.

   This class defines the interface for continuous-time system models. It provides
   methods for setting initial values, computing state derivatives, interconnecting
   subsystems, and saving simulation results. The class also provides methods for
   setting ZOH inputs and computing outputs. The model can be configured to use either
   PWM or ZOH for the carrier comparison. The class also provides a method for saving
   the simulation results, which includes the time history and the ZOH inputs. The
   class is designed to be subclassed for specific applications.















   ..
       !! processed by numpydoc !!

   .. py:method:: get_initial_values()

      
      Get initial values of all subsystems before the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: interconnect()

      
      Connect subsystem inputs and outputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t, state_list)

      
      Compute complete state derivative list for the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: save(sol)

      
      Save solution with all ZOH inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Compute output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list)

      
      Set states in all subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_zoh_input(name, value)

      
      Set a specific ZOH input value.
















      ..
          !! processed by numpydoc !!


.. py:class:: ModelTimeSeries

   
   Container for simulation result time series.
















   ..
       !! processed by numpydoc !!

   .. py:method:: build_subsystem_time_series(subsystems, connections, zoh_connections)

      
      Build time series for all subsystems.
















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

   .. py:method:: simulate(t_stop = 1.0)

      
      Solve continuous-time system model and call control system.

      :param t_stop: Simulation stop time, defaults to 1.
      :type t_stop: float, optional















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


