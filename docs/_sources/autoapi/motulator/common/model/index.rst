motulator.common.model
======================

.. py:module:: motulator.common.model

.. autoapi-nested-parse::

   
   Common functions and classes for continuous-time system models.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.common.model.CarrierComparison
   motulator.common.model.Delay
   motulator.common.model.Model
   motulator.common.model.Simulation
   motulator.common.model.Subsystem
   motulator.common.model.FrequencyConverter
   motulator.common.model.VoltageSourceConverter


Functions
---------

.. autoapisummary::

   motulator.common.model.zoh


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

.. py:class:: Model(pwm=None, delay=1)

   Bases: :py:obj:`abc.ABC`


   
   Base class for continuous-time system models.

   This base class is a template for a system model that interconnects the
   subsystems and provides an interface to the solver.

   :param pwm: Zero-order hold of duty ratios or carrier comparison. If None, the
               default is `zoh`.
   :type pwm: zoh | CarrierComparison, optional
   :param delay: Amount of computational delays. The default is 1.
   :type delay: int, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: get_initial_values()

      
      Get initial values of all subsystems before the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_states(state_list)

      
      Set the states in all subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Compute the output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Compute the input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: interconnect(t)
      :abstractmethod:


      
      Interconnect the subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t, state_list)

      
      Compute the complete state derivative list for the solver.
















      ..
          !! processed by numpydoc !!


   .. py:method:: save(sol)

      
      Save the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process after the inputs have been added.
















      ..
          !! processed by numpydoc !!


.. py:class:: Simulation(mdl=None, ctrl=None)

   
   Simulation environment.

   Each simulation object has a system model object and a controller object.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time controller.
   :type ctrl: ControlSystem















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


.. py:class:: Subsystem

   Bases: :py:obj:`abc.ABC`


   
   Base class for subsystems.
















   ..
       !! processed by numpydoc !!

.. py:function:: zoh(T_s, d_c_abc)

   
   Zero-order hold of the duty ratios over the sampling period.

   :param T_s: Sampling period.
   :type T_s: float
   :param d_c_abc: Duty ratios in the range [0, 1].
   :type d_c_abc: array_like of floats, shape (3,)

   :returns: * **t_steps** (*ndarray, shape (1,)*) -- Sampling period as an array compatible with the solver.
             * **q_cs** (*complex ndarray, shape (1,)*) -- Duty ratio vector as an array compatible with the solver.















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

   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















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


.. py:class:: VoltageSourceConverter(u_dc, C_dc=None, i_ext=lambda t: None)

   Bases: :py:obj:`motulator.common.model._simulation.Subsystem`


   
   Lossless three-phase voltage-source converter.

   :param u_dc: DC-bus voltage (V). If the DC-bus capacitor is modeled, this value is
                used as the initial condition.
   :type u_dc: float
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional
   :param i_ext: External current (A) fed to the DC bus. Needed if `C_dc` is not None.
   :type i_ext: callable, optional















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


   .. py:method:: set_inputs(t)

      
      Set input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















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


