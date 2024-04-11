:orphan:

:py:mod:`motulator.model._simulation`
=====================================

.. py:module:: motulator.model._simulation

.. autoapi-nested-parse::

   Simulation environment.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model._simulation.Delay
   motulator.model._simulation.CarrierComparison
   motulator.model._simulation.Simulation
   motulator.model._simulation.Model



Functions
~~~~~~~~~

.. autoapisummary::

   motulator.model._simulation.zoh



.. py:class:: Delay(length=1, elem=3)


   
   Computational delay.

   This models the computational delay as a ring buffer.

   :param length: Length of the buffer in samples. The default is 1.
   :type length: int, optional















   ..
       !! processed by numpydoc !!

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

.. py:function:: zoh(T_s, d_abc)

   
   Zero-order hold of the duty ratios over the sampling period.

   :param T_s: Sampling period.
   :type T_s: float
   :param d_abc: Duty ratios in the range [0, 1].
   :type d_abc: array_like of floats, shape (3,)

   :returns: * **t_steps** (*ndarray, shape (1,)*) -- Sampling period as an array compatible with the solver.
             * **q** (*complex ndarray, shape (1,)*) -- Duty ratio vector as an array compatible with the solver.















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


.. py:class:: Model(pwm=None, delay=1)


   
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
   .. py:method:: clear()

      
      Clear the simulation data of the system model.

      This method is automatically run when the instance for the system model
      is created. It can also be used in the case of repeated simulations to
      clear the data from the previous simulation run.















      ..
          !! processed by numpydoc !!

   .. py:method:: get_initial_values()
      :abstractmethod:

      
      Get the initial values.

      :returns: **x0** -- Initial values of the state variables.
      :rtype: complex list















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)
      :abstractmethod:

      
      Set the initial values.

      :param t0: Initial time (s).
      :type t0: float
      :param x0: Initial values of the state variables.
      :type x0: complex ndarray















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)
      :abstractmethod:

      
      Compute the complete state derivative list for the solver.

      :param t: Time (s).
      :type t: float
      :param x: State vector.
      :type x: complex ndarray

      :returns: State derivatives.
      :rtype: complex list















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Save the solution.

      :param sol: Solution from the solver.
      :type sol: Bunch















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


