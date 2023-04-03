:py:mod:`motulator.simulation`
==============================

.. py:module:: motulator.simulation

.. autoapi-nested-parse::

   Simulation environment.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.simulation.Delay
   motulator.simulation.CarrierCmp
   motulator.simulation.Simulation



Functions
~~~~~~~~~

.. autoapisummary::

   motulator.simulation.zoh



.. py:class:: Delay(length=1, elem=3)

   
   Computational delay.

   This models the computational delay as a ring buffer.

   :param length: Length of the buffer in samples. The default is 1.
   :type length: int, optional















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(u)

      
      Delay the input.

      :param u: Input array.
      :type u: array_like, shape (elem,)

      :returns: Output array.
      :rtype: array_like, shape (elem,)















      ..
          !! processed by numpydoc !!


.. py:class:: CarrierCmp(N=2**12, return_complex=True)

   
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

   >>> from motulator.simulation import CarrierCmp
   >>> carrier_cmp = CarrierCmp(return_complex=False)
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
   .. py:method:: __call__(T_s, d_abc)

      
      Compute the switching state durations and vectors.

      :param T_s: Half carrier period.
      :type T_s: float
      :param d_abc: Duty ratios in the range [0, 1].
      :type d_abc: array_like of floats, shape (3,)

      :returns: * **t_steps** (*ndarray, shape (4,)*) -- Switching state durations, `[t0, t1, t2, t3]`.
                * **q** (*complex ndarray, shape (4,)*) -- Switching state vectors, `[q0, q1, q2, q3]`, where `q1` and `q2`
                  are active vectors.

      .. rubric:: Notes

      No switching (e.g. `d_a == 0` or `d_a == 1`) or simultaneous switchings
      (e.g. `d_a == d_b`) lead to zeroes in `t_steps`.















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

.. py:class:: Simulation(mdl=None, ctrl=None, delay=1, pwm=False)

   
   Simulation environment.

   Each simulation object has a system model object and a controller object.

   :param mdl: Continuous-time system model.
   :type mdl: InductionMotorDrive | SynchronousMotorDrive
   :param ctrl: Discrete-time controller.
   :type ctrl: Ctrl
   :param delay: Amount of computational delays. The default is 1.
   :type delay: int, optional
   :param pwm: Enable carrier comparison. The default is False.
   :type pwm: bool, optional















   ..
       !! processed by numpydoc !!
   .. py:method:: simulate(t_stop=1, max_step=np.inf)

      
      Solve the continuous-time model and call the discrete-time controller.

      :param t_stop: Simulation stop time. The default is 1.
      :type t_stop: float, optional
      :param max_step: Max step size of the solver. The default is inf.
      :type max_step: float, optional

      .. rubric:: Notes

      Other options of solve_ivp could be easily changed if needed, but, for
      simplicity, only max_step is included as an option of this method.















      ..
          !! processed by numpydoc !!

   .. py:method:: simulation_loop(t_stop, max_step)

      
      Run the main simulation loop.
















      ..
          !! processed by numpydoc !!

   .. py:method:: save_mat(name='sim')

      
      Save the simulation data into MATLAB .mat files.

      :param name: Name for the simulation instance. The default is 'sim'.
      :type name: str, optional















      ..
          !! processed by numpydoc !!


