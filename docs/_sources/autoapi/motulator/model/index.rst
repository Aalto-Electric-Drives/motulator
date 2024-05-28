motulator.model
===============

.. py:module:: motulator.model

.. autoapi-nested-parse::

   
   Continuous-time system models.
















   ..
       !! processed by numpydoc !!


Subpackages
-----------

.. toctree::
   :maxdepth: 1

   /autoapi/motulator/model/im/index
   /autoapi/motulator/model/sm/index


Classes
-------

.. autoapisummary::

   motulator.model.Mechanics
   motulator.model.MechanicsTwoMass
   motulator.model.FrequencyConverter
   motulator.model.Inverter
   motulator.model.LCFilter
   motulator.model.CarrierComparison
   motulator.model.Simulation
   motulator.model.Delay


Functions
---------

.. autoapisummary::

   motulator.model.zoh


Package Contents
----------------

.. py:class:: Mechanics(J, tau_L_w=lambda w_M: 0 * w_M, tau_L_t=lambda t: 0 * t)

   
   Mechanics subsystem.

   This models an equation of motion for stiff mechanics.

   :param J: Total moment of inertia (kgm²).
   :type J: float
   :param tau_L_w: Load torque (Nm) as a function of speed, `tau_L_w(w_M)`. For example,
                   ``tau_L_w = b*w_M``, where `b` is the viscous friction coefficient. The
                   default is zero, ``lambda w_M: 0*w_M``.
   :type tau_L_w: callable
   :param tau_L_t: Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
                   zero, ``lambda t: 0*t``.
   :type tau_L_t: callable















   ..
       !! processed by numpydoc !!

   .. py:method:: f(t, w_M, tau_M)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float
      :param tau_M: Electromagnetic torque (Nm).
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      This returns the rotor speed at the end of the sampling period.

      :returns: **w_M0** -- Rotor angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure the rotor angle.

      This returns the rotor angle at the end of the sampling period.

      :returns: **theta_M0** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: MechanicsTwoMass(J_M, J_L, K_S, C_S, tau_L_w=None, tau_L_t=None)

   Bases: :py:obj:`Mechanics`


   
   Two-mass mechanics subsystem.

   This models an equation of motion for two-mass mechanics.

   :param J_M: Motor moment of inertia (kgm²).
   :type J_M: float
   :param J_L: Load moment of inertia (kgm²).
   :type J_L: float
   :param K_S: Shaft torsional stiffness (Nm).
   :type K_S: float
   :param C_S: Shaft torsional damping (Nms).
   :type C_S: float
   :param tau_L_w: Load torque (Nm) as a function of the load speed, `tau_L_w(w_L)`, e.g.,
                   ``tau_L_w = B*w_L``, where `B` is the viscous friction coefficient. The
                   default is zero, ``lambda w_L: 0*w_L``.
   :type tau_L_w: callable
   :param tau_L_t: Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
                   zero, ``lambda t: 0*t``.
   :type tau_L_t: callable















   ..
       !! processed by numpydoc !!

   .. py:method:: f(t, w_M, w_L, theta_ML, tau_M)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param w_M: Rotor angular speed (mechanical rad/s).
      :type w_M: float
      :param w_L: Load angular speed (mechanical rad/s).
      :type w_L: float
      :param theta_ML: Twist angle, theta_M - theta_L (mechanical rad).
      :type theta_ML: float
      :param tau_M: Electromagnetic torque (Nm).
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 4















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_speed()

      
      Measure the load speed.

      This returns the load speed at the end of the sampling period.

      :returns: **w_L0** -- Load angular speed (mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_position()

      
      Measure the load angle.

      This returns the load angle at the end of the sampling period.

      :returns: **theta_L0** -- Rotor angle (mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(L, C, U_g, f_g)

   Bases: :py:obj:`Inverter`


   
   Frequency converter.

   This extends the Inverter class with models for a strong grid, a
   three-phase diode-bridge rectifier, an LC filter, and a three-phase
   inverter.

   :param L: DC-bus inductance (H).
   :type L: float
   :param C: DC-bus capacitance (F).
   :type C: float
   :param U_g: Grid voltage (V, line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency (Hz).
   :type f_g: float















   ..
       !! processed by numpydoc !!

   .. py:method:: grid_voltages(t)

      
      Compute three-phase grid voltages.

      :param t: Time (s).
      :type t: float

      :returns: **u_g_abc** -- Phase voltages (V).
      :rtype: ndarray of floats, shape (3,)















      ..
          !! processed by numpydoc !!


   .. py:method:: f(t, u_dc, i_L, i_dc)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param u_dc: DC-bus voltage (V) over the capacitor.
      :type u_dc: float
      :param i_L: DC-bus inductor current (A).
      :type i_L: float
      :param i_dc: Current to the inverter (A).
      :type i_dc: float

      :returns: Time derivative of the state vector, [du_dc, di_L]
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!


.. py:class:: Inverter(u_dc)

   
   Inverter with constant DC-bus voltage and switching-cycle averaging.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float















   ..
       !! processed by numpydoc !!

   .. py:method:: ac_voltage(q, u_dc)
      :staticmethod:


      
      Compute the AC-side voltage of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **u_c** -- AC-side converter voltage (V).
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: dc_current(q, i_c)
      :staticmethod:


      
      Compute the DC-side current of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param i_c: AC-side converter current (A).
      :type i_c: complex

      :returns: **i_dc** -- DC-side current (A).
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure the DC-bus voltage.

      :returns: DC-bus voltage (V).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: LCFilter(L, C, R=0)

   
   LC-filter model.

   :param L: Inductance (H).
   :type L: float
   :param C: Capacitance (F).
   :type C: float
   :param R: Series resistance (Ω) of the inductor. The default is 0.
   :type R: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: f(i_cs, u_ss, u_cs, i_ss)

      
      Compute the state derivative.

      :param i_cs: Converter output current (A).
      :type i_cs: complex
      :param u_ss: Capacitor (stator) voltage (V).
      :type u_ss: complex
      :param u_cs: Converter output voltage (V).
      :type u_cs: complex
      :param i_ss: Stator current (A).
      :type i_ss: complex

      :returns: Time derivative of the state vector, [di_cs, du_ss]
      :rtype: complex list, length 2















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Returns the converter phase currents at the end of the sampling period.

      :returns: **i_c_abc** -- Phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_voltages()

      
      Returns the capacitor (stator) phase voltages at the end of the
      sampling period.

      :returns: **u_s_abc** -- Phase voltages (V).
      :rtype: 3-tuple of floats















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


.. py:class:: Delay(length=1, elem=3)

   
   Computational delay.

   This models the computational delay as a ring buffer.

   :param length: Length of the buffer in samples. The default is 1.
   :type length: int, optional















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

