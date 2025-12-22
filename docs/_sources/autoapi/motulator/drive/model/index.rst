motulator.drive.model
=====================

.. py:module:: motulator.drive.model

.. autoapi-nested-parse::

   
   Continuous-time machine drive models.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.model.Drive
   motulator.drive.model.ExternalRotorSpeed
   motulator.drive.model.FrequencyConverter
   motulator.drive.model.InductionMachine
   motulator.drive.model.InductionMachineInvGammaPars
   motulator.drive.model.InductionMachinePars
   motulator.drive.model.LCFilter
   motulator.drive.model.MechanicalSystem
   motulator.drive.model.SaturatedSynchronousMachinePars
   motulator.drive.model.Simulation
   motulator.drive.model.SpatialSaturatedSynchronousMachinePars
   motulator.drive.model.SynchronousMachine
   motulator.drive.model.SynchronousMachinePars
   motulator.drive.model.TwoMassMechanicalSystem
   motulator.drive.model.VoltageSourceConverter


Package Contents
----------------

.. py:class:: Drive(machine, mechanics, converter, lc_filter = None, pwm = False, delay = 1)

   Bases: :py:obj:`motulator.common.model._base.Model`


   
   Continuous-time system model for a machine drive.

   :param machine: Electric machine model.
   :type machine: InductionMachine | SynchronousMachine
   :param mechanics: Mechanical system model.
   :type mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
   :param converter: Converter model.
   :type converter: VoltageSourceConverter | FrequencyConverter
   :param lc_filter: LC filter model. If not given, a direct connection between the converter and
                     machine is used.
   :type lc_filter: LCFilter, optional
   :param pwm: Enable PWM model, defaults to False.
   :type pwm: bool, optional
   :param delay: Computational delay (samples), defaults to 1.
   :type delay: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ExternalRotorSpeed

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Integrate rotor angle from externally given rotor speed.

   This class maintains the same interface as other mechanical systems but the speed is
   determined by an external function rather than by torque dynamics.















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)
      :abstractmethod:


      
      Set external load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(w_M)

      
      Set external rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(C_dc, L_dc, U_g, f_g)

   Bases: :py:obj:`VoltageSourceConverter`


   
   Frequency converter with a six-pulse diode bridge.

   A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The diode
   bridge is connected to the voltage-source inverter. The grid inductance is zero.

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

   .. py:method:: compute_voltages(state)

      
      Compute grid and rectified voltages.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables for interconnection.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachine(par)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Γ-equivalent model of an induction machine.

   An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. The stator
   inductance `L_s` can either be constant or a function of the stator flux magnitude::

       L_s = L_s(abs(psi_s_ab))

   :param par: Machine parameters.
   :type par: InductionMachinePars | InductionMachineInvGammaPars

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic saturation
   model in a straightforward manner. If the magnetic saturation is omitted, the Γ
   model is mathematically identical to the inverse-Γ and T models [#Sle1989]_.

   .. rubric:: References

   .. [#Sle1989] Slemon, "Modelling of induction machines for electric
      drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_outputs(state)

      
      Compute output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure phase currents (A).
















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


.. py:class:: InductionMachineInvGammaPars

   
   Constant inverse-Γ model parameters of an induction machine.

   This contains constant inverse-Γ model parameters of an induction machine. To model
   the main-flux saturation, use the `InductionMachinePars` class instead.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_R: Rotor resistance (Ω).
   :type R_R: float
   :param L_sgm: Leakage inductance (H).
   :type L_sgm: float
   :param L_M: Magnetizing inductance (H).
   :type L_M: float

   .. attribute:: R_sgm

      Inverse-Γ total resistance `R_s` plus `R_R` (Ω).

      :type: float

   .. attribute:: alpha

      Inverse rotor time constant (rad/s).

      :type: float

   .. attribute:: w_rb

      Breakdown slip angular frequency (rad/s).

      :type: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_gamma_pars(par)
      :classmethod:


      
      Compute inverse-Γ model parameters from Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Γ-model parameters.
      :type par: InductionMachinePars

      :returns: Inverse-Γ model parameters.
      :rtype: InductionMachineInvGammaPars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMachinePars

   
   Γ-model parameters of an induction machine.

   This contains Γ-model parameters of an induction machine. The main-flux saturation
   saturation can also be modeled by providing a callable `L_s` parameter. For
   convenience, the class also provides the corresponding inverse-Γ model parameters,
   which can be used in control systems. If the saturation is modeled, these inverse-Γ
   parameters depend on the stator flux linkage magnitude `psi_s` that should be
   updated by calling the `update_psi_s` method.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H). If callable, it should be a function of the stator flux
               linkage magnitude (Vs).
   :type L_s: float | Callable[[float], float]

   .. attribute:: gamma

      Magnetic coupling factor.

      :type: float

   .. attribute:: L_M

      Inverse-Γ magnetizing inductance (H).

      :type: float

   .. attribute:: L_sgm

      Inverse-Γ leakage inductance (H).

      :type: float

   .. attribute:: R_R

      Inverse-Γ rotor resistance (Ω).

      :type: float

   .. attribute:: R_sgm

      Inverse-Γ total resistance `R_s` plus `R_R` (Ω).

      :type: float

   .. attribute:: alpha

      Inverse rotor time constant (rad/s).

      :type: float

   .. attribute:: w_rb

      Breakdown slip angular frequency (rad/s).

      :type: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_inv_gamma_pars(par)
      :classmethod:


      
      Compute Γ-model parameters from inverse-Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Inverse-Γ model parameters.
      :type par: InductionMachineInvGammaPars

      :returns: Γ model parameters.
      :rtype: InductionMachinePars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: LCFilter(L_f, C_f, R_f = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   LC-filter model.

   :param L_f: Filter inductance (H).
   :type L_f: float
   :param C_f: Filter capacitance (F).
   :type C_f: float
   :param R_f: Series resistance (Ω) of the inductor, defaults to 0.
   :type R_f: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor phase voltages.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents.
















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


.. py:class:: MechanicalSystem(J, B_L = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Stiff mechanical system.

   :param J: Total moment of inertia (kgm²).
   :type J: float
   :param B_L: Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
               friction, or an arbitrary function of the rotor speed. For example, choosing
               ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic load torque ``k*w_M**2``.
               The default is ``B_L = 0``.
   :type B_L: float | Callable[[float], float]















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_total_load_torque(t, state)

      
      Total load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)

      
      Set external load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(_)
      :abstractmethod:


      
      Set external rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: SaturatedSynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Parameters of a saturated synchronous machine.

   The saturation model is specified as a current map (current as a function of the
   flux linkage). Optionally, to be used only in control systems, a flux map (flux
   linkage as a function of the current) can be provided. For convenience, this class
   also provides the incremental inductance matrix, which can be used in control and
   optimal reference generation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param i_s_dq_fcn: Stator current (A) as a function of the stator flux linkage (Vs). Needed in the
                      system model and in some control methods.
   :type i_s_dq_fcn: Callable[[complex], complex], optional
   :param psi_s_dq_fcn: Stator flux linkage (Vs) as a function of the stator current (A). This function
                        should be differentiable, if incremental inductances are used. Needed only for
                        control methods and optimal reference loci, not used in the system model.
   :type psi_s_dq_fcn: Callable[[complex], complex], optional
   :param use_iterative_current: If `i_s_dq_fcn` is not provided, the current is computed iteratively from the
                                 flux map using a root-finding algorithm. This is less efficient, but may be
                                 convenient to parametrize some control methods if only the flux map is
                                 available. Defaults to `False`.
   :type use_iterative_current: bool, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix at given current.
















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage as a function of the stator current.
















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


.. py:class:: SpatialSaturatedSynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Parameters of a saturated synchronous machine with spatial harmonics.

   This saturation model contains spatial harmonics in addition to the saturation
   effects. This version is intended to parametrize high-fidelity machine models for
   simulation purposes, while control methods do not support spatial harmonics.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param magnetic_map_fcn: Stator current (A) and electromagnetic torque (Nm) per pole pair as functions of
                            the stator flux linkage (Vs) and the complex exponential of the electrical rotor
                            angle.
   :type magnetic_map_fcn: Callable[[complex, complex], Tuple[complex, float]]















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m = None)

      
      Current (A) as a function of flux linkage (Vs) and rotor angle.
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)
      :abstractmethod:


      
      Incremental inductance matrix.

      :param i_s_dq: Stator current (A) in rotor coordinates.
      :type i_s_dq: complex | ndarray
      :param exp_j_theta_m: Complex exponential of the electrical rotor angle.
      :type exp_j_theta_m: complex | ndarray, optional

      :returns: Incremental inductance matrix (H).
      :rtype: ndarray















      ..
          !! processed by numpydoc !!


   .. py:method:: magnetic_map(psi_s_dq, exp_j_theta_m = None)

      
      Magnetic map.

      :param psi_s_dq: Stator flux linkage (Vs) in rotor coordinates.
      :type psi_s_dq: complex | ndarray
      :param exp_j_theta_m: Complex exponential of electrical rotor angle.
      :type exp_j_theta_m: complex | ndarray

      :returns: Stator current (A) in rotor coordinates and electromagnetic torque (Nm) per
                pole pair.
      :rtype: Tuple [complex | ndarray, float | ndarray]















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)
      :abstractmethod:


      
      Flux linkage as a function of current and electrical rotor angle.

      :param i_s_dq: Stator current (A) in rotor coordinates.
      :type i_s_dq: complex | ndarray
      :param exp_j_theta_m: Complex exponential of the electrical rotor angle.
      :type exp_j_theta_m: complex | ndarray, optional

      :returns: Stator flux linkage in rotor coordinates (Vs).
      :rtype: complex | ndarray















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMachine(par)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Synchronous machine model.

   This model is internally represented in rotor coordinates, which results in the
   simplest implementation. The interfaces are in stator coordinates.

   :param par: Machine parameters. The magnetic saturation can be modeled by providing a
               nonlinear current map par.i_s_dq (callable).
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars         | SpatialSaturatedSynchronousMachinePars















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_outputs(state)

      
      Compute output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure phase currents (A).
















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


.. py:class:: SynchronousMachinePars

   Bases: :py:obj:`BaseSynchronousMachinePars`


   
   Synchronous machine parameters, without saturation.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param L_d: d-axis inductance (H).
   :type L_d: float
   :param L_q: q-axis inductance (H).
   :type L_q: float
   :param psi_f: Permanent-magnet flux linkage (Vs).
   :type psi_f: float















   ..
       !! processed by numpydoc !!

   .. py:method:: i_s_dq(psi_s_dq, exp_j_theta_m=None)

      
      Current (A) as a function of the flux linkage (Vs).
















      ..
          !! processed by numpydoc !!


   .. py:method:: incr_ind_mat(i_s_dq, exp_j_theta_m=None)

      
      Incremental inductance matrix (H).
















      ..
          !! processed by numpydoc !!


   .. py:method:: psi_s_dq(i_s_dq, exp_j_theta_m=None)

      
      Flux linkage (Vs) as a function of the stator current (A).
















      ..
          !! processed by numpydoc !!


.. py:class:: TwoMassMechanicalSystem(J_M, J_L, K_S, C_S, B_L = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Two-mass mechanical subsystem.

   :param J_M: Motor moment of inertia (kgm²).
   :type J_M: float
   :param J_L: Load moment of inertia (kgm²).
   :type J_L: float
   :param K_S: Shaft torsional stiffness (Nm/rad).
   :type K_S: float
   :param C_S: Shaft torsional damping (Nm/(rad/s)).
   :type C_S: float
   :param B_L: Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
               friction, or an arbitrary function of the load speed. For example, choosing
               ``B_L = lambda w_L: k*abs(w_L)`` leads to quadratic load torque ``k*w_L**2``.
               The default is ``B_L = 0``.
   :type B_L: float | Callable[[float], float]















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_torques(t, state)

      
      Compute shaft and load torques (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_position()

      
      Measure the load angle.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_load_speed()

      
      Measure the load speed.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_position()

      
      Measure mechanical rotor angle (rad).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_speed()

      
      Measure mechanical rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_load_torque(tau_L)

      
      Set external load torque (Nm).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_rotor_speed(_)
      :abstractmethod:


      
      Set external rotor speed (rad/s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: VoltageSourceConverter(u_dc)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Lossless three-phase voltage-source converter with constant DC-bus voltage.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float















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


   .. py:method:: set_external_dc_current(i_dc)
      :abstractmethod:


      
      Set external DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


