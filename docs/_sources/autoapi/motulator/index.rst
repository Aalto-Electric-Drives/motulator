:py:mod:`motulator`
===================

.. py:module:: motulator

.. autoapi-nested-parse::

   
   Import simulation environment.
















   ..
       !! processed by numpydoc !!


Subpackages
-----------
.. toctree::
   :titlesonly:
   :maxdepth: 3

   control/index.rst
   model/index.rst


Submodules
----------
.. toctree::
   :titlesonly:
   :maxdepth: 1

   helpers/index.rst
   plots/index.rst
   simulation/index.rst


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.Simulation
   motulator.Mechanics
   motulator.MechanicsTwoMass
   motulator.Inverter
   motulator.FrequencyConverter
   motulator.InductionMotor
   motulator.InductionMotorSaturated
   motulator.InductionMotorInvGamma
   motulator.SynchronousMotor
   motulator.SynchronousMotorSaturated
   motulator.SynchronousMotorSaturatedLUT
   motulator.InductionMotorDrive
   motulator.InductionMotorDriveDiode
   motulator.InductionMotorDriveTwoMass
   motulator.SynchronousMotorDrive
   motulator.SynchronousMotorDriveTwoMass
   motulator.InductionMotorVHzCtrl
   motulator.InductionMotorVHzCtrlPars
   motulator.InductionMotorVectorCtrl
   motulator.InductionMotorVectorCtrlPars
   motulator.SynchronousMotorVectorCtrl
   motulator.SynchronousMotorVectorCtrlPars
   motulator.TorqueCharacteristics
   motulator.SynchronousMotorFluxVectorCtrl
   motulator.SynchronousMotorFluxVectorCtrlPars
   motulator.SynchronousMotorSignalInjectionCtrl
   motulator.SynchronousMotorSignalInjectionCtrlPars
   motulator.InductionMotorVHzObsCtrl
   motulator.InductionMotorObsVHzCtrlPars
   motulator.SynchronousMotorVHzObsCtrl
   motulator.SynchronousMotorVHzObsCtrlPars
   motulator.BaseValues
   motulator.Sequence
   motulator.Step



Functions
~~~~~~~~~

.. autoapisummary::

   motulator.import_syre_data
   motulator.plot_flux_map
   motulator.plot_flux_vs_current
   motulator.abc2complex
   motulator.complex2abc
   motulator.plot
   motulator.plot_extra



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


.. py:class:: Mechanics(J=0.015, tau_L_w=lambda w_M: 0 * w_M, tau_L_t=lambda t: 0 * t)

   
   Mechanics subsystem.

   This models an equation of motion for stiff mechanics.

   :param J: Total moment of inertia.
   :type J: float
   :param tau_L_w: Load torque as function of speed, `tau_L_w(w_M)`. For example,
                   tau_L_w = b*w_M, where b is the viscous friction coefficient.
   :type tau_L_w: function
   :param tau_L_t: Load torque as a function of time, `tau_L_t(t)`.
   :type tau_L_t: function















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, tau_M)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float
      :param tau_M: Electromagnetic torque.
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_speed()

      
      Measure the rotor speed.

      This returns the rotor speed at the end of the sampling period.

      :returns: **w_M0** -- Rotor angular speed (in mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_position()

      
      Measure the rotor angle.

      This returns the rotor angle at the end of the sampling period.

      :returns: **theta_M0** -- Rotor angle (in mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: MechanicsTwoMass(J_M=0.005, J_L=0.005, K_S=700.0, C_S=0.13, tau_L_w=lambda w_M: 0 * w_M, tau_L_t=lambda t: 0 * t)

   Bases: :py:obj:`Mechanics`

   
   Two-mass mechanics subsystem.

   This models an equation of motion for two-mass mechanics.

   :param J_M: Moment of inertia of the motor.
   :type J_M: float
   :param J_L: Moment of inertia of the load.
   :type J_L: float
   :param K_S: Torsional stiffness of the shaft.
   :type K_S: float
   :param C_S: Torsional damping of the shaft.
   :type C_S: float
   :param tau_L_w: Load torque as function of the load speed, `tau_L_w(w_L)`. For example,
                   tau_L_w = b*w_L, where b is the viscous friction coefficient.
   :type tau_L_w: function
   :param tau_L_t: Load torque as a function of time, `tau_L_t(t)`.
   :type tau_L_t: function















   ..
       !! processed by numpydoc !!
   .. py:method:: f(t, w_M, w_L, theta_ML, tau_M)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float
      :param w_L: Load angular speed (in mechanical rad/s).
      :type w_L: float
      :param theta_ML: Twist angle, theta_M - theta_L (in mechanical rad).
      :type theta_ML: float
      :param tau_M: Electromagnetic torque.
      :type tau_M: float

      :returns: Time derivatives of the state vector.
      :rtype: list, length 4















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_speed()

      
      Measure the load speed.

      This returns the load speed at the end of the sampling period.

      :returns: **w_L0** -- Load angular speed (in mechanical rad/s).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_load_position()

      
      Measure the load angle.

      This returns the load angle at the end of the sampling period.

      :returns: **theta_L0** -- Rotor angle (in mechanical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: Inverter(u_dc=540)

   
   Inverter with constant DC-bus voltage and switching-cycle averaging.

   :param u_dc: DC-bus voltage.
   :type u_dc: float















   ..
       !! processed by numpydoc !!
   .. py:method:: ac_voltage(q, u_dc)
      :staticmethod:

      
      Compute the AC-side voltage of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param u_dc: DC-bus voltage.
      :type u_dc: float

      :returns: **u_ac** -- AC-side voltage.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: dc_current(q, i_ac)
      :staticmethod:

      
      Compute the DC-side current of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param i_ac: AC-side current.
      :type i_ac: complex

      :returns: **i_dc** -- DC-side current.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_dc_voltage()

      
      Measure the DC-bus voltage.

      :returns: DC-bus voltage.
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(L=0.002, C=0.000235, U_g=400, f_g=50)

   Bases: :py:obj:`Inverter`

   
   Frequency converter.

   This extends the Inverter class with models for a strong grid, a
   three-phase diode-bridge rectifier, an LC filter, and a three-phase
   inverter.

   :param L: DC-bus inductance.
   :type L: float
   :param C: DC-bus capacitance.
   :type C: float
   :param U_g: Grid voltage (line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency.
   :type f_g: float















   ..
       !! processed by numpydoc !!
   .. py:method:: grid_voltages(t)

      
      Compute three-phase grid voltages.

      :param t: Time.
      :type t: float

      :returns: **u_g_abc** -- The phase voltages.
      :rtype: ndarray of floats, shape (3,)















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, u_dc, i_L, i_dc)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param u_dc: DC-bus voltage over the capacitor.
      :type u_dc: float
      :param i_L: DC-bus inductor current.
      :type i_L: float
      :param i_dc: Current to the inverter.
      :type i_dc: float

      :returns: Time derivative of the state vector, [du_dc, d_iL]
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotor(n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_s=0.245)

   
   Γ-equivalent model of an induction motor.

   An induction motor is modeled using the Γ-equivalent model [R743146ac54e0-1]_. The model
   is implemented in stator coordinates. The flux linkages are used as state
   variables.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_r: Rotor resistance.
   :type R_r: float
   :param L_ell: Leakage inductance.
   :type L_ell: float
   :param L_s: Stator inductance.
   :type L_s: float

   .. rubric:: Notes

   The Γ model is chosen here since it can be extended with the magnetic
   saturation model in a staightforward manner. If the magnetic saturation is
   omitted, the Γ model is mathematically identical to the inverse-Γ and T
   models [R743146ac54e0-1]_.

   .. rubric:: References

   .. [R743146ac54e0-1] Slemon, "Modelling of induction machines for electric drives," IEEE
      Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Compute the stator and rotor currents.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current.
                * **i_rs** (*complex*) -- Rotor current.















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_ss, psi_rs)

      
      Magnetic model.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex

      :returns: * **i_ss** (*complex*) -- Stator current.
                * **i_rs** (*complex*) -- Rotor current.
                * **tau_M** (*float*) -- Electromagnetic torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_ss, psi_rs, u_ss, w_M)

      
      Compute the state derivatives.

      :param psi_ss: Stator flux linkage.
      :type psi_ss: complex
      :param psi_rs: Rotor flux linkage.
      :type psi_rs: complex
      :param u_ss: Stator voltage.
      :type u_ss: complex
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float

      :returns: * *complex list, length 2* -- Time derivative of the state vector, [dpsi_ss, dpsi_rs]
                * **i_ss** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.

      .. rubric:: Notes

      In addition to the state derivatives, this method also returns the
      output signals (stator current `i_ss` and torque `tau_M`) needed for
      interconnection with other subsystems. This avoids overlapping
      computation in simulation.















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the phase currents at the end of the sampling period.

      :returns: **i_s_abc** -- Phase currents.
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorSaturated(n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_su=0.34, beta=0.84, S=7)

   Bases: :py:obj:`InductionMotor`

   
   Γ-equivalent model of an induction motor model with main-flux saturation.

   This extends the InductionMotor class with a main-flux magnetic saturation
   model [R31fc15c1345a-2]_::

       L_s(psi_ss) = L_su/(1 + (beta*abs(psi_ss)**S)

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_r: Rotor resistance.
   :type R_r: float
   :param L_ell: Leakage inductance.
   :type L_ell: float
   :param L_su: Unsaturated stator inductance.
   :type L_su: float
   :param beta: Positive coefficient.
   :type beta: float
   :param S: Positive coefficient.
   :type S: float

   .. rubric:: References

   .. [R31fc15c1345a-2] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
      induction motor drives," IEEE Trans. Ind. Appl., 2012,
      https://doi.org/10.1109/TIA.2012.2190818















   ..
       !! processed by numpydoc !!
   .. py:method:: currents(psi_ss, psi_rs)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorInvGamma(n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224)

   Bases: :py:obj:`InductionMotor`

   
   Inverse-Γ model of an induction motor.

   This extends the InductionMotor class (based on the Γ model) by providing
   an interface for the inverse-Γ model parameters. Linear magnetics are
   assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param R_R: Rotor resistance.
   :type R_R: float
   :param L_sgm: Leakage inductance.
   :type L_sgm: float
   :param L_M: Magnetizing inductance.
   :type L_M: float















   ..
       !! processed by numpydoc !!

.. py:class:: SynchronousMotor(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545, mech=None)

   
   Synchronous motor model.

   This models a synchronous motor in rotor coordinates. The stator flux
   linkage is the state variable. The default values correspond to a 2.2-kW
   permanent-magnet synchronous motor.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param L_d: d-axis inductance.
   :type L_d: float
   :param L_q: q-axis inductance.
   :type L_q: float
   :param psi_f: PM-flux linkage.
   :type psi_f: float
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Compute the stator current.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex

      :returns: **i_s** -- Stator current.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: magnetic(psi_s)

      
      Magnetic model.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex

      :returns: * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: f(psi_s, u_s, w_M)

      
      Compute the state derivative.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex
      :param u_s: Stator voltage.
      :type u_s: complex
      :param w_M: Rotor angular speed (in mechanical rad/s).
      :type w_M: float

      :returns: * **dpsi_s** (*complex list*) -- Time derivative of the stator flux linkage.
                * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.

      .. rubric:: Notes

      In addition to the state derivative, this method also returns the
      output signals (stator current `i_ss` and torque `tau_M`) needed for
      interconnection with other subsystems. This avoids overlapping
      computation in simulation.















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the phase currents at the end of the sampling period.

      :returns: **i_s_abc** -- Phase currents.
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorSaturated(n_p=2, R_s=0.54, i_f=0, a_d0=17.4, a_q0=52.1, a_dd=373.0, a_qq=658.0, a_dq=1120.0, S=5, T=1, U=1, V=0, mech=None)

   Bases: :py:obj:`SynchronousMotor`

   
   Model of a saturated synchronous motor.

   This extends the SynchronousMotor class with an analytical saturation
   model [R83c7849ec2d6-1]_, [R83c7849ec2d6-2]_. The permanent magnets (PMs) are assumed to be along the
   d-axis. The default values correspond to a 6.7-kW synchronous reluctance
   motor.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param i_f: Constant current corresponding to the magnetomotive force (MMF) of PMs.
               In the magnetically linear case, `i_f = psi_f/L_d`.
   :type i_f: float
   :param a_d0: Nonnegative parameter of the saturation model. In the magnetically
                linear case, `a_d0 = 1/L_d`.
   :type a_d0: float
   :param a_q0: Nonnegative parameter of the saturation model. In the magnetically
                linear case, `a_q0 = 1/L_q`.
   :type a_q0: float
   :param a_dd: Nonnegative constant defining the d-axis self-saturation together with
                `S`. In the magnetically linear case, `a_dd = 0`.
   :type a_dd: float
   :param a_qq: Nonnegative constant defining the q-axis self-saturation together with
                `T`. In the magnetically linear case, `a_qq = 0`.
   :type a_qq: float
   :param a_dq: Nonnegative constant defining the cross-saturation together with `U`
                and `V`. In the magnetically linear case, `a_dq = 0`.
   :type a_dq: float
   :param S: Nonnegative constant defining the d-axis self-saturation.
   :type S: float
   :param T: Nonnegative constant defining the q-axis self-saturation.
   :type T: float
   :param U: Nonnegative constant defining the cross-saturation.
   :type U: float
   :param V: Nonnegative constant defining the cross-saturation.
   :type V: float
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics

   .. rubric:: Notes

   The magnetomotive force (MMF) of the PMs is modeled using constant current
   source `i_f` on the d-axis [R83c7849ec2d6-3]_. Correspondingly, this approach assumes
   that the MMFs of the d-axis current and of the PMs are in series. This
   model cannot capture the desaturation phenomenon of thin iron ribs [R83c7849ec2d6-4]_.
   For such motors, look-up tables can be used.

   .. rubric:: References

   .. [R83c7849ec2d6-1] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi,
      “Sensorless self-commissioning of synchronous reluctance motors at
      standstill without rotor locking, ”IEEE Trans. Ind. Appl., 2017,
      https://doi.org/10.1109/TIA.2016.2644624

   .. [R83c7849ec2d6-2] Awan, Song, Saarakkala, Hinkkanen, "Optimal torque control of
      saturated synchronous motors: plug-and-play method," IEEE Trans. Ind.
      Appl., 2018, https://doi.org/10.1109/TIA.2018.2862410

   .. [R83c7849ec2d6-3] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous
      motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986,
      https://doi.org/10.1109/TIA.1986.4504786

   .. [R83c7849ec2d6-4] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate
      modeling and performance analysis of IPM-PMASR motors," IEEE Trans. Ind.
      Appl., 2009, https://doi.org/10.1109/TIA.2008.2009493















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorSaturatedLUT(n_p=2, R_s=0.2, psi_s_data=None, i_s_data=None, mech=None)

   Bases: :py:obj:`SynchronousMotor`

   
   Look-up-table-based model of a saturated synchronous motor.

   This extends the SynchronousMotor class with a saturation model, where the
   stator current depends on the stator flux linkage. The coordinates assume
   the PMSM convention, i.e., that the PM flux is along the d-axis.
   Unstructured flux map data can be used.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance.
   :type R_s: float
   :param psi_s_data: Stator flux data points for creating the interpolant.
   :type psi_s_data: ndarray of complex
   :param i_s_data: Stator current data values for creating the interpolant.
   :type i_s_data: ndarray of complex
   :param mech: Model of the mechanical subsystem, needed only for the coordinate
                transformation in the measure_currents method.
   :type mech: Mechanics















   ..
       !! processed by numpydoc !!
   .. py:method:: current(psi_s)

      
      Override the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorDrive(motor=None, mech=None, conv=None)

   
   Continuous-time model for an induction motor drive.

   This interconnects the subsystems of an induction motor drive and provides
   an interface to the solver. More complicated systems could be modeled using
   a similar template.

   :param motor: Induction motor model.
   :type motor: InductionMotor | InductionMotorSaturated
   :param mech: Mechanics model.
   :type mech: Mechanics
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Get the initial values.

      :returns: **x0** -- Initial values of the state variables.
      :rtype: complex list, length 4















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Set the initial values.

      :param x0: Initial values of the state variables.
      :type x0: complex ndarray















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Compute the complete state derivative list for the solver.

      :param t: Time.
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
      :type sol: Bunch object















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorDriveDiode(motor=None, mech=None, conv=None)

   Bases: :py:obj:`InductionMotorDrive`

   
   Induction motor drive equipped with a diode bridge.

   This model extends the InductionMotorDrive class with a model for a
   three-phase diode bridge fed from stiff supply voltages. The DC bus is
   modeled as an inductor and a capacitor.

   :param motor: Induction motor model.
   :type motor: InductionMotor | InductionMotorSaturated
   :param mech: Mechanics model.
   :type mech: Mechanics
   :param conv: Frequency converter model.
   :type conv: FrequencyConverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Override the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorDriveTwoMass(motor=None, mech=None, conv=None)

   Bases: :py:obj:`InductionMotorDrive`

   
   Induction motor drive with two-mass mechanics.

   This interconnects the subsystems of an induction motor drive and provides
   an interface to the solver.

   :param motor: Induction motor model.
   :type motor: InductionMotor | InductionMotorSaturated
   :param mech: Mechanics model.
   :type mech: MechanicsTwoMass
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Override the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorDrive(motor=None, mech=None, conv=None)

   
   Continuous-time model for a synchronous motor drive.

   This interconnects the subsystems of a synchronous motor drive and provides
   an interface to the solver. More complicated systems could be modeled using
   a similar template.

   :param motor: Synchronous motor model.
   :type motor: SynchronousMotor
   :param mech: Mechanics model.
   :type mech: Mechanics
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Get the initial values.

      :returns: **x0** -- Initial values of the state variables.
      :rtype: complex list, length 3















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Set the initial values.

      :param x0: Initial values of the state variables.
      :type x0: complex ndarray















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Compute the complete state derivative list for the solver.

      :param t: Time.
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
      :type sol: Bunch object















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Transform the lists to the ndarray format and post-process them.
















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorDriveTwoMass(motor=None, mech=None, conv=None)

   Bases: :py:obj:`SynchronousMotorDrive`

   
   Synchronous motor drive with two-mass mechanics.

   This interconnects the subsystems of a synchronous motor drive and provides
   an interface to the solver.

   :param motor: Synchronous motor model.
   :type motor: SynchronousMotor
   :param mech: Mechanics model.
   :type mech: MechanicsTwoMass
   :param conv: Inverter model.
   :type conv: Inverter















   ..
       !! processed by numpydoc !!
   .. py:method:: get_initial_values()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: set_initial_values(t0, x0)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, x)

      
      Override the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: save(sol)

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!

   .. py:method:: post_process()

      
      Extend the base class.
















      ..
          !! processed by numpydoc !!


.. py:function:: import_syre_data(fname='THOR.mat', add_negative_q_axis=True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format,
   please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which
   the PM flux is along the d axis.

   :param fname: MATLAB file name. The default is 'THOR.mat'.
   :type fname: str, optional
   :param add_negative_q_axis: Adds the negative q-axis data based on the symmetry.
   :type add_negative_q_axis: bool, optional

   :returns: * *Bunch object with the following fields defined*
             * **i_s** (*complex ndarray*) -- Stator current data.
             * **psi_s** (*complex ndarray*) -- Stator flux linkage data.
             * **tau_M** (*ndarray*)

   .. rubric:: Notes

   Some example data files (including THOR.mat) are available in the SyR-e
   repository, licensed under the Apache License, Version 2.0.















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_map(data)

   
   Plot the flux linkage as function of the current.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_vs_current(data)

   
   Plot the flux vs. current characteristics.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:class:: InductionMotorVHzCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   V/Hz control with the stator current feedback.

   :param pars: Control parameters.
   :type pars: InductionMotorVHzCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of an induction motor drive for getting the
                  feedback signals.
      :type mdl: InductionMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!

   .. py:method:: stator_freq(w_s_ref, i_s)

      
      Compute the dynamic stator frequency.

      This computes the dynamic stator frequency reference used in the
      coordinate transformations.















      ..
          !! processed by numpydoc !!

   .. py:method:: voltage_reference(w_s, i_s)

      
      Compute the stator voltage reference.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorVHzCtrlPars

   
   V/Hz control parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: six_step
      :type: bool
      :value: False

      

   .. py:attribute:: psi_s_nom
      :type: float
      :value: 1.04

      

   .. py:attribute:: rate_limit
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.7

      

   .. py:attribute:: R_R
      :type: float
      :value: 2.1

      

   .. py:attribute:: L_sgm
      :type: float
      :value: 0.021

      

   .. py:attribute:: L_M
      :type: float
      :value: 0.224

      

   .. py:attribute:: k_u
      :type: float
      :value: 1

      

   .. py:attribute:: k_w
      :type: float
      :value: 4

      


.. py:class:: InductionMotorVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Vector control for an induction motor drive.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: InductionMotorVectorControlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of an induction motor drive for getting the
                  feedback signals.
      :type mdl: InductionMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorVectorCtrlPars

   
   Vector control parameters for induction motor drives.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: sensorless
      :type: bool
      :value: True

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: alpha_c
      :type: float

      

   .. py:attribute:: alpha_o
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: g
      :value: 0.2

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: psi_R_nom
      :type: float
      :value: 0.9

      

   .. py:attribute:: u_dc_nom
      :type: float
      :value: 540

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.7

      

   .. py:attribute:: R_R
      :type: float
      :value: 2.1

      

   .. py:attribute:: L_sgm
      :type: float
      :value: 0.021

      

   .. py:attribute:: L_M
      :type: float
      :value: 0.224

      

   .. py:attribute:: n_p
      :type: int
      :value: 2

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      


.. py:class:: SynchronousMotorVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Vector control for a synchronous motor drive.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorVectorCtrlPars

   
   Vector control parameters for synchronous motors.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: sensorless
      :type: bool
      :value: True

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: alpha_c
      :type: float

      

   .. py:attribute:: alpha_fw
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: psi_s_min
      :type: float

      

   .. py:attribute:: k_u
      :type: float
      :value: 0.95

      

   .. py:attribute:: w_nom
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.6

      

   .. py:attribute:: L_d
      :type: float
      :value: 0.036

      

   .. py:attribute:: L_q
      :type: float
      :value: 0.051

      

   .. py:attribute:: psi_f
      :type: float
      :value: 0.545

      

   .. py:attribute:: n_p
      :type: int
      :value: 3

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      

   .. py:attribute:: w_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.2

      


.. py:class:: TorqueCharacteristics(pars)

   
   Compute MTPA and MTPV loci based on the motor parameters.

   The magnetic saturation is omitted.

   :param pars: Motor parameters.
   :type pars: data object















   ..
       !! processed by numpydoc !!
   .. py:method:: torque(psi_s)

      
      Compute the torque as a function of the stator flux linkage.

      :param psi_s: Stator flux.
      :type psi_s: complex

      :returns: **tau_M** -- Electromagnetic torque.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: current(psi_s)

      
      Compute the stator current as a function of the stator flux linkage.

      :param psi_s: Stator flux linkage.
      :type psi_s: complex

      :returns: **i_s** -- Stator current.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: flux(i_s)

      
      Compute the stator flux linkage as a function of the current.

      :param i_s: Stator current.
      :type i_s: complex

      :returns: **psi_s** -- Stator flux linkage.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpa(abs_i_s)

      
      Compute the MTPA stator current angle.

      :param abs_i_s: Stator current magnitude.
      :type abs_i_s: float

      :returns: **beta** -- MTPA angle of the stator current vector.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv(abs_psi_s)

      
      Compute the MTPV stator flux angle.

      :param abs_psi_s: Stator flux magnitude.
      :type abs_psi_s: float

      :returns: **delta** -- MTPV angle of the stator flux vector.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_current(abs_i_s)

      
      Compute the MTPV based on the current magnitude.

      This computes the MTPV based on the current magnitude, i.e., the
      intersection of the MTPV current locus and the current limit circle.
      This method is not necessary for computing the control look-up tables.
      It is used here to "cut" the MTPV characteristics at the desired
      current. Alternatively just a large enough maximum flux magnitude could
      be used.

      :param abs_i_s: Stator current magnitude.
      :type abs_i_s: float

      :returns: **i_s** -- MTPV stator current.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpa_locus(i_s_max=1, N=20)

      
      Compute the MTPA locus.

      :param i_s_max: Maximum stator current magnitude at which the locus is computed.
                      The default is 1.
      :type i_s_max: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux.
                * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.
                * **abs_psi_s_vs_tau_M** (*interp1d object*) -- Stator flux magnitude as a function of the torque.
                * **i_sd_vs_tau_M** (*interp1d object*) -- d-axis current as a function of the torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_locus(psi_s_max=1, i_s_max=None, N=20)

      
      Compute the MTPV locus.

      :param psi_s_max: Maximum stator flux magnitude at which the locus is computed. The
                        default is 1.
      :type psi_s_max: float, optional
      :param i_s_max: Maximum stator current magnitude at which the locus is computed.
                      The default is None.
      :type i_s_max: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux.
                * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque as a function of the flux magnitude.















      ..
          !! processed by numpydoc !!

   .. py:method:: current_limit(i_s_max=1, gamma1=np.pi, gamma2=0, N=20)

      
      Compute the current limit.

      :param i_s_max: Current limit. The default is 1.
      :type i_s_max: float, optional
      :param gamma1: Starting angle in radians. The default is 0.
      :type gamma1: float, optional
      :param gamma2: End angle in radians. The defauls in np.pi.
      :type gamma2: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux.
                * **i_s** (*complex*) -- Stator current.
                * **tau_M** (*float*) -- Electromagnetic torque.
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque as a function of the flux magnitude.















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_and_current_limits(i_s_max=1, N=20)

      
      Merge the MTPV and current limits into a single interpolant.

      :param i_s_max: Current limit. The default is 1.
      :type i_s_max: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque as a function of the flux magnitude.
                * **i_sd_vs_tau_M** (*interp1d object*) -- d-axis current as a function of the torque.















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_flux_loci(i_s_max, base, N=20)

      
      Plot the stator flux linkage loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: object
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_current_loci(i_s_max, base, N=20)

      
      Plot the current loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: object
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_torque_current(i_s_max, base, N=20)

      
      Plot torque vs. current characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: object
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_torque_flux(i_s_max, base, N=20)

      
      Plot torque vs. flux magnitude characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: object
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorFluxVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Flux-vector control for synchronous motor drives.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: SynchronousMotoroFluxVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorFluxVectorCtrlPars

   
   Control parameters: flux-vector control for synchronous motor drives.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: sensorless
      :type: bool
      :value: True

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: psi_s_min
      :type: float

      

   .. py:attribute:: psi_s_max
      :type: float

      

   .. py:attribute:: k_u
      :type: float
      :value: 0.9

      

   .. py:attribute:: alpha_psi
      :type: float

      

   .. py:attribute:: alpha_tau
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.6

      

   .. py:attribute:: L_d
      :type: float
      :value: 0.036

      

   .. py:attribute:: L_q
      :type: float
      :value: 0.051

      

   .. py:attribute:: psi_f
      :type: float
      :value: 0.545

      

   .. py:attribute:: n_p
      :type: int
      :value: 3

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      

   .. py:attribute:: w_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.2

      

   .. py:attribute:: g
      :type: float

      


.. py:class:: SynchronousMotorSignalInjectionCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Sensorless control with signal injection for a synchronous motor drive.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorSignalInjectionCtrlPars

   Bases: :py:obj:`motulator.control.sm_vector.SynchronousMotorVectorCtrlPars`

   
   Square-wave signal injection parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: U_inj
      :type: float
      :value: 200

      


.. py:class:: InductionMotorVHzObsCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Observer-based V/Hz control for induction motors.

   :param pars: Control parameters.
   :type pars: InductionMotorObsVHzCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of an induction motor drive for getting the
                  feedback signals.
      :type mdl: InductionMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!

   .. py:method:: state_feedback(i_s, psi_R, w_s)

      
      Compute the stator voltage reference.
















      ..
          !! processed by numpydoc !!


.. py:class:: InductionMotorObsVHzCtrlPars

   
   Control parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: psi_s_nom
      :type: float
      :value: 1.04

      

   .. py:attribute:: rate_limit
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: alpha_f
      :type: float

      

   .. py:attribute:: alpha_psi
      :type: float

      

   .. py:attribute:: k_tau
      :type: float
      :value: 3.0

      

   .. py:attribute:: six_step
      :type: bool
      :value: False

      

   .. py:attribute:: slip_compensation
      :type: bool
      :value: True

      

   .. py:attribute:: alpha_r
      :type: float

      

   .. py:attribute:: alpha_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.7

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.7

      

   .. py:attribute:: R_R
      :type: float
      :value: 2.1

      

   .. py:attribute:: L_sgm
      :type: float
      :value: 0.021

      

   .. py:attribute:: L_M
      :type: float
      :value: 0.224

      

   .. py:attribute:: n_p
      :type: int
      :value: 2

      


.. py:class:: SynchronousMotorVHzObsCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Observer-based V/Hz control for synchronous motors.

   :param pars: Control parameters.
   :type pars: SynchronousMotorVHzObsCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: SynchronousMotorVHzObsCtrlPars

   
   Control parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: psi_s_max
      :type: float

      

   .. py:attribute:: psi_s_min
      :type: float

      

   .. py:attribute:: rate_limit
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: alpha_psi
      :type: float

      

   .. py:attribute:: alpha_tau
      :type: float

      

   .. py:attribute:: alpha_f
      :type: float

      

   .. py:attribute:: k_u
      :type: float
      :value: 1

      

   .. py:attribute:: alpha_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.2

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.6

      

   .. py:attribute:: L_d
      :type: float
      :value: 0.036

      

   .. py:attribute:: L_q
      :type: float
      :value: 0.051

      

   .. py:attribute:: psi_f
      :type: float
      :value: 0.545

      

   .. py:attribute:: n_p
      :type: int
      :value: 3

      


.. py:class:: BaseValues

   
   Base values.

   Base values are computed from the nominal values and the number of pole
   pairs. They can be used, e.g., for scaling the plotted waveforms.















   ..
       !! processed by numpydoc !!
   .. py:attribute:: U_nom
      :type: float

      

   .. py:attribute:: I_nom
      :type: float

      

   .. py:attribute:: f_nom
      :type: float

      

   .. py:attribute:: P_nom
      :type: float

      

   .. py:attribute:: tau_nom
      :type: float

      

   .. py:attribute:: n_p
      :type: int

      

   .. py:method:: __post_init__()

      
      Compute the base values.
















      ..
          !! processed by numpydoc !!


.. py:function:: abc2complex(u)

   
   Transform three-phase quantities to a complex space vector.

   :param u: Phase quantities.
   :type u: array_like, shape (3,)

   :returns: Complex space vector (peak-value scaling).
   :rtype: complex

   .. rubric:: Examples

   >>> from motulator import abc2complex
   >>> y = abc2complex([1, 2, 3])
   >>> y
   (-1-0.5773502691896258j)















   ..
       !! processed by numpydoc !!

.. py:function:: complex2abc(u)

   
   Transform a complex space vector to three-phase quantities.

   :param u: Complex space vector (peak-value scaling).
   :type u: complex

   :returns: Phase quantities.
   :rtype: ndarray, shape (3,)

   .. rubric:: Examples

   >>> from motulator import complex2abc
   >>> y = complex2abc(1-.5j)
   >>> y
   array([ 1.       , -0.9330127, -0.0669873])















   ..
       !! processed by numpydoc !!

.. py:class:: Sequence(times, values, periodic=False)

   
   Sequence generator.

   The time array must be increasing. The output values are interpolated
   between the data points.

   :param times: Time values.
   :type times: ndarray
   :param values: Output values.
   :type values: ndarray
   :param periodic: Enables periodicity. The default is False.
   :type periodic: bool, optional















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(t)

      
      Interpolate the output.

      :param t: Time.
      :type t: float

      :returns: Interpolated output.
      :rtype: float or complex















      ..
          !! processed by numpydoc !!


.. py:class:: Step(step_time, step_value, initial_value=0)

   
   Step function.
















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(t)

      
      Step function.

      :param t: Time.
      :type t: float

      :returns: Step output.
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:function:: plot(sim, t_span=None, base=None)

   
   Plot example figures.

   Plots figures in per-unit values, if the base values are given. Otherwise
   SI units are used.

   :param sim: Should contain the simulated data.
   :type sim: Simulation object
   :param t_span: Time span. The default is (0, sim.ctrl.t[-1]).
   :type t_span: 2-tuple, optional
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_extra(sim, t_span=(1.1, 1.125), base=None)

   
   Plot extra waveforms for a motor drive with a diode bridge.

   :param sim: Should contain the simulated data.
   :type sim: Simulation object
   :param t_span: Time span. The default is (1.1, 1.125).
   :type t_span: 2-tuple, optional
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

