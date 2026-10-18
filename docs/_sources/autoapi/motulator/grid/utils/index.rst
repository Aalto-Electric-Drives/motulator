motulator.grid.utils
====================

.. py:module:: motulator.grid.utils

.. autoapi-nested-parse::

   
   Utility functions for grid converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.utils.BaseValues
   motulator.grid.utils.IdentificationCfg
   motulator.grid.utils.IdentificationResults
   motulator.grid.utils.NominalValues
   motulator.grid.utils.SequenceGenerator
   motulator.grid.utils.Step


Functions
---------

.. autoapisummary::

   motulator.grid.utils.plot_control_signals
   motulator.grid.utils.plot_grid_waveforms
   motulator.grid.utils.plot_identification
   motulator.grid.utils.plot_vector_identification
   motulator.grid.utils.plot_voltage_vector
   motulator.grid.utils.run_identification


Package Contents
----------------

.. py:class:: BaseValues

   
   Base values.

   :param u: Voltage (V, peak, line-neutral).
   :type u: float
   :param i: Current (A, peak).
   :type i: float
   :param w: Angular frequency (rad/s).
   :type w: float
   :param psi: Flux linkage (Vs).
   :type psi: float
   :param p: Power (W).
   :type p: float
   :param Z: Impedance (Ω).
   :type Z: float
   :param L: Inductance (H).
   :type L: float
   :param C: Capacitance (F).
   :type C: float
   :param tau: Torque (Nm), defaults to 0.
   :type tau: float, optional
   :param n_p: Number of pole pairs, defaults to 0.
   :type n_p: int, optional
   :param w_M: Mechanical angular frequency (rad/s), defaults to 0.
   :type w_M: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: from_nominal(nom, n_p = None)
      :classmethod:


      
      Compute base values from nominal values.

      :param nom:
                  Nominal values containing the following fields:
                      U : float
                          Voltage (V, rms, line-line).
                      I : float
                          Current (A, rms).
                      f : float
                          Frequency (Hz).
      :type nom: NominalValues
      :param n_p: Number of pole pairs, defaults to None.
      :type n_p: int | None, optional

      :returns: Base values.
      :rtype: BaseValues

      .. rubric:: Notes

      Notice that the nominal torque is larger than the base torque due to the power
      factor and efficiency being less than unity.















      ..
          !! processed by numpydoc !!


   .. py:method:: unity()
      :classmethod:


      
      Create base values with all values set to 1.
















      ..
          !! processed by numpydoc !!


.. py:class:: IdentificationCfg

   
   Configuration parameters for converter output admittance identification.

   :param abs_u_e: Magnitude of the voltage excitation (V) at the first frequency.
   :type abs_u_e: float
   :param freqs: Excitation frequencies (Hz, in dq-coordinates), defaults to 100 logarithmically
                 spaced frequencies from 1 Hz to 10 kHz.
   :type freqs: ndarray, optional
   :param amplitude_multiplier: Excitation magnitude at the last frequency relative to the first one, defaults
                                to 5. The magnitude changes linearly over the frequency array. Use 1 for a
                                constant magnitude.
   :type amplitude_multiplier: float, optional
   :param t0: Stop time for initial simulating to the operating point (s), defaults to 1.0.
              Should be set large enough to reach steady-state.
   :type t0: float, optional
   :param t1: Additional simulation time for reaching sinusoidal steady-state during signal
              injection (s), defaults to 0.05.
   :type t1: float, optional
   :param N_eval: Number of evenly spaced data points the solver should return for each
                  integration interval, i.e., for each sampling period or, if carrier comparison
                  is used, for each switching interval, defaults to 10.
   :type N_eval: int, optional
   :param n_periods_excitation: Number of excitation signal periods used in calculating the DFT, defaults to 4.
                                At least 3 are needed if `use_window` is True.
   :type n_periods_excitation: int, optional
   :param n_periods_init: Number of fundamental periods to include for calculating operating-point
                          vectors, defaults to 1.
   :type n_periods_init: int, optional
   :param multiprocess: If True, the frequencies are simulated in parallel using all available CPU
                        cores, defaults to True. The model and the control system are copied to the
                        worker processes, which can require pickling them. Lambda functions and nested
                        functions (e.g., as references) cannot be pickled, but module-level functions
                        can.
   :type multiprocess: bool, optional
   :param use_window: Whether to use window function for calculating DFT, defaults to True.
   :type use_window: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: IdentificationResults

   
   Container for identification results.

   Contains fields for excitation signal frequency `f_e`, elements of the output
   admittance matrix `[Y_dd, Y_qd; Y_dq, Y_qq]`, and the following operating-point
   vectors (in synchronous coordinates aligned with the PCC voltage): grid current
   `i_g0`, grid voltage `e_g0`, PCC voltage `u_g0` and converter voltage `u_c0`. The
   results can be saved, e.g., by ``scipy.io.savemat(path, dataclasses.asdict(res))``.















   ..
       !! processed by numpydoc !!

.. py:class:: NominalValues

   
   Nominal values.

   :param U: Voltage (V, rms, line-line).
   :type U: float
   :param I: Current (A, rms).
   :type I: float
   :param f: Frequency (Hz).
   :type f: float
   :param P: Power (W).
   :type P: float
   :param tau: Torque (Nm), defaults to 0.
   :type tau: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: SequenceGenerator(times, values, periodic = False)

   
   Sequence generator.

   The time array must be increasing. The output values are interpolated between the
   data points.

   :param times: Time values.
   :type times: ndarray
   :param values: Output values.
   :type values: ndarray
   :param periodic: Enables periodicity, defaults to False.
   :type periodic: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Step(step_time, step_value, initial_value = 0.0)

   
   Step function.

   :param step_time: Time of the step.
   :type step_time: float
   :param step_value: Value of the step.
   :type step_value: float
   :param initial_value: Initial value, defaults to 0.
   :type initial_value: float, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_control_signals(res, base = None, t_lims = None, t_ticks = None, y_lims = None, y_ticks = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot control signals and converter voltages.

   :param res: Simulation results.
   :type res: ~motulator.common.model.SimulationResults
   :param base: Base values for scaling the waveforms. If not given, the waveforms are plotted
                in SI units.
   :type base: BaseValues, optional
   :param t_lims: Time axis limits. If None, uses full time range.
   :type t_lims: tuple[float, float], optional
   :param t_ticks: Time axis tick locations.
   :type t_ticks: ArrayLike, optional
   :param y_lims: y-axis limits for each subplot.
   :type y_lims: list[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: list[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: plot_grid_waveforms(res, base = None, t_lims = None, t_ticks = None, y_lims = None, y_ticks = None, latex = False, plot_pcc_voltage = True, save_path = None, **savefig_kwargs)

   
   Plot grid waveforms and phase angles.

   The phase angles are the grid voltage angle and the angle `fbk.theta_c` of the
   coordinates of the post-processed controller signals. For the observer-based
   grid-forming controllers, these coordinates are aligned with the estimated
   quasi-static converter voltage `v_c` (or its positive-sequence component).

   :param res: Simulation results.
   :type res: ~motulator.common.model.SimulationResults
   :param base: Base values for scaling the waveforms. If not given, the waveforms are plotted
                in SI units.
   :type base: BaseValues, optional
   :param t_lims: Time axis limits. If None, uses full time range.
   :type t_lims: tuple[float, float], optional
   :param t_ticks: Time axis tick locations.
   :type t_ticks: ArrayLike, optional
   :param y_lims: y-axis limits for each subplot.
   :type y_lims: list[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: list[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
   :type latex: bool, optional
   :param plot_pcc_voltage: If True, plot the phase voltage waveforms at the point of common coupling (PCC).
                            Otherwise, plot the grid voltage waveforms, defaults to True.
   :type plot_pcc_voltage: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: plot_identification(res, plot_style = 're_im', plot_passivity_index = True, latex = False)

   
   Plot the identification results

   :param res: Should contain the results from the identification.
   :type res: IdentificationResults
   :param plot_style: Style for plotting of identification results, defaults to "re_im". Options are:
                      - "polar": plot magnitude and phase
                      - "re_im": plot real and imaginary parts
   :type plot_style: Literal["polar", "re_im"], optional
   :param plot_passivity_index: Plot input feedforward passivity index calculated from the identification
                                results, defaults to True.
   :type plot_passivity_index: bool, optional
   :param latex: Use latex for plots, defaults to False.
   :type latex: bool, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_vector_identification(res, base, latex = False)

   
   Plot the converter voltage, PCC voltage, grid voltage, and grid current vectors in
   the operating point.

   :param res: Should contain the results from the identification.
   :type res: IdentificationResults
   :param base: Base values for per-unit conversion.
   :type base: BaseValues
   :param latex: Use latex for plots, defaults to False.
   :type latex: bool, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_voltage_vector(res, base = None, save_path = None, **savefig_kwargs)

   
   Plot locus of the grid voltage vector.

   :param res: Simulation results.
   :type res: ~motulator.common.model.SimulationResults
   :param base: Base values for scaling the waveforms. If not given, the waveforms are plotted
                in SI units.
   :type base: BaseValues, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: run_identification(cfg, mdl, ctrl)

   
   Run the identification.

   The system is first simulated to the operating point. Then, the output admittance
   of the converter is identified at the PCC: the PCC voltage is imposed by a stiff
   voltage source, whose magnitude and frequency equal those of the operating point,
   and small-signal voltage excitations are injected in the d- and q-axes. The grid
   impedance of `mdl` as well as any time variation and negative-sequence component of
   its source only affect the operating point. A DC bus of `mdl` is included in the
   identification.

   :param cfg: Identification configuration.
   :type cfg: IdentificationCfg
   :param mdl: Continuous-time system model.
   :type mdl: GridConverterSystem
   :param ctrl: Discrete-time control system.
   :type ctrl: GridConverterControlSystem

   :returns: Admittance matrix elements and the operating point.
   :rtype: IdentificationResults

   .. rubric:: Notes

   With carrier-comparison PWM, background switching harmonics can bias the identified
   admittance at the excitation frequency. Results near switching harmonics and their
   sidebands may depend strongly on the excitation amplitude and its phase relative to
   the carrier, even when the numerical solution has converged.















   ..
       !! processed by numpydoc !!

