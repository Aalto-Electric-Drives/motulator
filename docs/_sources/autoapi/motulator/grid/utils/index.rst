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

   :param abs_u_e: Magnitude of the voltage excitation (V).
   :type abs_u_e: float
   :param f_start: Starting frequency of the voltage excitation (Hz, in dq-coordinates), defaults
                   to 1.
   :type f_start: float, optional
   :param f_stop: End frequency of the voltage excitation (Hz, in dq-coordinates), defaults to
                  10e3.
   :type f_stop: float, optional
   :param n_freqs: Number of frequencies for measurement, defaults to 100.
   :type n_freqs: int, optional
   :param spacing: The spacing used for creating the array of measurement frequencies, defaults to
                   "log". Valid options are:
                   - "log": logarithmic spacing
                   - "lin": linear spacing
   :type spacing: Literal["log", "lin"], optional
   :param manual_freqs: Manually specified array of frequencies (Hz) to measure admittance at. If set to
                        None, f_start, f_stop and n_freqs parameters are used to create the array of
                        frequencies. Defaults to None.
   :type manual_freqs: ndarray | None, optional
   :param t0: Stop time for initial simulating to the operating point (s), defaults to 1.0.
              Should be set large enough to reach steady-state.
   :type t0: float, optional
   :param t1: Additional simulation time for reaching sinusoidal steady-state during signal
              injection (s), defaults to 0.05.
   :type t1: float, optional
   :param T_s: Sampling period of the control system (s), defaults to 125e-6.
   :type T_s: float, optional
   :param N_eval: Number of evenly spaced data points the solver should return for each controller
                  sampling period, defaults to 10.
   :type N_eval: int, optional
   :param n_periods_excitation: Number of excitation signal periods used in calculating the DFT, defaults to 4.
   :type n_periods_excitation: int, optional
   :param n_periods_init: Number of fundamental periods to include for calculating operating-point
                          vectors, defaults to 1.
   :type n_periods_init: int, optional
   :param multiprocess: If set to True, multiprocessing.Pool() is used to run the identification using
                        all available CPU cores in the system. Defaults to True.
   :type multiprocess: bool, optional
   :param filename: If given, the identification result is saved in
                    ``data/{date}_{time}_{filename}`` (relative to the project root directory),
                    defaults to None. The file format is set by the `filetype` parameter.
   :type filename: str | None, optional
   :param filetype: The filetype for saving identification results, defaults to "csv". Valid options
                    are:
                    - "csv": save results in .csv-format
                    - "mat": save results in MATLAB .mat-format
   :type filetype: Literal["csv", "mat"], optional
   :param delay: Number of samples for modeling the computational delay, defaults to 1.
   :type delay: int, optional
   :param use_window: Whether to use window function for calculating DFT, defaults to True.
   :type use_window: bool, optional
   :param variable_amplitude: Whether to increase the excitation signal amplitude with the frequency, defaults
                              to True.
   :type variable_amplitude: bool, optional
   :param amplitude_multiplier: Gain value to set how much the excitation signal amplitude is increased at the
                                highest frequency compared to the lowest if `variable_amplitude` is set to True.
                                Defaults to 5.0.
   :type amplitude_multiplier: float, optional















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
   :type res: SimulationResults
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

   :param res: Simulation results.
   :type res: SimulationResults
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
   :type res: SimulationResults
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

   :param cfg: Dataclass object containing the identification configuration.
   :type cfg: IdentificationCfg
   :param mdl: Continuous-time system model object.
   :type mdl: GridConverterSystem
   :param ctrl: Discrete-time control system object.
   :type ctrl: GridConverterControlSystem

   :returns: **res** -- Dataclass object containing the results from the identification, along with
             information about the operating point.
   :rtype: IdentificationResults















   ..
       !! processed by numpydoc !!

