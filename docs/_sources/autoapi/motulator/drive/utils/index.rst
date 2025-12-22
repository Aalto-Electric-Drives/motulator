motulator.drive.utils
=====================

.. py:module:: motulator.drive.utils

.. autoapi-nested-parse::

   
   Utility functions for machine drives.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.utils.BaseValues
   motulator.drive.utils.ControlLoci
   motulator.drive.utils.MachineCharacteristics
   motulator.drive.utils.MagneticModel
   motulator.drive.utils.NominalValues
   motulator.drive.utils.SaturationModelBase
   motulator.drive.utils.SaturationModelPMSyRM
   motulator.drive.utils.SaturationModelSyRM
   motulator.drive.utils.SequenceGenerator
   motulator.drive.utils.Step


Functions
---------

.. autoapisummary::

   motulator.drive.utils.import_syre_data
   motulator.drive.utils.plot
   motulator.drive.utils.plot_dc_bus_waveforms
   motulator.drive.utils.plot_flux_vs_current
   motulator.drive.utils.plot_map
   motulator.drive.utils.plot_stator_waveforms


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


.. py:class:: ControlLoci(par)

   
   Compute MTPA and MTPV loci based on the machine parameters.

   This class computes optimal control loci for synchronous machines, including the
   maximum-torque-per-ampere (MTPA), maximum-torque-per-volt (MTPV), and current limit
   loci [#Mor1994]_. The magnetic saturation is taken into account. The methods can be
   used to precompute lookup tables for control and to analyze the machine
   characteristics.

   :param par: Machine model parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars

   .. rubric:: Notes

   The MTPA and MTPV conditions are expressed in terms of the auxiliary flux and the
   auxiliary current, respectively [#Var2022]_, allowing a compact representation of
   the conditions. Notice that we define these auxiliary vectors 90 degrees rotated as
   compared to [#Var2022]_, but otherwise the concepts are equivalent.

   .. rubric:: References

   .. [#Mor1994] Morimoto, Sanada, Takeda, "Wide-speed operation of interior permanent
      magnet synchronous motors with high-performance current regulator," IEEE Trans.
      Ind. Appl., https://doi.org/10.1109/28.297908

   .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
      synchronous motor drives: Accurate decoupled control with online adaptive maximum
      torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
      Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_const_current_locus(i_s_max, gamma_range = (np.pi, 0.5 * np.pi), num = 16)

      
      Compute the constant current locus.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param gamma_range: Range of the current angle (electrical rad), defaults to (pi, pi/2).
      :type gamma_range: tuple, optional
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: Constant current locus data.
      :rtype: CurrentLimitLocus















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpa_current_angle(i_s_abs)

      
      MTPA current angle (rad) at given current magnitude (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpa_locus(i_s_max, num = 16)

      
      Compute the MTPA locus.

      :param i_s_max: Maximum current magnitude (A) at which the locus is computed.
      :type i_s_max: float
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: MTPA locus data.
      :rtype: MTPALocus















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpv_current_angle(i_s_abs)

      
      MTPV current angle (rad) at given current magnitude (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpv_locus(i_s_max, num = 16)

      
      Compute the MTPV locus.

      :param i_s_max: Maximum current (A) at which the locus is computed.
      :type i_s_max: float
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: MTPV locus data.
      :rtype: MTPVLocus















      ..
          !! processed by numpydoc !!


   .. py:method:: solve_current_for_mtpv_torque(tau_M, i_s_abs0)

      
      Solve for the current yielding the given MTPV torque.

      :param tau_M: Target torque (Nm).
      :type tau_M: float
      :param i_s_abs0: Initial guess for the current magnitude (A).
      :type i_s_abs0: float

      :returns: Stator current magnitude (A) that yields the target torque.
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: MachineCharacteristics(par)

   
   Analyze and visualize control loci for synchronous machines.

   This class provides a unified interface for plotting different characteristics of
   synchronous machines directly from machine parameters.

   :param par: Machine parameters.
   :type par: SynchronousMachinePars | SaturatedSynchronousMachinePars















   ..
       !! processed by numpydoc !!

   .. py:method:: plot_current_loci(i_s_vals, base = None, num = 16, latex = False, save_path = None, **savefig_kwargs)

      
      Plot the current loci.

      :param i_s_vals: Current magnitudes (A) at which the loci are evaluated. If `base` is given,
                       the values are interpreted as per-unit values.
      :type i_s_vals: float or list
      :param base: Base values for scaling the loci.
      :type base: BaseValues, optional
      :param num: Amount of points to be evaluated, defaults to 16.
      :type num: int, optional
      :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
      :type latex: bool, optional
      :param save_path: If provided, save the figure to this path.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_current_vs_torque(i_s_vals, base = None, num = 16, latex = False, save_path = None, **savefig_kwargs)

      
      Plot current vs. torque characteristics.

      :param i_s_vals: Current magnitudes (A) at which the loci are evaluated. If `base` is given,
                       the values are interpreted as per-unit values.
      :type i_s_vals: float or list
      :param base: Base values for scaling the loci.
      :type base: BaseValues, optional
      :param num: Amount of points to be evaluated, defaults to 16.
      :type num: int, optional
      :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
      :type latex: bool, optional
      :param save_path: If provided, save the figure to this path.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_loci(i_s_vals, base = None, num = 16, latex = False, save_path = None, **savefig_kwargs)

      
      Plot the flux linkage loci.

      :param i_s_vals: Current magnitudes (A) at which the loci are evaluated. If `base` is given,
                       the values are interpreted as per-unit values.
      :type i_s_vals: float or list
      :param base: Base values for scaling the loci.
      :type base: BaseValues, optional
      :param num: Amount of points to be evaluated, defaults to 16.
      :type num: int, optional
      :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
      :type latex: bool, optional
      :param save_path: If provided, save the figure to this path.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_vs_torque(i_s_vals, base = None, num = 16, latex = False, save_path = None, **savefig_kwargs)

      
      Plot flux magnitude vs. torque characteristics.

      :param i_s_vals: Current magnitudes (A) at which the loci are evaluated. If `base` is given,
                       the values are interpreted as per-unit values.
      :type i_s_vals: float or list
      :param base: Base values for scaling the loci.
      :type base: BaseValues, optional
      :param num: Amount of points to be evaluated, defaults to 16.
      :type num: int, optional
      :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
      :type latex: bool, optional
      :param save_path: If provided, save the figure to this path.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


.. py:class:: MagneticModel

   
   Store and manipulate flux linkage or current maps for synchronous machines.

   .. attribute:: i_s_dq

      Complex array of stator current (A).

      :type: np.ndarray

   .. attribute:: psi_s_dq

      Complex array of stator flux linkage (Vs).

      :type: np.ndarray

   .. attribute:: lookup_fcn

      Linear interpolation function that evaluates the map at arbitrary points. Takes
      complex inputs (d + j*q) and returns interpolated output values. For flux maps,
      maps i_s_dq → psi_s_dq; for current maps, maps psi_s_dq → i_s_dq. The function
      extrapolates outside the map range.

      :type: Callable[[complex | np.ndarray], complex | np.ndarray], optional

   .. attribute:: tau_M

      Array of electromagnetic torque (Nm).

      :type: np.ndarray, optional

   .. attribute:: type

      Type of the map, defaults to "flux_map".

      :type: Literal["current_map", "flux_map"], optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_interpolated_model(d_range = None, q_range = None, num = None, invert = False)

      
      Interpolate or invert this magnetic model onto a regular grid.

      :param d_range: Range for the d-axis. If None, the range is determined from the data,
                      defaults to None.
      :type d_range: np.ndarray | None, optional
      :param q_range: Range for the q-axis. If None, the range is determined from the data,
                      defaults to None.
      :type q_range: np.ndarray | None, optional
      :param num: Number of points in each axis. If None, uses the maximum dimension from the
                  original map to preserve resolution, defaults to None.
      :type num: int, optional
      :param invert: Invert the map (swap input and output), defaults to False.
      :type invert: bool, optional

      :returns: Interpolated magnetic model.
      :rtype: MagneticModel















      ..
          !! processed by numpydoc !!


   .. py:method:: get_input_output()

      
      Get input and output arrays based on map type.
















      ..
          !! processed by numpydoc !!


   .. py:method:: invert(d_range = None, q_range = None, num = None)

      
      Invert the map (swap input and output).

      :param d_range: Range for the d-axis. If None, the range is determined from the data,
                      defaults to None.
      :type d_range: np.ndarray | None, optional
      :param q_range: Range for the q-axis. If None, the range is determined from the data,
                      defaults to None.
      :type q_range: np.ndarray | None, optional
      :param num: Number of points in each axis. If None, uses the maximum dimension from the
                  original map to preserve resolution, defaults to None.
      :type num: int, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: is_current_map()

      
      Return True if this is a current map (psi_s → i_s).
















      ..
          !! processed by numpydoc !!


   .. py:method:: is_flux_map()

      
      Return True if this is a flux map (i_s → psi_s).
















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

.. py:class:: SaturationModelBase(current_map)

   
   Base class for analytical saturation models.

   This class implements a callable interface that maps flux linkage to current,
   matching the interface of MagneticModel.lookup_fcn for current maps. It can be used
   directly as a lookup function for a MagneticModel.















   ..
       !! processed by numpydoc !!

   .. py:method:: as_magnetic_model(d_range, q_range, n_p = None)

      
      Create a magnetic model that uses this analytical model.

      :param d_range: Range for the d-axis flux linkage (Vs).
      :type d_range: ndarray
      :param q_range: Range for the q-axis flux linkage (Vs).
      :type q_range: ndarray
      :param n_p: Number of pole pairs. If provided, the torque is included.
      :type n_p: int, optional

      :returns: Current map that uses this saturation model as its lookup_fcn.
      :rtype: MagneticModel















      ..
          !! processed by numpydoc !!


.. py:class:: SaturationModelPMSyRM(current_map)

   Bases: :py:obj:`SaturationModelSyRM`


   
   Saturation model for PM synchronous reluctance machines.

   This model takes into account the bridge saturation in addition to the self- and
   cross-saturation effects of the d- and q-axis [#Lel2024]_. The bridge saturation
   model is based on a nonlinear reluctance element in parallel with the Norton-
   equivalent PM model.

   .. attribute:: psi_n

      PM flux linkage (Vs).

      :type: float

   .. attribute:: a_b

      Coefficient for bridge inverse inductance.

      :type: float

   .. attribute:: a_bp

      Coefficient for bridge saturation.

      :type: float

   .. attribute:: W

      Exponent for bridge saturation.

      :type: float

   .. attribute:: k_q

      Cross-coupling factor for bridge flux.

      :type: float

   .. rubric:: References

   .. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a
      simplified equivalent magnetic circuit for permanent magnet machines," Proc.
      ICEM, 2024, https://doi.org/10.1109/ICEM60801.2024.10700403















   ..
       !! processed by numpydoc !!

.. py:class:: SaturationModelSyRM(current_map)

   Bases: :py:obj:`SaturationModelBase`


   
   Saturation model for synchronous reluctance machines.

   This model takes into account the self- and cross-saturation effects of the d- and
   q-axis [#Hin2017]_.

   .. attribute:: a_d0

      Offset coefficient for d-axis inverse inductance.

      :type: float

   .. attribute:: a_dd

      Self-saturation coefficient for d-axis.

      :type: float

   .. attribute:: a_q0

      Offset coefficient for q-axis inverse inductance.

      :type: float

   .. attribute:: a_qq

      Self-saturation coefficient for q-axis.

      :type: float

   .. attribute:: a_dq

      Cross-saturation coefficient.

      :type: float

   .. attribute:: S

      Exponent for d-axis self-saturation.

      :type: float

   .. attribute:: T

      Exponent for q-axis self-saturation.

      :type: float

   .. attribute:: U

      First exponent for cross-saturation.

      :type: float

   .. attribute:: V

      Second exponent for cross-saturation.

      :type: float

   .. rubric:: References

   .. [#Hin2017] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi, "Sensorless
      self-commissioning of synchronous reluctance motors at standstill without rotor
      locking," IEEE Trans. Ind. Appl., 2017, https://doi.org/10.1109/TIA.2016.2644624















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

.. py:function:: import_syre_data(fname, add_negative_q_axis = True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format, please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which the PM
   flux is along the d axis.

   :param fname: MATLAB file name.
   :type fname: str
   :param add_negative_q_axis: Adds the negative q-axis data based on the symmetry, defaults to True.
   :type add_negative_q_axis: bool, optional

   :returns: Magnetic model data.
   :rtype: MagneticModel

   .. rubric:: Notes

   Some example data files (including THOR.mat) are available in the SyR-e repository,
   licensed under the Apache License, Version 2.0.















   ..
       !! processed by numpydoc !!

.. py:function:: plot(res, base = None, subplots = None, t_lims = None, t_ticks = None, y_lims = None, y_ticks = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot example figures with selectable subplots.

   :param res: Simulation results.
   :type res: SimulationResults
   :param base: Base values for scaling the waveforms. If not given, the waveforms are plotted
                in SI units.
   :type base: BaseValues, optional
   :param subplots: List of subplot names to include. Valid names: "speed", "torque", "current",
                    "voltage", "flux". Defaults to all.
   :type subplots: list[str], optional
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

.. py:function:: plot_dc_bus_waveforms(res, base = None, t_lims = None, t_ticks = None, y_lims = None, y_ticks = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot DC bus and grid-side waveforms.

   :param res: Should contain the simulated data.
   :type res: SimulationResults
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional
   :param t_lims: Time axis limits. If None, uses full time range.
   :type t_lims: tuple[float, float], optional
   :param t_ticks: Time axis tick locations.
   :type t_ticks: ArrayLike, optional
   :param y_lims: y-axis limits for each subplot.
   :type y_lims: list[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: list[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_vs_current(data, base = None, x_lims = None, x_ticks = None, y_lims = None, y_ticks = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot the flux vs. current characteristics.

   :param data: Flux map data. The current array should be a rectilinear grid.
   :type data: MagneticModel
   :param base: Base values for scaling the maps. If not given, the maps are plotted
                in SI units.
   :type base: BaseValues, optional
   :param x_lims: x-axis limits. If None, uses automatic scaling.
   :type x_lims: tuple[float, float], optional
   :param x_ticks: x-axis tick locations.
   :type x_ticks: ArrayLike, optional
   :param y_lims: y-axis limits. If None, uses automatic scaling.
   :type y_lims: tuple[float, float], optional
   :param y_ticks: y-axis tick locations.
   :type y_ticks: ArrayLike, optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: plot_map(data, component, base = None, lims = None, ticks = None, raw_data = None, axlim_clip = True, latex = False, save_path = None, **savefig_kwargs)

   
   Plot component (d or q) of flux linkage or current maps.

   :param data: Data containing the flux and current information.
   :type data: MagneticModel
   :param component: Which component to plot: "d" for d-axis, "q" for q-axis.
   :type component: {"d", "q"}
   :param base: Base values for scaling the maps. If not given, the maps are plotted
                in SI units.
   :type base: BaseValues, optional
   :param lims: Axis limits. Keys should be 'x', 'y', 'z'.
   :type lims: dict[str, tuple[float, float]], optional
   :param ticks: Axis tick locations. Keys should be 'x', 'y', 'z'.
   :type ticks: dict[str, ArrayLike], optional
   :param raw_data: Raw data for comparison (shown as scatter points).
   :type raw_data: MagneticModel, optional
   :param axlim_clip: Whether to clip the axes limits to the data limits, defaults to True.
   :type axlim_clip: bool, optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

.. py:function:: plot_stator_waveforms(res, base = None, t_lims = None, t_ticks = None, y_lims = None, y_ticks = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot stator voltage and current waveforms.

   :param res: Should contain the simulated data.
   :type res: SimulationResults
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional
   :param t_lims: Time axis limits. If None, uses full time range.
   :type t_lims: tuple[float, float], optional
   :param t_ticks: Time axis tick locations.
   :type t_ticks: ArrayLike, optional
   :param y_lims: y-axis limits for each subplot.
   :type y_lims: list[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: list[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels, requires a working LaTeX installation.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

