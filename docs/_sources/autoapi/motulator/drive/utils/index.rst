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
   motulator.drive.utils.Locus
   motulator.drive.utils.MachineCharacteristics
   motulator.drive.utils.MagneticModel
   motulator.drive.utils.NominalValues
   motulator.drive.utils.SaturationModelBase
   motulator.drive.utils.SaturationModelPMSyRM
   motulator.drive.utils.SaturationModelSyRM
   motulator.drive.utils.SequenceGenerator


Functions
---------

.. autoapisummary::

   motulator.drive.utils.import_syre_data
   motulator.drive.utils.plot
   motulator.drive.utils.plot_flux_vs_current
   motulator.drive.utils.plot_map
   motulator.drive.utils.plot_stator_waveforms


Package Contents
----------------

.. py:class:: BaseValues

   
   Base values.

   :param u: Voltage (V, peak, line-to-neutral).
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

      :param nom: Nominal values.
      :type nom: NominalValues
      :param n_p: Number of pole pairs. Defaults to None, in which case the mechanical base
                  values are not computed.
      :type n_p: int, optional

      :returns: Base values.
      :rtype: BaseValues

      .. rubric:: Notes

      The nominal torque is smaller than the base torque, since the base power equals
      the nominal apparent power and the power factor and efficiency are less than
      unity.















      ..
          !! processed by numpydoc !!


   .. py:method:: unity()
      :classmethod:


      
      Create base values with all values set to 1.
















      ..
          !! processed by numpydoc !!


.. py:class:: ControlLoci(par)

   
   Compute the control loci of a synchronous machine.

   The class computes the maximum-torque-per-ampere (MTPA), maximum-torque-per-volt
   (MTPV), and constant-current loci [#Mor1994]_, taking the magnetic saturation into
   account. The loci can be used to precompute lookup tables for control and to analyze
   the machine characteristics.

   :param par: Machine model parameters.
   :type par: ~motulator.drive.control.sm.BaseSynchronousMachineControlPars

   .. rubric:: Notes

   The MTPA and MTPV conditions are expressed in terms of the auxiliary flux and the
   auxiliary current, respectively [#Var2022]_, allowing a compact representation of
   the conditions. The auxiliary vectors are rotated by 90° compared to [#Var2022]_.

   The d axis is assumed to be aligned with the PM flux, in which case `L_q >= L_d`,
   or, in machines without PMs, with the maximum inductance, i.e., `L_d > L_q`.

   .. rubric:: References

   .. [#Mor1994] Morimoto, Sanada, Takeda, "Wide-speed operation of interior permanent
      magnet synchronous motors with high-performance current regulator," IEEE Trans.
      Ind. Appl., 1994, https://doi.org/10.1109/28.297908

   .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
      synchronous motor drives: Accurate decoupled control with online adaptive maximum
      torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
      Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_const_current_locus(i_s_max, gamma_range = None, num = NUM)

      
      Compute the constant-current locus.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param gamma_range: Range of the current angle (electrical rad), defaults to (pi, pi/2) for PM
                          machines and (pi/2, 0) for machines without PMs. If the first element is NaN
                          (no MTPV), the range starts from pi or pi/2, respectively.
      :type gamma_range: tuple, optional
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: Constant-current locus.
      :rtype: Locus















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpa_current_angle(i_s_abs)

      
      MTPA current angle (rad) at given current magnitude (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpa_locus(i_s_max, num = NUM)

      
      Compute the MTPA locus.

      :param i_s_max: Maximum current magnitude (A), up to which the locus is computed.
      :type i_s_max: float
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: MTPA locus.
      :rtype: Locus















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpv_current(i_s_abs)

      
      MTPV current at given current magnitude.

      :param i_s_abs: Current magnitude (A).
      :type i_s_abs: float

      :returns: MTPV current (A). If no MTPV exists, returns np.nan.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpv_flux_angle(psi_s_abs)

      
      MTPV flux angle (rad) and current (A) at given flux linkage magnitude (Vs).

      NaN values are returned if no MTPV exists in the range of the flux angle.















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_mtpv_locus(psi_s_max, num = NUM)

      
      Compute the MTPV locus.

      :param psi_s_max: Maximum flux linkage magnitude (Vs), up to which the locus is computed.
      :type psi_s_max: float
      :param num: Number of points, defaults to 16.
      :type num: int, optional

      :returns: MTPV locus.
      :rtype: Locus















      ..
          !! processed by numpydoc !!


.. py:class:: Locus

   
   Control locus of a synchronous machine.

   :param i_s_dq: Stator current (A) in rotor coordinates.
   :type i_s_dq: ndarray
   :param psi_s_dq: Stator flux linkage (Vs) in rotor coordinates.
   :type psi_s_dq: ndarray
   :param tau_M: Electromagnetic torque (Nm).
   :type tau_M: ndarray















   ..
       !! processed by numpydoc !!

.. py:class:: MachineCharacteristics(par, i_s_vals, base = None, num = 16)

   
   Visualize the optimal control loci of synchronous machines.

   The MTPA locus and the MTPV locus (if it exists) are computed up to the largest
   given current magnitude, and the constant-current loci between them at each given
   current magnitude. The methods plot these loci in different planes.

   :param par: Machine parameters.
   :type par: ~motulator.drive.control.sm.BaseSynchronousMachineControlPars
   :param i_s_vals: Current magnitudes (A) of the constant-current loci, in ascending order. If
                    `base` is given, the values are per-unit values.
   :type i_s_vals: float | Sequence[float]
   :param base: Base values. If given, the loci are plotted in per-unit values, otherwise in SI
                units.
   :type base: BaseValues, optional
   :param num: Number of points of each locus, defaults to 16.
   :type num: int, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: plot_current_loci(latex = False, save_path = None, **savefig_kwargs)

      
      Plot the loci in the current plane.

      :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working
                    LaTeX installation, defaults to False.
      :type latex: bool, optional
      :param save_path: Path to save the figure. If None, the figure is not saved.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_current_vs_torque(latex = False, save_path = None, **savefig_kwargs)

      
      Plot the current components as functions of the torque.

      :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working
                    LaTeX installation, defaults to False.
      :type latex: bool, optional
      :param save_path: Path to save the figure. If None, the figure is not saved.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_loci(latex = False, save_path = None, **savefig_kwargs)

      
      Plot the loci in the flux linkage plane.

      :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working
                    LaTeX installation, defaults to False.
      :type latex: bool, optional
      :param save_path: Path to save the figure. If None, the figure is not saved.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


   .. py:method:: plot_flux_vs_torque(latex = False, save_path = None, **savefig_kwargs)

      
      Plot the flux linkage magnitude as a function of the torque.

      :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working
                    LaTeX installation, defaults to False.
      :type latex: bool, optional
      :param save_path: Path to save the figure. If None, the figure is not saved.
      :type save_path: str | Path, optional
      :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















      ..
          !! processed by numpydoc !!


.. py:class:: MagneticModel

   
   Store and manipulate flux linkage or current maps for synchronous machines.

   :param i_s_dq: Complex array of stator current (A).
   :type i_s_dq: np.ndarray
   :param psi_s_dq: Complex array of stator flux linkage (Vs).
   :type psi_s_dq: np.ndarray
   :param lookup_fcn: Function that evaluates the map at arbitrary points. Takes complex inputs (d +
                      j*q) and returns the output values. For flux maps, maps i_s_dq → psi_s_dq; for
                      current maps, maps psi_s_dq → i_s_dq. If not given, a linear interpolation
                      function, which extrapolates outside the map range, is created from the data.
   :type lookup_fcn: Callable[[complex | np.ndarray], complex | np.ndarray], optional
   :param tau_M: Array of electromagnetic torque (Nm).
   :type tau_M: np.ndarray, optional
   :param type: Type of the map, defaults to "flux_map".
   :type type: Literal["current_map", "flux_map"], optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_interpolated_model(d_range = None, q_range = None, num = None, invert = False)

      
      Interpolate or invert this magnetic model onto a regular grid.

      :param d_range: Range for the d axis. If None, the range is determined from the data,
                      defaults to None.
      :type d_range: np.ndarray | None, optional
      :param q_range: Range for the q axis. If None, the range is determined from the data,
                      defaults to None.
      :type q_range: np.ndarray | None, optional
      :param num: Number of points in each axis. If None, uses the maximum dimension from the
                  original map to preserve resolution, defaults to None.
      :type num: int | None, optional
      :param invert: Invert the map (swap input and output), defaults to False.
      :type invert: bool, optional

      :returns: Interpolated magnetic model.
      :rtype: MagneticModel















      ..
          !! processed by numpydoc !!


   .. py:method:: invert(d_range = None, q_range = None, num = None)

      
      Invert the map (swap input and output).

      :param d_range: Range for the d axis. If None, the range is determined from the data,
                      defaults to None.
      :type d_range: np.ndarray | None, optional
      :param q_range: Range for the q axis. If None, the range is determined from the data,
                      defaults to None.
      :type q_range: np.ndarray | None, optional
      :param num: Number of points in each axis. If None, uses the maximum dimension from the
                  original map to preserve resolution, defaults to None.
      :type num: int | None, optional

      :returns: Inverted magnetic model on a regular grid.
      :rtype: MagneticModel















      ..
          !! processed by numpydoc !!


.. py:class:: NominalValues

   
   Nominal values.

   :param U: Voltage (V, rms, line-to-line).
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

.. py:class:: SaturationModelBase

   
   Base class for saturation models given as functions.

   A subclass implements `__call__`, which evaluates the map: the current as a function
   of the flux linkage for a current map, or the flux linkage as a function of the
   current for a flux map, as declared by `map_type`. The model can be used directly as
   the lookup function of a `MagneticModel`, and `as_magnetic_model` samples it on a
   grid.















   ..
       !! processed by numpydoc !!

   .. py:method:: as_magnetic_model(d_range, q_range, n_p = None)

      
      Sample the map on a grid as a magnetic model.

      :param d_range: Range for the d-axis input. For current maps, this is psi_d (Vs). For flux
                      maps, this is i_d (A).
      :type d_range: ndarray
      :param q_range: Range for the q-axis input. For current maps, this is psi_q (Vs). For flux
                      maps, this is i_q (A).
      :type q_range: ndarray
      :param n_p: Number of pole pairs. If provided, the torque is included.
      :type n_p: int, optional

      :returns: Magnetic model using this object as its `lookup_fcn`.
      :rtype: MagneticModel















      ..
          !! processed by numpydoc !!


.. py:class:: SaturationModelPMSyRM

   Bases: :py:obj:`SaturationModelSyRM`


   
   Saturation model for PM synchronous reluctance machines.

   This model takes into account the bridge saturation in addition to the self- and
   cross-saturation effects of the d and q axes [#Lel2024]_. The bridge saturation
   model is based on a nonlinear reluctance element in parallel with the
   Norton-equivalent PM model. The other parameters are those of `SaturationModelSyRM`.

   :param psi_n: Flux linkage (Vs) of the Norton-equivalent PM model, i.e., the d-axis flux
                 linkage at which the bridge current ``G_b*(psi_d - psi_n)`` vanishes. It is
                 higher than the PM flux linkage at zero current.
   :type psi_n: float
   :param a_b: Coefficient for bridge inverse inductance.
   :type a_b: float
   :param a_bp: Coefficient for bridge saturation.
   :type a_bp: float
   :param W: Exponent for bridge saturation.
   :type W: float
   :param k_q: Cross-coupling factor for bridge flux.
   :type k_q: float

   .. rubric:: References

   .. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a
      simplified equivalent magnetic circuit for permanent magnet machines," Proc.
      ICEM, 2024, https://doi.org/10.1109/ICEM60801.2024.10700403















   ..
       !! processed by numpydoc !!

.. py:class:: SaturationModelSyRM

   Bases: :py:obj:`SaturationModelBase`


   
   Saturation model for synchronous reluctance machines.

   This model takes into account the self- and cross-saturation effects of the d and q
   axes [#Hin2017]_.

   :param a_d0: Offset coefficient for the d-axis inverse inductance.
   :type a_d0: float
   :param a_dd: Self-saturation coefficient for the d axis.
   :type a_dd: float
   :param a_q0: Offset coefficient for the q-axis inverse inductance.
   :type a_q0: float
   :param a_qq: Self-saturation coefficient for the q axis.
   :type a_qq: float
   :param a_dq: Cross-saturation coefficient.
   :type a_dq: float
   :param S: Exponent for the d-axis self-saturation.
   :type S: float
   :param T: Exponent for the q-axis self-saturation.
   :type T: float
   :param U: First exponent for cross-saturation.
   :type U: float
   :param V: Second exponent for cross-saturation.
   :type V: float

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
   :param periodic: Repeat the sequence with the period ``times[-1] - times[0]``, defaults to False.
   :type periodic: bool, optional















   ..
       !! processed by numpydoc !!

.. py:function:: import_syre_data(fname, add_negative_q_axis = True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format, please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which the PM
   flux is along the d axis.

   :param fname: MATLAB file name.
   :type fname: Path | str
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

   
   Plot the drive signals in selectable subplots.

   :param res: Simulation results. Without a control system, the magnitudes of the stator
               current and voltage are plotted instead of the controller signals.
   :type res: ~motulator.common.model.SimulationResults
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
   :type y_lims: Sequence[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: Sequence[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
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
   :param base: Base values for scaling the maps. If not given, the maps are plotted in SI
                units.
   :type base: BaseValues, optional
   :param x_lims: x-axis limits. The q-axis curves are plotted at the d-axis currents closest to
                  these limits, or at the extreme d-axis currents of the data if not given.
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

.. py:function:: plot_map(data, component, base = None, lims = None, ticks = None, raw_data = None, current_loci = None, axlim_clip = True, latex = False, save_path = None, **savefig_kwargs)

   
   Plot component (d or q) of flux linkage or current maps.

   :param data: Data containing the flux and current information.
   :type data: MagneticModel
   :param component: Which component to plot: "d" for d-axis, "q" for q-axis.
   :type component: {"d", "q"}
   :param base: Base values for scaling the maps. If not given, the maps are plotted in SI
                units.
   :type base: BaseValues, optional
   :param lims: Axis limits. Keys should be 'x', 'y', 'z'.
   :type lims: dict[str, tuple[float, float]], optional
   :param ticks: Axis tick locations. Keys should be 'x', 'y', 'z'.
   :type ticks: dict[str, ArrayLike], optional
   :param raw_data: Raw data for comparison, shown as scatter points (blue and red alternately for a
                    list). A tuple of arrays is interpreted as `(psi_s_dq, i_s_dq, ...)` in SI
                    units.
   :type raw_data: MagneticModel | tuple | list[MagneticModel | tuple], optional
   :param current_loci: Levels `(i_d_levels, i_q_levels)` of the constant-current loci, in the units of
                        the plot. If given, the loci are drawn on the surface instead of the wireframe.
                        The map is evaluated along the loci by the lookup function of `data`.
   :type current_loci: tuple[ArrayLike, ArrayLike], optional
   :param axlim_clip: Whether to hide the parts of the surface and wireframe outside the axis view
                      limits, defaults to True. The grid is cropped to the x and y limits, and the
                      flag is passed to the `axlim_clip` argument of matplotlib's 3D plotting
                      functions.
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
   :type y_lims: Sequence[tuple[float, float] | None], optional
   :param y_ticks: y-axis tick locations for each subplot.
   :type y_ticks: Sequence[ArrayLike | None], optional
   :param latex: Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
                 installation, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path, optional
   :param \*\*savefig_kwargs: Additional keyword arguments passed to plt.savefig().















   ..
       !! processed by numpydoc !!

