motulator.drive.gradnet
=======================

.. py:module:: motulator.drive.gradnet

.. autoapi-nested-parse::

   
   Gradient-based neural network maps.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.gradnet.AlgebraicSigmoid
   motulator.drive.gradnet.CurrentMap
   motulator.drive.gradnet.CurrentMapWithHarmonics
   motulator.drive.gradnet.FluxMap
   motulator.drive.gradnet.FluxMapWithHarmonics
   motulator.drive.gradnet.PNormGradient
   motulator.drive.gradnet.PlotOptions
   motulator.drive.gradnet.Softmax
   motulator.drive.gradnet.Squareplus


Functions
---------

.. autoapisummary::

   motulator.drive.gradnet.get_training_data
   motulator.drive.gradnet.load_gradnet
   motulator.drive.gradnet.plot_maps
   motulator.drive.gradnet.plot_output_vs_angle
   motulator.drive.gradnet.plot_surface_vs_current_and_angle
   motulator.drive.gradnet.print_current_map_errors_meas
   motulator.drive.gradnet.print_flux_map_errors_meas
   motulator.drive.gradnet.sample_map_on_grid
   motulator.drive.gradnet.train_gradnet


Package Contents
----------------

.. py:class:: AlgebraicSigmoid(beta_log0 = -3.0)

   Bases: :py:obj:`torch.nn.Module`


   
   Sigmoid-type activation function with one learnable parameter.
















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentMap(model)

   
   Callable wrapper for GradNet current map models.

   The map is symmetrized along to the d-axis to ensure physical consistency.

   :param model: Trained GradNet model for the current map.
   :type model: GradNet















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentMapWithHarmonics(model, k = 6)

   
   Callable wrapper for GradNet current maps with spatial harmonics.

   :param model: Trained GradNet model for the current map with harmonics.
   :type model: GradNet
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMap(model)

   Bases: :py:obj:`CurrentMap`


   
   Callable wrapper for GradNet current map models.

   The map is symmetrized along to the q-axis to ensure physical consistency.

   :param model: Trained GradNet model for the current map.
   :type model: GradNet

   :returns: Stator flux linkage (Vs).
   :rtype: complex | np.ndarray















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMapWithHarmonics(model, k = 6)

   
   Callable wrapper for GradNet flux maps with spatial harmonics.

   :param model: Trained GradNet model for the flux map with harmonics.
   :type model: GradNet
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: PNormGradient(dim = -1, p = 8, beta_log0 = 0.0, freeze_beta = False)

   Bases: :py:obj:`torch.nn.Module`


   
   p-norm gradient activation function.

   Defined as the gradient of S(z) = (1 + sum(z_n**p))**(1/p)/beta, where p is a
   positive even integer. This potential function corresponds to a smooth p-norm, which
   is convex, thus guaranteeing monotonicity.















   ..
       !! processed by numpydoc !!

.. py:class:: PlotOptions

   
   Options for plotting functions.

   :param base: Base values for per-unit conversion. If None, SI values are used.
   :type base: BaseValues | None, optional
   :param lims: Axis limits as {'x': (xmin, xmax), 'y': (ymin, ymax)}.
   :type lims: dict[str, tuple[float, float]] | None, optional
   :param ticks: Axis ticks as {'x': [x1, x2, ...], 'y': [y1, y2, ...]}.
   :type ticks: dict[str, list[float]] | None, optional
   :param surface_cmap: Colormap for the surface plot, defaults to "viridis".
   :type surface_cmap: str, optional
   :param latex: Use LaTeX fonts if True, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure. If None, the figure is not saved.
   :type save_path: str | Path | None, optional
   :param savefig_kwargs: Additional keyword arguments passed to plt.savefig().
   :type savefig_kwargs: Dict[str, Any], optional
   :param loci_levels_source: Source for constant-loci levels in `plot_surface_vs_current_and_angle()` when
                              plotting over current and angle. If "ticks", uses `ticks["x"]`/`ticks["y"]`
                              (with lims/fallback defaults). If "val" or "trn", uses unique x/y values
                              present in the corresponding point cloud slice.
   :type loci_levels_source: {"ticks", "val", "trn"}, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Softmax(dim = -1, beta_log0 = 2.0, freeze_beta = False)

   Bases: :py:obj:`torch.nn.Module`


   
   Softmax activation function.
















   ..
       !! processed by numpydoc !!

.. py:class:: Squareplus(beta_log0 = -2.0)

   Bases: :py:obj:`torch.nn.Module`


   
   Rectifier-type activation function with one learnable parameter.
















   ..
       !! processed by numpydoc !!

.. py:function:: get_training_data(dataset_path, base, subsample = 1, other_keys = None)

   
   Get the training and validation data used by the model.

   This function re-instantiates the dataset with the same parameters to reproduce
   the training set. Validation data is the set difference (complement).

   :returns: ((train_psi, train_i, ...), (val_psi, val_i, ...))
   :rtype: tuple















   ..
       !! processed by numpydoc !!

.. py:function:: load_gradnet(model_path, activation = None)

   
   Load a GradNet model, inferring dimensions from the saved weights.

   :param model_path: Path to the saved model file.
   :type model_path: Path | str
   :param activation: Activation function factory, defaults to PNormGradient.
   :type activation: Callable[[], nn.Module] | None, optional

   :returns: Loaded GradNet model.
   :rtype: GradNet















   ..
       !! processed by numpydoc !!

.. py:function:: plot_maps(data, component, base = None, lims = None, ticks = None, raw_data = None, surface_cmap = 'viridis', current_loci = False, current_loci_levels = None, latex = False, save_path = None, **savefig_kwargs)

   
   Plot flux linkage or current maps.

   :param data: Sampled rectilinear-grid map data.
   :type data: MapGrid
   :param component: Component of the flux linkage or current to plot.
   :type component: {"d", "q"}
   :param base: Base values for per-unit conversion. If None, unity base values are used.
   :type base: BaseValues | None, optional
   :param lims: Axis limits as {'x': (xmin, xmax), 'y': (ymin, ymax), 'z': (zmin, zmax)}.
   :type lims: dict[str, tuple[float, float]] | None, optional
   :param ticks: Axis ticks as {'x': [x1, x2, ...], 'y': [y1, y2, ...], 'z': [z1, z2, ...]}.
   :type ticks: dict[str, list[float]] | None, optional
   :param raw_data: Raw validation/training data scatter overlays.
   :type raw_data: tuple | list[tuple] | None, optional
   :param surface_cmap: Colormap for the surface plot, defaults to "viridis".
   :type surface_cmap: str, optional
   :param current_loci: Plot constant-current loci overlays, defaults to False.
   :type current_loci: bool, optional
   :param current_loci_levels: Levels for the constant-current loci overlays. If list or array, the same
                               levels are used for `i_d` and `i_q`. If a tuple of two lists/arrays,
                               `(i_d_levels, i_q_levels)`. Defaults to None.
   :type current_loci_levels: list | ndarray | tuple | None, optional
   :param latex: Use LaTeX fonts if True, defaults to False.
   :type latex: bool, optional
   :param save_path: Path to save the figure, defaults to None (not saved).
   :type save_path: str | Path | None, optional
   :param savefig_kwargs: Additional keyword arguments passed to `plt.savefig()`.
   :type savefig_kwargs: Any















   ..
       !! processed by numpydoc !!

.. py:function:: plot_output_vs_angle(fixed_value, theta_m_range, map_fcn, output = 'tau_m', input_type = 'i_s_dq', val_data = None, trn_data = None, opts = None)

   
   Plot torque or flux linkage as a function of angle at a fixed input.

   :param fixed_value: Fixed complex input value (e.g., i_s_dq or psi_s_dq).
   :type fixed_value: complex
   :param theta_m_range: Electrical rotor angle range (rad).
   :type theta_m_range: np.ndarray
   :param map_fcn: Flux or current map function with harmonics.
   :type map_fcn: Callable
   :param output: Output quantity to plot, defaults to "tau_m".
   :type output: {"psi_d", "psi_q", "tau_m"}, optional
   :param input_type: Type of the fixed input value, defaults to "i_s_dq".
   :type input_type: {"i_s_dq", "psi_s_dq"}, optional
   :param val_data: Validation data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
   :type val_data: tuple, optional
   :param trn_data: Training data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
   :type trn_data: tuple, optional
   :param opts: Plotting options.
   :type opts: PlotOptions, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_surface_vs_current_and_angle(current_range, fixed_value, theta_m_range, map_fcn, input = 'i_q', output = 'tau_m', val_data = None, trn_data = None, opts = None)

   
   Plot selected output component vs selected input component over angle.

   :param current_range: Range of input values to vary.
   :type current_range: np.ndarray
   :param fixed_value: Fixed input value for the other axis.
   :type fixed_value: float
   :param theta_m_range: Electrical rotor angle range (rad).
   :type theta_m_range: np.ndarray
   :param map_fcn: Flux or current map function with harmonics.
   :type map_fcn: Callable
   :param input: Input axis to vary, defaults to "i_q".
   :type input: {"i_d", "i_q", "psi_d", "psi_q"}, optional
   :param output: Output quantity to plot, defaults to "tau_m".
   :type output: {"psi_d", "psi_q", "tau_m", "i_d", "i_q"}, optional
   :param val_data: Validation data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
   :type val_data: tuple, optional
   :param trn_data: Training data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
   :type trn_data: tuple, optional
   :param opts: Plotting options.
   :type opts: PlotOptions, optional















   ..
       !! processed by numpydoc !!

.. py:function:: print_current_map_errors_meas(current_map, data, base = None)

   
   Print per-unit error metrics for a measured current map.
















   ..
       !! processed by numpydoc !!

.. py:function:: print_flux_map_errors_meas(flux_map, data, base = None)

   
   Print per-unit error metrics for a measured flux map.
















   ..
       !! processed by numpydoc !!

.. py:function:: sample_map_on_grid(map_fcn, map_type, d_range, q_range)

   
   Sample a (measured or learned) map callable on a rectilinear grid.

   For `map_type="current_map"`, (`d_range`, `q_range`) are in (`psi_d`, `psi_q`)
   units. For `map_type="flux_map"`, (`d_range`, `q_range`) are in (`i_d`, `i_q`)
   units.

   :param map_fcn: Callable map function.
   :type map_fcn: Callable[[complex | ndarray], complex | ndarray]
   :param map_type: Type of the map.
   :type map_type: {"current_map", "flux_map"}
   :param d_range: Range of values for the d-axis.
   :type d_range: ndarray
   :param q_range: Range of values for the q-axis.
   :type q_range: ndarray

   :returns: Sampled grid data.
   :rtype: MapGrid















   ..
       !! processed by numpydoc !!

.. py:function:: train_gradnet(dataset_path, base, is_flux_map=False, k = None, num_modules = 1, embed_dim = 12, batch_size = 128, epochs = 2000, lr = 0.001, save_model_path = None, subsample = 1, activation = None, device = None)

   
   Train and save the GradNet model.

   :param dataset_path: Path to the training data file (npz format).
   :type dataset_path: str | Path
   :param is_flux_map: Whether the model is a flux map or current map, defaults to False.
   :type is_flux_map: bool, optional
   :param k: Spatial harmonics order. If None, no harmonics are used, defaults to None.
   :type k: int | None, optional
   :param num_modules: Number of GradNet modules, defaults to 1.
   :type num_modules: int, optional
   :param embed_dim: Embedding dimension for the GradNet modules, defaults to 12.
   :type embed_dim: int, optional
   :param batch_size: Batch size for training, defaults to 128.
   :type batch_size: int, optional
   :param epochs: Number of training epochs, defaults to 2000.
   :type epochs: int, optional
   :param lr: Learning rate for the optimizer, defaults to 1e-3.
   :type lr: float, optional
   :param save_model_path: Path to save the trained model. If None, saves to `model.pth` in current
                           directory.
   :type save_model_path: str | Path | None, optional
   :param subsample: Subsampling factor for the training data, defaults to 1 (uses all data).
   :type subsample: int, optional
   :param activation: Activation function factory, defaults to PNormGradient.
   :type activation: Callable[[], torch.nn.Module] | None, optional
   :param device: Device to use for training. If None, automatically selects CUDA if available,
                  otherwise CPU.
   :type device: torch.device | None, optional















   ..
       !! processed by numpydoc !!

