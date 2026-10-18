motulator.drive.gradnet
=======================

.. py:module:: motulator.drive.gradnet

.. autoapi-nested-parse::

   
   Gradient-based neural network maps.

   The trained networks (see `load_gradnet`) and the maps based on them (e.g.,
   `CurrentMap`) are evaluated with NumPy. Training the networks (see `training`)
   requires PyTorch.















   ..
       !! processed by numpydoc !!


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/motulator/drive/gradnet/training/index


Classes
-------

.. autoapisummary::

   motulator.drive.gradnet.CurrentMap
   motulator.drive.gradnet.CurrentMapWithHarmonics
   motulator.drive.gradnet.FluxMap
   motulator.drive.gradnet.FluxMapWithHarmonics
   motulator.drive.gradnet.GradNetFunction
   motulator.drive.gradnet.PlotOptions


Functions
---------

.. autoapisummary::

   motulator.drive.gradnet.load_gradnet
   motulator.drive.gradnet.plot_output_vs_angle
   motulator.drive.gradnet.plot_surface_vs_current_and_angle
   motulator.drive.gradnet.print_current_map_errors_meas
   motulator.drive.gradnet.print_flux_map_errors_meas


Package Contents
----------------

.. py:class:: CurrentMap(model)

   Bases: :py:obj:`motulator.drive.utils._sm_flux_maps.SaturationModelBase`


   
   Callable wrapper for GradNet current map models.

   The map is symmetrized about the d-axis to ensure physical consistency. The map can
   be sampled on a grid by `as_magnetic_model()`, e.g., for plotting.

   :param model: Trained GradNet model for the current map.
   :type model: GradNetFunction | GradNet















   ..
       !! processed by numpydoc !!

   .. py:method:: jacobian(x_dq)

      
      Jacobian matrix of the symmetrized map.

      :param x_dq: Input of the map (flux linkage in Vs or current in A).
      :type x_dq: complex | np.ndarray

      :returns: Jacobian matrix [[dy_d/dx_d, dy_d/dx_q], [dy_q/dx_d, dy_q/dx_q]], where y is
                the output of the map, in SI units.
      :rtype: np.ndarray, shape (..., 2, 2)















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentMapWithHarmonics(model, k = 6)

   
   Callable wrapper for GradNet current maps with spatial harmonics.

   :param model: Trained GradNet model for the current map with harmonics.
   :type model: GradNetFunction | GradNet
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMap(model)

   Bases: :py:obj:`CurrentMap`


   
   Callable wrapper for GradNet flux-linkage map models.

   The map returns the stator flux linkage (Vs) at the given stator current (A). It
   is symmetrized about the d-axis to ensure physical consistency.

   :param model: Trained GradNet model for the flux-linkage map.
   :type model: GradNetFunction | GradNet















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMapWithHarmonics(model, k = 6)

   Bases: :py:obj:`CurrentMapWithHarmonics`


   
   Callable wrapper for GradNet flux maps with spatial harmonics.

   :param model: Trained GradNet model for the flux map with harmonics.
   :type model: GradNetFunction | GradNet
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: GradNetFunction

   
   Trained GradNet as a numpy function in double precision.

   The function maps the per-unit input, shape (..., in_dim), to the per-unit output
   of the same shape, see `GradNet`. It is created from a trained model by
   `from_model`, saved to an npz file by `save`, and loaded by `load_gradnet`.

   :param W: Weights of the modules.
   :type W: ndarray, shape (num_modules, embed_dim, in_dim)
   :param b: Biases of the modules.
   :type b: ndarray, shape (num_modules, embed_dim)
   :param beta_log: Logarithms of the scales of the activations of the modules.
   :type beta_log: ndarray, shape (num_modules,)
   :param mu_log: Logarithms of the coefficients of the linear term.
   :type mu_log: ndarray, shape (mu_dim,)
   :param bias: Bias of the output.
   :type bias: ndarray, shape (in_dim,)
   :param activation: Name of the activation function, see `ACTIVATIONS`.
   :type activation: str
   :param p: Exponent of the p-norm gradient activation (0 for the other activations).
   :type p: int
   :param psi_base: Base value of the flux linkage (Vs).
   :type psi_base: float
   :param i_base: Base value of the current (A).
   :type i_base: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_model(model)
      :classmethod:


      
      Create the function from a trained `GradNet` model.

      The modules of the model must have the same activation, one of `ACTIVATIONS`
      applied along the last axis (the default), with the same exponent `p`.















      ..
          !! processed by numpydoc !!


   .. py:method:: save(path)

      
      Save the parameters to an npz file.
















      ..
          !! processed by numpydoc !!


.. py:class:: PlotOptions

   
   Options for plotting functions.

   :param base: Base values for per-unit conversion. If None, SI values are used.
   :type base: ~motulator.drive.utils.BaseValues | None, optional
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

.. py:function:: load_gradnet(model_path)

   
   Load a trained GradNet from an npz file, see `GradNetFunction`.

   :param model_path: Path to the saved model file.
   :type model_path: Path | str

   :returns: Trained network as a numpy function.
   :rtype: GradNetFunction















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
   :param val_data: Validation data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
   :type val_data: tuple, optional
   :param trn_data: Training data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
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
   :param val_data: Validation data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
   :type val_data: tuple, optional
   :param trn_data: Training data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
   :type trn_data: tuple, optional
   :param opts: Plotting options.
   :type opts: PlotOptions, optional















   ..
       !! processed by numpydoc !!

.. py:function:: print_current_map_errors_meas(current_map, data, base = None)

   
   Print error metrics for a measured current map.
















   ..
       !! processed by numpydoc !!

.. py:function:: print_flux_map_errors_meas(flux_map, data, base = None)

   
   Print error metrics for a measured flux map.
















   ..
       !! processed by numpydoc !!

