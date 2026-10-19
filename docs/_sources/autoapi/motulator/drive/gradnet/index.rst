motulator.drive.gradnet
=======================

.. py:module:: motulator.drive.gradnet

.. autoapi-nested-parse::

   
   Gradient-based neural network maps.

   The magnetic maps (e.g., `CurrentMap`) evaluate trained gradient networks (GradNets)
   with NumPy, see `load_gradnet`. The networks are trained with the related package
   motulator-gradnet, https://github.com/Aalto-Electric-Drives/motulator-gradnet.















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.gradnet.CurrentMap
   motulator.drive.gradnet.CurrentMapWithHarmonics
   motulator.drive.gradnet.FluxMap
   motulator.drive.gradnet.FluxMapWithHarmonics
   motulator.drive.gradnet.GradNetFunction


Functions
---------

.. autoapisummary::

   motulator.drive.gradnet.load_gradnet


Package Contents
----------------

.. py:class:: CurrentMap(model)

   Bases: :py:obj:`motulator.drive.utils._sm_flux_maps.SaturationModelBase`


   
   Callable wrapper for GradNet current map models.

   The map is symmetrized about the d-axis to ensure physical consistency. The map can
   be sampled on a grid by `as_magnetic_model()`, e.g., for plotting.

   :param model: Trained GradNet for the current map, see `load_gradnet`.
   :type model: GradNetFunction















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

   :param model: Trained GradNet for the current map with harmonics, see `load_gradnet`.
   :type model: GradNetFunction
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMap(model)

   Bases: :py:obj:`CurrentMap`


   
   Callable wrapper for GradNet flux-linkage map models.

   The map returns the stator flux linkage (Vs) at the given stator current (A). It
   is symmetrized about the d-axis to ensure physical consistency.

   :param model: Trained GradNet for the flux-linkage map, see `load_gradnet`.
   :type model: GradNetFunction















   ..
       !! processed by numpydoc !!

.. py:class:: FluxMapWithHarmonics(model, k = 6)

   Bases: :py:obj:`CurrentMapWithHarmonics`


   
   Callable wrapper for GradNet flux maps with spatial harmonics.

   :param model: Trained GradNet for the flux map with harmonics, see `load_gradnet`.
   :type model: GradNetFunction
   :param k: Spatial harmonic order, defaults to 6.
   :type k: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: GradNetFunction

   
   Trained GradNet as a numpy function in double precision.

   The function maps the per-unit input, shape (..., in_dim), to the per-unit output
   of the same shape, which is the gradient of the scalar state function of the
   network. It is created from a trained model by the related package
   motulator-gradnet, saved to an npz file by `save`, and loaded by `load_gradnet`.

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

   .. py:method:: save(path)

      
      Save the parameters to an npz file.
















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

