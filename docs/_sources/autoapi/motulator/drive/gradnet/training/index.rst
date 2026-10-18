motulator.drive.gradnet.training
================================

.. py:module:: motulator.drive.gradnet.training

.. autoapi-nested-parse::

   Training of GradNets.

   The networks are defined and trained with PyTorch, which is installed as an optional
   dependency with `pip install "motulator[gradnet]"`. The trained networks are saved
   as numpy functions, see `motulator.drive.gradnet.load_gradnet`.

   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.gradnet.training.AlgebraicSigmoid
   motulator.drive.gradnet.training.GradNet
   motulator.drive.gradnet.training.PNormGradient
   motulator.drive.gradnet.training.Softmax
   motulator.drive.gradnet.training.Squareplus


Functions
---------

.. autoapisummary::

   motulator.drive.gradnet.training.get_training_data
   motulator.drive.gradnet.training.train_gradnet


Module Contents
---------------

.. py:class:: AlgebraicSigmoid(beta_log0 = -3.0)

   Bases: :py:obj:`torch.nn.Module`


   
   Sigmoid-type activation function with one learnable parameter.
















   ..
       !! processed by numpydoc !!

.. py:class:: GradNet(in_dim = 2, mu_dim = 2, num_modules = 1, embed_dim = 12, activation = Softmax, mu_log0 = 1.0, psi_base = 1.0, i_base = 1.0)

   Bases: :py:obj:`torch.nn.Module`


   
   Modular and monotonous gradient network.

   :param in_dim: Input dimension, defaults to 2.
   :type in_dim: int, optional
   :param mu_dim: Dimension of the linear term, defaults to 2.
   :type mu_dim: int, optional
   :param num_modules: Number of GradNet modules, defaults to 1.
   :type num_modules: int, optional
   :param embed_dim: Embedding dimension for the GradNet modules, defaults to 12.
   :type embed_dim: int, optional
   :param activation: Activation factory used by GradNet modules, defaults to Softmax.
   :type activation: Callable[[], nn.Module], optional
   :param mu_log0: Initial value for the linear term coefficients in log-domain, defaults to 1.0.
   :type mu_log0: float, optional
   :param psi_base: Base value of the flux linkage (Vs), defaults to 1.0. The network operates in
                    per-unit quantities. The base values are stored in the model and used by the
                    map wrappers (e.g., `CurrentMap`) to convert between SI units and per-unit
                    quantities.
   :type psi_base: float, optional
   :param i_base: Base value of the current (A), defaults to 1.0.
   :type i_base: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: PNormGradient(dim = -1, p = 8, beta_log0 = 0.0, freeze_beta = False)

   Bases: :py:obj:`torch.nn.Module`


   
   p-norm gradient activation function.

   Defined as the gradient of S(z) = (1 + sum((beta*z_n)**p))**(1/p)/beta, where p is a
   positive even integer. This potential function corresponds to a smooth p-norm, which
   is convex, thus guaranteeing monotonicity.















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

   :param dataset_path: Path to the npz file containing the dataset.
   :type dataset_path: str
   :param base: Base values of the target machine. They are only needed to re-instantiate the
                dataset; the returned data are not normalized.
   :type base: ~motulator.drive.utils.BaseValues
   :param subsample: Subsampling rate used in the training, defaults to 1 (no subsampling).
   :type subsample: int, optional
   :param other_keys: Additional fields of the npz file to be returned after the flux linkage and
                      current, defaults to None (no additional fields).
   :type other_keys: list[str] | None, optional

   :returns: ((train_psi, train_i, ...), (val_psi, val_i, ...)), where the arrays are
             flattened and in the units of the npz file.
   :rtype: tuple















   ..
       !! processed by numpydoc !!

.. py:function:: train_gradnet(dataset_path, base, is_flux_map=False, k = None, num_modules = 1, embed_dim = 12, batch_size = 128, epochs = 2000, lr = 0.001, save_model_path = None, subsample = 1, activation = None, device = None, *, i_low = 0.0, weight_low = 1.0)

   
   Train and save the GradNet model.

   :param dataset_path: Path to the training data file (npz format).
   :type dataset_path: str | Path
   :param base: Base values of the target machine. The training data are normalized with the
                base current and flux linkage, which are also stored in the model.
   :type base: ~motulator.drive.utils.BaseValues
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
   :param save_model_path: Path to save the trained model (npz file, see `GradNetFunction`). If None,
                           saves to `model.npz` in current directory.
   :type save_model_path: str | Path | None, optional
   :param subsample: Subsampling factor for the training data, defaults to 1 (uses all data).
   :type subsample: int, optional
   :param activation: Activation function factory, defaults to PNormGradient. The trained model is
                      saved as a numpy function, which supports the activations `Softmax`,
                      `PNormGradient`, `Squareplus`, and `AlgebraicSigmoid` (see `GradNetFunction`).
   :type activation: Callable[[], torch.nn.Module] | None, optional
   :param device: Device to use for training. If None, automatically selects CUDA if available,
                  otherwise CPU.
   :type device: torch.device | None, optional
   :param i_low: Current limit (A, peak value) defining the low-current region, defaults to 0.
   :type i_low: float, optional
   :param weight_low: Loss weight of the data points whose current magnitude is below `i_low`,
                      defaults to 1 (unweighted loss). The low-current region is determined by the
                      current of the data point for both flux and current maps. A weight above 1
                      improves the accuracy at low currents, where a uniform data grid gives only a
                      few points. The weights are normalized with their mean over the training set,
                      so the loss of each batch is an unbiased estimate of the weighted loss of the
                      whole set, which corresponds to repeating those points `weight_low` times in the
                      dataset, except that the number of steps per epoch is unchanged.
   :type weight_low: float, optional















   ..
       !! processed by numpydoc !!

