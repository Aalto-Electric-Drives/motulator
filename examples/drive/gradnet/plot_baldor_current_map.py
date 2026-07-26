"""
Train current map (5.6-kW PM-SyRM Baldor)
============================================

This script demonstrates how to train GradNet current map from
a four-pole 5.6-kW PM synchronous reluctance machine (ABB Baldor ECS101M0H7EF4).
It includes loading a dataset, training a GradNet model,
and visualizing the trained model against the original dataset.
It can be run in following options:
1. Without spatial harmonics using measurement dataset.
2. With spatial harmonics using FEM dataset.

"""

from pathlib import Path

import numpy as np

from motulator.drive import utils
from motulator.drive.utils import (
    get_training_data,
    gn,
    plot_gn_map,
    print_meas_current_map_error_metrics,
    sample_map_on_grid,
    stat_fem_curr,
    train_gradnet,
)

# %%
# Set nominal and base values.
nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Set up the paths and parameters.
p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
model_with_harmonics = True

# Train model
if not model_with_harmonics:
    dataset_path = p / "datasets/baldor_meas.npz"
    trained_model_path = (
        p / "trained_models/baldor_meas_current_map_squareplus_d12_sub10_.pth"
    )
    subsample = 10
    activation = gn.Squareplus
    # %%
    # Train the model.
    if not trained_model_path.exists():
        train_gradnet(
            dataset_path=dataset_path,
            base=base,
            save_model_path=trained_model_path,
            epochs=20000,
            subsample=subsample,
            embed_dim=12,
            activation=activation,
        )

    # %%
    # Create the GradNet model and its callable.
    model = gn.load_gradnet(trained_model_path, activation=activation)
    current_map_fcn = gn.CurrentMap(model)
    current_map = sample_map_on_grid(
        current_map_fcn,
        map_type="current_map",
        d_range=np.linspace(0 * base.psi, 1 * base.psi, 50),
        q_range=np.linspace(-1.5 * base.psi, 1.5 * base.psi, 50),
    )

    # Constant current contours corresponding to the measured dataset, for visualization
    i_d_levels = np.arange(-20, 22, 2) / base.i
    i_q_levels = np.arange(-26, 28, 2) / base.i
    current_loci_levels = (i_d_levels, i_q_levels)

    # %%
    # Load the dataset for comparison and split it into training and validation sets.
    train_data, val_data = get_training_data(
        str(dataset_path), subsample=subsample, base=base
    )
    # %%
    # Plot the current map.
    plot_gn_map(
        current_map,
        "d",
        base,
        lims={"x": (0, 1), "y": (-1.5, 1.5), "z": (-2, 4)},
        ticks={"x": [0, 0.5, 1], "y": [-1.5, 0, 1.5], "z": [-2, 0, 2, 4]},
        raw_data=[val_data, train_data],
        current_loci=True,
        current_loci_levels=current_loci_levels,
        latex=False,
        save_path=p / "figs" / "baldor_meas_current_map_d.pdf",
    )
    plot_gn_map(
        current_map,
        "q",
        base,
        lims={"x": (0, 1), "y": (-1.5, 1.5), "z": (-4, 4)},
        ticks={"x": [0, 0.5, 1], "y": [-1.5, 0, 1.5], "z": [-4, -2, 0, 2, 4]},
        raw_data=[val_data, train_data],
        current_loci=True,
        current_loci_levels=current_loci_levels,
        latex=False,
        save_path=p / "figs" / "baldor_meas_current_map_q.pdf",
    )

    # %%
    # Compute and print statistical error metrics.

    print_meas_current_map_error_metrics(
        current_map_fcn, val_data, base=base, name="val"
    )

else:
    dataset_path = p / "datasets/baldor_fem.npz"
    trained_model_path = (
        p / "trained_models" / "baldor_fem_current_map_harm_squareplus_d48_sub10_.pth"
    )
    subsample = 10
    k = 6
    activation = gn.Squareplus
    # %%
    # Train the model.
    if not trained_model_path.exists():
        train_gradnet(
            dataset_path=dataset_path,
            base=base,
            save_model_path=trained_model_path,
            k=k,
            embed_dim=48,
            epochs=1000,
            subsample=subsample,
            activation=activation,
        )
    # %%
    # Load the dataset for visualization comparison.

    # Get the training and validation data (complement) from the helper function
    # Note: get_training_data returns (psi, i, ...), but we need (i, psi, ...)
    (trn_psi, trn_i, trn_theta, trn_tau), (val_psi, val_i, val_theta, val_tau) = (
        get_training_data(
            str(dataset_path),
            base=base,
            subsample=subsample,
            other_keys=["theta_m", "tau_m"],
        )
    )
    trn_data = (trn_i, trn_psi, trn_theta, trn_tau)
    val_data = (val_i, val_psi, val_theta, val_tau)

    # %%
    # Load the GradNet model and create its callable.

    model = gn.load_gradnet(trained_model_path, activation=activation)
    harm_map = gn.CurrentMapWithHarmonics(model, k=k)
    # %%
    # Compute and print statistical error metrics on validation data.

    val_dict = {
        "i_s_dq": val_i,
        "psi_s_dq": val_psi,
        "theta_m": val_theta,
        "tau_m": val_tau,
    }
    stat_fem_curr(map_fcn=harm_map, raw_data=val_dict, base=base)
