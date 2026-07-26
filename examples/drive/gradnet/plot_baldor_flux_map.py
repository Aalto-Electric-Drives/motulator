"""
Train flux map (5.6-kW PM-SyRM Baldor)
============================================

This script demonstrates how to train GradNet flux-linkage map from
a four-pole 5.6-kW PM synchronous reluctance machine (ABB Baldor ECS101M0H7EF4).
It includes loading a dataset, training a GradNet model,
and visualizing the trained model against the original dataset.
It can be run in following options:
1. Without spatial harmonics using measurement dataset.
2. Without spatial harmonics using FEM dataset, as a control model.
3. With spatial harmonics using FEM dataset, as a machine model.

"""

from pathlib import Path

import numpy as np

from motulator.drive import utils
from motulator.drive.utils import (
    PlotOptions,
    get_training_data,
    gn,
    plot_gn_map,
    plot_output_vs_angle,
    plot_surface_vs_current_and_angle,
    print_meas_flux_map_error_metrics,
    sample_map_on_grid,
    stat_fem,
    train_gradnet,
)

# %%
# Set nominal and base values.
nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Set up the paths and parameters.
p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
model_with_harmonics = False

# %%
# Train models
if not model_with_harmonics:
    # ## option 1.
    # dataset_path = p / "datasets/baldor_meas.npz"
    # trained_model_path = p / "trained_models/baldor_meas_flux_map_pnorm_d6_sub10_.pth"
    # subsample = 10

    ## option 2.
    dataset_path = p / "datasets/baldor_fem.npz"
    trained_model_path = p / "trained_models/baldor_fem_flux_map_pnorm_d12_sub20_.pth"
    subsample = 20

    activation = gn.PNormGradient

    # %%
    # Train the model (if enabled).
    if not trained_model_path.exists():
        train_gradnet(
            dataset_path=dataset_path,
            base=base,
            save_model_path=trained_model_path,
            is_flux_map=True,
            embed_dim=12,
            epochs=1000,
            subsample=subsample,
            activation=activation,
        )

    # %%
    # Create the GradNet model and its callable.
    model = gn.load_gradnet(trained_model_path, activation=activation)
    flux_map_fcn = gn.FluxMap(model)
    flux_map = sample_map_on_grid(
        flux_map_fcn,
        map_type="flux_map",
        d_range=np.linspace(-2, 2, 50) * base.i,
        q_range=np.linspace(-2.5, 2.5, 50) * base.i,
    )
    # Constant current contours corresponding to the measured dataset, for visualization
    i_d_levels = np.arange(-20, 22, 2) / base.i
    i_q_levels = np.arange(-26, 28, 2) / base.i
    current_loci_levels = (i_d_levels, i_q_levels)

    # %%
    # Load the dataset for comparison and split it into training and validation sets.
    train_data, val_data = get_training_data(
        str(dataset_path), base=base, subsample=subsample
    )

    # %%
    # Plot the flux map.
    plot_gn_map(
        flux_map,
        "d",
        base,
        current_loci=True,
        lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (0, 1)},
        ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2], "z": [0, 0.5, 1]},
        raw_data=[val_data, train_data],
        current_loci_levels=current_loci_levels,
        latex=False,
        save_path=p / "figs" / "baldor_meas_flux_map_d.pdf",
    )
    plot_gn_map(
        flux_map,
        "q",
        base,
        current_loci=True,
        lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (-1.5, 1.5)},
        ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2], "z": [-1.5, 0, 1.5]},
        raw_data=[val_data, train_data],
        current_loci_levels=current_loci_levels,
        latex=False,
        save_path=p / "figs" / "baldor_meas_flux_map_q.pdf",
    )

    # %%
    # Compute and print statistical error metrics.

    print_meas_flux_map_error_metrics(flux_map_fcn, val_data, base=base, name="val")
else:
    # %% Flux map with spatial harmonics for plot, trained on FEM data
    dataset_path = p / "datasets/baldor_fem.npz"
    trained_model_path = (
        p / "trained_models/baldor_fem_flux_map_harm_softmax_d48_sub10_.pth"
    )
    subsample = 10
    k = 6
    activation = gn.Softmax

    # %%
    # Train the model (if enabled).

    if not trained_model_path.exists():
        train_gradnet(
            dataset_path=dataset_path,
            base=base,
            save_model_path=trained_model_path,
            is_flux_map=True,
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
    harm_map = gn.FluxMapWithHarmonics(model, k=k)

    # %%
    # Try the model at single point.
    i_s_dq_mtpa = -9 + 9j  # Approximately the rated MTPA current
    theta_m = np.deg2rad(30)
    psi_s_dq, tau_m = harm_map(i_s_dq_mtpa, np.exp(1j * theta_m))
    print(f"Current: {i_s_dq_mtpa:.2f} A")
    print(f"Angle: {np.rad2deg(theta_m):.2f} deg")
    print(f"Flux linkage: {psi_s_dq:.2f} Vs")
    print(f"Torque per pole pair: {tau_m:.2f} Nm")

    # %%
    # Plot torque surface.

    # Define ranges for visualization
    i_q_range = np.linspace(0, 2 * base.i, 50)
    theta_m_range = np.linspace(0, 2 * np.pi / k, 50)
    # Plot torque as a function of i_q and theta_m at fixed i_d
    plot_surface_vs_current_and_angle(
        current_range=i_q_range,
        fixed_value=i_s_dq_mtpa.real,
        theta_m_range=theta_m_range,
        map_fcn=harm_map,
        input="i_q",
        output="tau_m",
        val_data=val_data,
        trn_data=trn_data,
        opts=PlotOptions(
            base=base,
            latex=False,
            lims={"x": (0, 2), "y": (0, 60), "z": (0, 1.5)},
            ticks={"x": [0, 1, 2], "y": [0, 30, 60], "z": [0, 0.5, 1.0, 1.5]},
            loci_levels_source="val",
            save_path=p / "figs" / "fem_torque_map_fixed_id.pdf",
        ),
    )

    # %%
    # Plot torque vs angle.

    theta_m_range = np.linspace(0, 2 * np.pi / k, 120)
    plot_output_vs_angle(
        fixed_value=i_s_dq_mtpa,
        theta_m_range=theta_m_range,
        map_fcn=harm_map,
        val_data=val_data,
        trn_data=trn_data,
        opts=PlotOptions(
            base=base,
            latex=False,
            lims={"x": (0, 60), "y": (0, 1)},
            ticks={"x": [0, 15, 30, 45, 60], "y": [0, 0.2, 0.4, 0.6, 0.8, 1.0]},
            save_path=p / "figs" / "fem_torque_vs_angle.pdf",
        ),
    )
    # Compute and print statistical error metrics on validation data.
    val_dict = {
        "i_s_dq": val_i,
        "psi_s_dq": val_psi,
        "theta_m": val_theta,
        "tau_m": val_tau,
    }
    stat_fem(map_fcn=harm_map, raw_data=val_dict, base=base)
