"""
5.6-kW PM-SyRM, train flux map, measured data
=============================================

This example trains a GradNet flux-linkage map for a four-pole 5.6-kW PM synchronous
reluctance machine (Baldor ECS101M0H7EF4) from a measured dataset.

"""

from pathlib import Path

import numpy as np

import motulator.drive.gradnet as gn
from motulator.drive import utils

# %%
# Set nominal and base values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Set up the paths and parameters.

p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()

dataset_path = p / "datasets/baldor_meas.npz"
trained_path = p / "trained_models/baldor_meas_flux_map_pnorm_d6_sub10.pth"
subsample = 10
activation = gn.PNormGradient

# %%
# Train the model.

if not trained_path.exists():
    gn.train_gradnet(
        dataset_path=dataset_path,
        base=base,
        save_model_path=trained_path,
        is_flux_map=True,
        embed_dim=6,
        epochs=20000,
        subsample=subsample,
        activation=activation,
    )

# %%
# Create the GradNet model and its callable.

model = gn.load_gradnet(trained_path, activation=activation)
flux_map_fcn = gn.FluxMap(model)

# %%
# Load the dataset for comparison and split it into training and validation sets.

train_data, val_data = gn.get_training_data(
    str(dataset_path), base=base, subsample=subsample
)

# %%
# Plot the flux map.

# Sample the map on a grid for plotting
flux_map = gn.sample_map_on_grid(
    flux_map_fcn,
    map_type="flux_map",
    d_range=np.linspace(-2, 2, 50) * base.i,
    q_range=np.linspace(-2.5, 2.5, 50) * base.i,
)

# Constant current contours corresponding to the measured dataset, for visualization
i_d_levels = np.arange(-20, 22, 2) / base.i
i_q_levels = np.arange(-26, 28, 2) / base.i
current_loci_levels = (i_d_levels, i_q_levels)

gn.plot_maps(
    flux_map,
    "d",
    base,
    current_loci=True,
    lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (0, 1)},
    ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2], "z": [0, 0.5, 1]},
    raw_data=[val_data, train_data],
    current_loci_levels=current_loci_levels,
)

gn.plot_maps(
    flux_map,
    "q",
    base,
    current_loci=True,
    lims={"x": (-2, 2), "y": (-2.5, 2.5), "z": (-1.5, 1.5)},
    ticks={"x": [-2, -1, 0, 1, 2], "y": [-2, -1, 0, 1, 2], "z": [-1.5, 0, 1.5]},
    raw_data=[val_data, train_data],
    current_loci_levels=current_loci_levels,
)

# %%
# Print error metrics.

gn.print_flux_map_errors_meas(flux_map_fcn, val_data, base=base)
