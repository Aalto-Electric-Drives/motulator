"""
5.6-kW PM-SyRM, train flux map, FEM data, no spatial harmonics
==============================================================

This example trains a GradNet flux-linkage map for a four-pole 5.6-kW PM synchronous
reluctance machine (Baldor ECS101M0H7EF4) from a FEM dataset without spatial harmonics.

"""

from pathlib import Path

import motulator.drive.gradnet as gn
from motulator.drive import utils

# %%
# Set nominal and base values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Set up the paths and parameters.

p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()

dataset_path = p / "datasets/baldor_fem.npz"
trained_path = p / "trained_models/baldor_fem_flux_map_pnorm_d12_sub20.pth"
subsample = 20

activation = gn.PNormGradient

# %%
# Train the model.

if not trained_path.exists():
    gn.train_gradnet(
        dataset_path=dataset_path,
        base=base,
        save_model_path=trained_path,
        is_flux_map=True,
        embed_dim=12,
        epochs=1000,
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
# Print statistical error metrics.

gn.print_flux_map_errors_meas(flux_map_fcn, val_data, base=base)
