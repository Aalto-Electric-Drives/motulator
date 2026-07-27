"""
5.6-kW PM-SyRM, train current map, FEM data with spatial harmonics
==================================================================

This example trains a GradNet current map for a four-pole 5.6-kW PM synchronous
reluctance machine (Baldor ECS101M0H7EF4) from a FEM dataset with spatial harmonics.

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
trained_path = p / "trained_models" / "baldor_fem_curr_map_harm_softmax_d48_sub10.pth"
subsample = 10
k = 6
activation = gn.Softmax

# %%
# Train the model.

if not trained_path.exists():
    gn.train_gradnet(
        dataset_path=dataset_path,
        base=base,
        save_model_path=trained_path,
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
    gn.get_training_data(
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

model = gn.load_gradnet(trained_path, activation=activation)
harm_map = gn.CurrentMapWithHarmonics(model, k=k)

# %%
# Print statistical error metrics on validation data.

val_dict = {
    "i_s_dq": val_i,
    "psi_s_dq": val_psi,
    "theta_m": val_theta,
    "tau_m": val_tau,
}
gn.print_current_map_errors_fem(map_fcn=harm_map, raw_data=val_dict, base=base)
