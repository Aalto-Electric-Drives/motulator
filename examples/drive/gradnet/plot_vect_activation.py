"""
Plot vector activation functions
================================

This example visualizes the softmax and p-norm gradient activation functions in two
dimensions.

"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch

from motulator.common.utils._plotting import (
    save_and_show,
    set_latex_style,
    set_screen_style,
)
from motulator.drive.utils import gn

# %%
# Initialize.

# Instantiate activation
activation = gn.Softmax(beta_log0=np.log(1), freeze_beta=True)
# activation = gn.PNormGradient(p=4, beta_log0=np.log(1), freeze_beta=True)

# Choose plotting style (LaTeX or screen)
use_latex = False

# %%
# Plot figure.

if use_latex:
    set_latex_style()
    width = plt.rcParams["figure.figsize"][0] * 1
    height = plt.rcParams["figure.figsize"][1] * 1
    figsize = (width, height)
    plt.rcParams.update({"savefig.pad_inches": 0.3})
else:
    set_screen_style()
    figsize = plt.rcParams["figure.figsize"]

surface_cmap = "viridis"
line_color = "k"

# 2D grid for illustration
x = np.linspace(-2, 2, 30)
y = np.linspace(-2, 2, 30)
X, Y = np.meshgrid(x, y)
Z = np.stack([X, Y], axis=-1)
Z_torch = torch.from_numpy(Z).float()

# Evaluate activation
sigma = activation(Z_torch)
sigma_x = sigma[..., 0].numpy()
sigma_y = sigma[..., 1].numpy()

# Plot surface for one component (e.g., sigma_x)
fig = plt.figure(figsize=figsize)
ax = fig.add_subplot(111, projection="3d")
surf = ax.plot_surface(X, Y, sigma_x, cmap=surface_cmap, alpha=0.5)
ax.plot_wireframe(X, Y, sigma_x, color=line_color, linewidth=0.5, alpha=0.5)
ax.set_xlabel("$z_1$", labelpad=-2)
ax.set_ylabel("$z_2$", labelpad=-2)
ax.zaxis.set_rotate_label(False)
ax.set_zlabel(r"$\sigma_1$", labelpad=-4, rotation=90)
if isinstance(activation, gn.Softmax):
    ax.view_init(elev=20, azim=-160)
elif isinstance(activation, gn.PNormGradient):
    ax.view_init(elev=20, azim=-160)
ax.set_xlim(-2, 2)
ax.set_xticks([-2, 0, 2])
ax.set_ylim(-2, 2)
ax.set_yticks([-2, 0, 2])
if isinstance(activation, gn.Softmax):
    ax.set_zlim(0, 1)
    ax.zaxis.set_ticks([0, 0.5, 1])
elif isinstance(activation, gn.PNormGradient):
    ax.set_zlim(-1, 1)
    ax.zaxis.set_ticks([-1, 0, 1])
# Fine tuning of tick label padding
ax.xaxis.set_tick_params(pad=2)
ax.yaxis.set_tick_params(pad=1)
ax.zaxis.set_tick_params(pad=-1)
plt.tight_layout()

p = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
save_path = Path(p / "figs" / "vectorial_activation.pdf")
save_path.parent.mkdir(parents=True, exist_ok=True)
save_and_show(save_path)
