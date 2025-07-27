"""Helper functions for plots."""

from pathlib import Path

import matplotlib.pyplot as plt
from cycler import cycler
from numpy.typing import ArrayLike


# %%
def set_screen_style() -> None:
    """Configure matplotlib for screen viewing."""
    plt.style.use("default")
    # plt.style.use("dark_background")
    plt.rcParams.update(
        {
            "axes.prop_cycle": cycler("color", ["b", "r", "m", "c"]),
            "lines.linewidth": 1,
            "axes.grid": True,
            "text.usetex": False,
            "axes3d.grid": True,
            "figure.constrained_layout.use": True,
        }
    )


# %%
def set_latex_style() -> None:
    """Configure matplotlib for LaTeX documents."""
    plt.style.use("default")
    plt.rcParams.update(
        {
            "axes.prop_cycle": cycler("color", ["b", "r", "m", "c"]),
            "lines.linewidth": 1,
            "axes.grid": True,
            # "axes.autolimit_mode": "round_numbers",
            "text.usetex": True,
            "text.latex.preamble": (r"\usepackage{newtxtext}\usepackage{newtxmath}"),
            "font.family": "serif",
            "font.serif": ["Liberation Serif"],
            "mathtext.fontset": "dejavuserif",
            "font.size": 8,
            "axes.labelsize": 8,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes3d.grid": True,
            "figure.figsize": [3.5, 2.5],
            # "figure.constrained_layout.use": True,
            # "savefig.bbox": "tight",
            # "savefig.pad_inches": 0.01,
            # Disable automatic adjustment
            "figure.constrained_layout.use": False,
            "figure.subplot.left": 0.18,  # Fixed margins
            "figure.subplot.right": 0.95,
            "figure.subplot.bottom": 0.18,
            "figure.subplot.top": 0.95,
        }
    )


# %%
def configure_axes(
    axes: list,
    t_lims: tuple[float, float] | None,
    t_ticks: ArrayLike | None,
    y_lims: list[tuple[float, float] | None] | None,
    y_ticks: list[ArrayLike | None] | None,
) -> None:
    """
    Configure time and y-axis settings for all subplots.

    Parameters
    ----------
    axes : list
        List of matplotlib axes objects to configure.
    t_lims : tuple[float, float] | None
        Time axis limits. If None, no limits are set.
    t_ticks : ArrayLike | None
        Time axis tick locations. If None, no ticks are set.
    y_lims : list[tuple[float, float] | None] | None
        y-axis limits for each subplot. Each element should be a tuple (y_min, y_max) or
        None for automatic scaling.
    y_ticks : list of ArrayLike or None
        y-axis tick locations for each subplot. Each element should be an array of tick
        positions or None for automatic ticks.

    """
    # Set time limits and ticks for all subplots
    for ax in axes:
        if t_lims is not None:
            ax.set_xlim(t_lims)
        if t_ticks is not None:
            ax.set_xticks(t_ticks)

    # Apply y-axis limits
    if y_lims is not None:
        for ax, ylim in zip(axes, y_lims, strict=False):
            if ylim is not None:
                ax.set_ylim(ylim)

    # Apply y-axis ticks
    if y_ticks is not None:
        for ax, ytick in zip(axes, y_ticks, strict=False):
            if ytick is not None:
                ax.set_yticks(ytick)


# %%
def save_and_show(save_path: str | Path | None = None, **savefig_kwargs) -> None:
    """
    Helper function to optionally save and show plots.

    Parameters
    ----------
    save_path : str | Path | None, optional
        Path to save the figure. If None, the figure is not saved.
    savefig_kwargs : dict, optional
        Additional keyword arguments for `plt.savefig`.

    """
    if save_path is not None:
        default_kwargs = {"bbox_inches": "tight"}
        default_kwargs.update(savefig_kwargs)
        plt.savefig(save_path, **default_kwargs)
    plt.show()
