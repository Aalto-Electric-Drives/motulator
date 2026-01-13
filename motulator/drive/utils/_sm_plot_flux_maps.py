"""Visualize flux linkage and current maps of synchronous machines."""

from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike

from motulator.common.utils._plotting import (
    save_and_show,
    set_latex_style,
    set_screen_style,
)
from motulator.common.utils._utils import BaseValues
from motulator.drive.utils._sm_flux_maps import MagneticModel


# %%
def _setup_plot_style(latex: bool) -> tuple[tuple[float, float], bool]:
    """Setup plot style and return figure size and pu_vals flag."""
    if latex:
        set_latex_style()
        width = plt.rcParams["figure.figsize"][0] * 1.4
        height = plt.rcParams["figure.figsize"][1] * 1.4
        size = (width, height)
        plt.rcParams.update({"savefig.pad_inches": 0.3})
    else:
        set_screen_style()
        size = plt.rcParams["figure.figsize"]
    return size


# %%
def _filter_and_plot_raw_data(
    ax,
    raw_data: MagneticModel | tuple[np.ndarray, np.ndarray],
    base: BaseValues,
    data_type: str,
    component: str,
    x_lims: tuple[float, float],
    y_lims: tuple[float, float],
    marker: str,
    color: str,
) -> None:
    """Filter and plot raw data as scatter points."""
    if isinstance(raw_data, tuple):
        raw_i_s_dq = raw_data[0] / base.i
        raw_psi_s_dq = raw_data[1] / base.psi
    else:
        raw_i_s_dq = raw_data.i_s_dq / base.i
        raw_psi_s_dq = raw_data.psi_s_dq / base.psi

    if data_type == "flux_map":
        mask = (
            (raw_i_s_dq.real >= x_lims[0])
            & (raw_i_s_dq.real <= x_lims[1])
            & (raw_i_s_dq.imag >= y_lims[0])
            & (raw_i_s_dq.imag <= y_lims[1])
        )
        x_scatter, y_scatter = raw_i_s_dq[mask].real, raw_i_s_dq[mask].imag
        z_scatter = (
            raw_psi_s_dq[mask].real if component == "d" else raw_psi_s_dq[mask].imag
        )
    else:
        mask = (
            (raw_psi_s_dq.real >= x_lims[0])
            & (raw_psi_s_dq.real <= x_lims[1])
            & (raw_psi_s_dq.imag >= y_lims[0])
            & (raw_psi_s_dq.imag <= y_lims[1])
        )
        x_scatter, y_scatter = raw_psi_s_dq[mask].real, raw_psi_s_dq[mask].imag
        z_scatter = raw_i_s_dq[mask].real if component == "d" else raw_i_s_dq[mask].imag

    ax.scatter(x_scatter, y_scatter, z_scatter, marker=marker, color=color)


def _get_labels(data_type: str, component: str, pu_vals: bool) -> tuple[str, str, str]:
    """Get axis labels based on data type, component, and units."""
    if data_type == "flux_map":
        if pu_vals:
            xlabel = r"$i_\mathrm{d}$ (p.u.)"
            ylabel = r"$i_\mathrm{q}$ (p.u.)"
            zlabel = rf"$\psi_\mathrm{{{component}}}$ (p.u.)"
        else:
            xlabel = r"$i_\mathrm{d}$ (A)"
            ylabel = r"$i_\mathrm{q}$ (A)"
            zlabel = rf"$\psi_\mathrm{{{component}}}$ (Vs)"
    elif pu_vals:
        xlabel = r"$\psi_\mathrm{d}$ (p.u.)"
        ylabel = r"$\psi_\mathrm{q}$ (p.u.)"
        zlabel = rf"$i_\mathrm{{{component}}}$ (p.u.)"
    else:
        xlabel = r"$\psi_\mathrm{d}$ (Vs)"
        ylabel = r"$\psi_\mathrm{q}$ (Vs)"
        zlabel = rf"$i_\mathrm{{{component}}}$ (A)"
    return xlabel, ylabel, zlabel


def _configure_3d_axes(
    ax,
    data_type: str,
    i_s_dq: np.ndarray,
    psi_s_dq: np.ndarray,
    lims: dict[str, tuple[float, float]] | None = None,
    ticks: dict[str, ArrayLike] | None = None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Configure 3D axis limits and ticks."""
    # Extract limits or determine from data
    x_lims = lims.get("x") if lims else None
    y_lims = lims.get("y") if lims else None
    z_lims = lims.get("z") if lims else None

    if x_lims is None:
        if data_type == "flux_map":
            i_d_vals = np.real(i_s_dq)
            x_lims = (np.min(i_d_vals), np.max(i_d_vals))
        else:
            psi_d_vals = np.real(psi_s_dq)
            x_lims = (np.min(psi_d_vals), np.max(psi_d_vals))

    if y_lims is None:
        if data_type == "flux_map":
            i_q_vals = np.imag(i_s_dq)
            y_lims = (np.min(i_q_vals), np.max(i_q_vals))
        else:
            psi_q_vals = np.imag(psi_s_dq)
            y_lims = (np.min(psi_q_vals), np.max(psi_q_vals))

    # Set axis limits
    ax.set_xlim(x_lims)
    ax.set_ylim(y_lims)
    if z_lims is not None:
        ax.set_zlim(z_lims)  # type: ignore

    # Set axis ticks
    if ticks is not None:
        x_ticks = ticks.get("x")
        y_ticks = ticks.get("y")
        z_ticks = ticks.get("z")

        if x_ticks is not None:
            ax.set_xticks(x_ticks)
        if y_ticks is not None:
            ax.set_yticks(y_ticks)
        if z_ticks is not None:
            ax.set_zticks(z_ticks)  # type: ignore

    return x_lims, y_lims


def plot_map(
    data: MagneticModel,
    component: Literal["d", "q"],
    base: BaseValues | None = None,
    lims: dict[str, tuple[float, float]] | None = None,
    ticks: dict[str, ArrayLike] | None = None,
    raw_data: MagneticModel
    | list[MagneticModel | tuple[np.ndarray, np.ndarray]]
    | None = None,
    raw_marker: str | list[str] | None = None,
    raw_color: str | list[str] | None = None,
    axlim_clip: bool = True,
    surface_cmap: str = "viridis",
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot component (d or q) of flux linkage or current maps.

    Parameters
    ----------
    data : MagneticModel
        Data containing the flux and current information.
    component : {"d", "q"}
        Which component to plot: "d" for d-axis, "q" for q-axis.
    base : BaseValues, optional
        Base values for scaling the maps. If not given, the maps are plotted
        in SI units.
    lims : dict[str, tuple[float, float]], optional
        Axis limits. Keys should be 'x', 'y', 'z'.
    ticks : dict[str, ArrayLike], optional
        Axis tick locations. Keys should be 'x', 'y', 'z'.
    raw_data : MagneticModel | list[MagneticModel], optional
        Raw data for comparison (shown as scatter points).
    raw_marker : str | list[str], optional
        Marker style for raw data.
    raw_color : str | list[str], optional
        Marker color for raw data.
    axlim_clip : bool, optional
        Whether to clip the axes limits to the data limits, defaults to True.
    surface_cmap : str, optional
        Colormap for the surface plot, defaults to "viridis".
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    # Setup style and base values
    size = _setup_plot_style(latex)
    pu_vals = base is not None
    if base is None:
        base = BaseValues.unity()

    # Normalize data
    i_s_dq = data.i_s_dq / base.i
    psi_s_dq = data.psi_s_dq / base.psi

    # Create figure and axis
    fig, ax = plt.subplots(figsize=size, subplot_kw={"projection": "3d"})

    # Select plot data based on map type
    if data.type == "flux_map":
        x, y = i_s_dq.real, i_s_dq.imag
        z = psi_s_dq.real if component == "d" else psi_s_dq.imag
    else:
        x, y = psi_s_dq.real, psi_s_dq.imag
        z = i_s_dq.real if component == "d" else i_s_dq.imag

    # Configure axes (this determines x_lims, y_lims internally)
    x_lims, y_lims = _configure_3d_axes(ax, data.type, i_s_dq, psi_s_dq, lims, ticks)

    # Plot raw data if provided
    if raw_data is not None:
        if isinstance(raw_data, list):
            datas = raw_data
            def_markers = [".", "."]
            def_colors = ["b", "r"]
        else:
            datas = [raw_data]
            def_markers = ["."]
            def_colors = ["b"]

        markers = raw_marker if raw_marker is not None else def_markers
        colors = raw_color if raw_color is not None else def_colors

        # Ensure markers and colors are lists
        if isinstance(markers, str):
            markers = [markers] * len(datas)
        if isinstance(colors, str):
            colors = [colors] * len(datas)

        # Plot each raw dataset
        for i, raw_d in enumerate(datas):
            m = markers[i % len(markers)]
            c = colors[i % len(colors)]
            _filter_and_plot_raw_data(
                ax,
                raw_d,
                base,
                data.type,
                component,
                x_lims,
                y_lims,
                marker=str(m),
                color=str(c),
            )

    # Plot a transparent surface
    ax.plot_surface(  # type: ignore
        x, y, z, cmap=surface_cmap, alpha=0.5, axlim_clip=axlim_clip
    )
    # Add wireframe on top
    ax.plot_wireframe(  # type: ignore
        x, y, z, color="black", linewidth=0.5, alpha=0.5, axlim_clip=axlim_clip
    )

    xlabel, ylabel, zlabel = _get_labels(data.type, component, pu_vals)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.zaxis.set_rotate_label(False)  # type: ignore

    # Set view angle based on component
    if component == "d":
        ax.view_init(elev=15, azim=-135)  # type: ignore
        ax.set_zlabel(zlabel, rotation=90)  # type: ignore
    else:
        ax.view_init(elev=15, azim=-45)  # type: ignore
        ax.set_zlabel(zlabel, rotation=90)  # type: ignore

    save_and_show(save_path, **savefig_kwargs)


# %%
def plot_flux_vs_current(
    data: MagneticModel,
    base: BaseValues | None = None,
    x_lims: tuple[float, float] | None = None,
    x_ticks: ArrayLike | None = None,
    y_lims: tuple[float, float] | None = None,
    y_ticks: ArrayLike | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot the flux vs. current characteristics.

    Parameters
    ----------
    data : MagneticModel
        Flux map data. The current array should be a rectilinear grid.
    base : BaseValues, optional
        Base values for scaling the maps. If not given, the maps are plotted
        in SI units.
    x_lims : tuple[float, float], optional
        x-axis limits. If None, uses automatic scaling.
    x_ticks : ArrayLike, optional
        x-axis tick locations.
    y_lims : tuple[float, float], optional
        y-axis limits. If None, uses automatic scaling.
    y_ticks : ArrayLike, optional
        y-axis tick locations.
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    if latex:
        set_latex_style()
    else:
        set_screen_style()

    # Check if the base values were given
    pu_vals = base is not None
    if base is None:
        base = BaseValues.unity()

    # Normalize the data
    i_s_dq = data.i_s_dq / base.i
    psi_s_dq = data.psi_s_dq / base.psi

    # Indices corresponding to i_d = 0 and i_q = 0
    ind_d_0 = np.argmin(np.abs(i_s_dq.real[:, 0]))
    ind_q_0 = np.argmin(np.abs(i_s_dq.imag[0, :]))

    # Indices corresponding to min(i_d) and max(i_d)
    if x_lims is None:
        ind_d_min = np.argmin(i_s_dq.real[:, 0])
        ind_d_max = np.argmax(i_s_dq.real[:, 0])
    else:
        ind_d_min = np.argmin(np.abs(i_s_dq.real[:, 0] - x_lims[0]))
        ind_d_max = np.argmin(np.abs(i_s_dq.real[:, 0] - x_lims[1]))

    fig, ax = plt.subplots()
    ax.plot(
        i_s_dq.real[:, ind_q_0],
        psi_s_dq.real[:, ind_q_0],
        color="r",
        linestyle="-",
        label=r"$\psi_\mathrm{d}(i_\mathrm{d}, 0)$",
    )
    ax.plot(
        i_s_dq.real[:, -1],
        psi_s_dq.real[:, -1],
        color="r",
        linestyle="--",
        label=r"$\psi_\mathrm{d}(i_\mathrm{d}, i_\mathrm{q}^\mathrm{max})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_0, :],
        psi_s_dq.imag[ind_d_0, :],
        color="b",
        linestyle="-",
        label=r"$\psi_\mathrm{q}(0, i_\mathrm{q})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_min, :],
        psi_s_dq.imag[ind_d_min, :],
        color="b",
        linestyle=":",
        label=r"$\psi_\mathrm{q}(i_\mathrm{d}^\mathrm{min}, i_\mathrm{q})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_max, :],
        psi_s_dq.imag[ind_d_max, :],
        color="b",
        linestyle="--",
        label=r"$\psi_\mathrm{q}(i_\mathrm{d}^\mathrm{max}, i_\mathrm{q})$",
    )

    # Set axis limits and ticks
    if x_lims is not None:
        ax.set_xlim(x_lims)
    if x_ticks is not None:
        ax.set_xticks(x_ticks)

    if y_lims is not None:
        ax.set_ylim(y_lims)
    if y_ticks is not None:
        ax.set_yticks(y_ticks)

    # Set axis labels
    if pu_vals:
        ax.set_xlabel(r"$i_\mathrm{d}$, $i_\mathrm{q}$ (p.u.)")
        ax.set_ylabel(r"$\psi_\mathrm{d}$, $\psi_\mathrm{q}$ (p.u.)")
    else:
        ax.set_xlabel(r"$i_\mathrm{d}$, $i_\mathrm{q}$ (A)")
        ax.set_ylabel(r"$\psi_\mathrm{d}$, $\psi_\mathrm{q}$ (Vs)")
    ax.legend()

    save_and_show(save_path, **savefig_kwargs)
