"""Visualize flux linkage and current maps of synchronous machines."""

from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
from contourpy import contour_generator
from matplotlib.ticker import MaxNLocator
from numpy.typing import ArrayLike

from motulator.common.utils._plotting import (
    configure_axes,
    save_and_show,
    set_style,
    setup_map_plot,
)
from motulator.common.utils._utils import BaseValues
from motulator.drive.utils._sm_flux_maps import MagneticModel


# %%
def _get_xyz(
    data_type: str, i_s_dq: np.ndarray, psi_s_dq: np.ndarray, component: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get the map input (x, y) and the plotted component of the output (z)."""
    inp, out = (i_s_dq, psi_s_dq) if data_type == "flux_map" else (psi_s_dq, i_s_dq)
    return (
        np.real(inp),
        np.imag(inp),
        np.real(out) if component == "d" else np.imag(out),
    )


def _filter_and_plot_raw_data(
    ax,
    raw_data: MagneticModel | tuple[np.ndarray, ...],
    base: BaseValues,
    data_type: str,
    component: str,
    x_lims: tuple[float, float],
    y_lims: tuple[float, float],
    color: str,
) -> None:
    """Filter and plot raw data as scatter points."""
    if isinstance(raw_data, tuple):
        raw_psi_s_dq, raw_i_s_dq = raw_data[0], raw_data[1]
    else:
        raw_psi_s_dq, raw_i_s_dq = raw_data.psi_s_dq, raw_data.i_s_dq
    x, y, z = _get_xyz(
        data_type,
        np.asarray(raw_i_s_dq) / base.i,
        np.asarray(raw_psi_s_dq) / base.psi,
        component,
    )
    mask = (x >= x_lims[0]) & (x <= x_lims[1]) & (y >= y_lims[0]) & (y <= y_lims[1])
    ax.scatter(x[mask], y[mask], z[mask], s=5, marker=".", color=color)


def _plot_current_loci(
    ax,
    data: MagneticModel,
    base: BaseValues,
    component: str,
    levels: tuple[ArrayLike, ArrayLike],
    axlim_clip: bool,
) -> None:
    """Plot the constant-current loci, evaluating the map along them."""
    i_s_dq, psi_s_dq = data.i_s_dq / base.i, data.psi_s_dq / base.psi
    inp, in_base, out_base = (
        (i_s_dq, base.i, base.psi)
        if data.type == "flux_map"
        else (psi_s_dq, base.psi, base.i)
    )
    x, y = np.real(inp), np.imag(inp)
    fields = (np.real(i_s_dq), np.imag(i_s_dq))
    for field, field_levels in zip(fields, levels, strict=True):
        contours = contour_generator(x, y, field, line_type="Separate")
        for level in np.atleast_1d(field_levels):
            for line in contours.lines(float(level)):
                x_l, y_l = np.asarray(line).T
                out = np.asarray(data(in_base * (x_l + 1j * y_l))) / out_base
                z_l = np.real(out) if component == "d" else np.imag(out)
                ax.plot(
                    x_l,
                    y_l,
                    z_l,
                    color="black",
                    linewidth=0.5,
                    alpha=0.5,
                    axlim_clip=axlim_clip,
                )


def _get_labels(data_type: str, component: str, pu: bool) -> tuple[str, str, str]:
    """Get the axis labels of the map input (x, y) and the output component (z)."""
    i, psi = ("i", "A"), (r"\psi", "Vs")
    inp, out = (i, psi) if data_type == "flux_map" else (psi, i)

    def label(quantity: tuple[str, str], subscript: str) -> str:
        symbol, unit = quantity
        return rf"${symbol}_\mathrm{{{subscript}}}$ ({'p.u.' if pu else unit})"

    return label(inp, "d"), label(inp, "q"), label(out, component)


def _configure_3d_axes(
    ax,
    x: np.ndarray,
    y: np.ndarray,
    lims: dict[str, tuple[float, float]] | None,
    ticks: dict[str, ArrayLike] | None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Set the axis limits and ticks, the x and y limits from the data by default."""
    lims, ticks = lims or {}, ticks or {}
    x_lims = lims.get("x")
    x_lims = (np.min(x), np.max(x)) if x_lims is None else x_lims
    y_lims = lims.get("y")
    y_lims = (np.min(y), np.max(y)) if y_lims is None else y_lims
    ax.set_xlim(x_lims)
    ax.set_ylim(y_lims)
    if (z_lims := lims.get("z")) is not None:
        ax.set_zlim(z_lims)
    # By default, at most five intervals per axis
    for name, axis in zip("xyz", (ax.xaxis, ax.yaxis, ax.zaxis), strict=True):
        axis.set_major_locator(MaxNLocator(nbins=5, steps=[1, 2, 2.5, 5, 10]))
        if (axis_ticks := ticks.get(name)) is not None:
            axis.set_ticks(axis_ticks)
    return x_lims, y_lims


def plot_map(
    data: MagneticModel,
    component: Literal["d", "q"],
    base: BaseValues | None = None,
    lims: dict[str, tuple[float, float]] | None = None,
    ticks: dict[str, ArrayLike] | None = None,
    raw_data: MagneticModel
    | tuple[np.ndarray, ...]
    | list[MagneticModel | tuple[np.ndarray, ...]]
    | None = None,
    current_loci: tuple[ArrayLike, ArrayLike] | None = None,
    axlim_clip: bool = True,
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
        Base values for scaling the maps. If not given, the maps are plotted in SI
        units.
    lims : dict[str, tuple[float, float]], optional
        Axis limits. Keys should be 'x', 'y', 'z'.
    ticks : dict[str, ArrayLike], optional
        Axis tick locations. Keys should be 'x', 'y', 'z'.
    raw_data : MagneticModel | tuple | list[MagneticModel | tuple], optional
        Raw data for comparison, shown as scatter points (blue and red alternately for a
        list). A tuple of arrays is interpreted as `(psi_s_dq, i_s_dq, ...)` in SI
        units.
    current_loci : tuple[ArrayLike, ArrayLike], optional
        Levels `(i_d_levels, i_q_levels)` of the constant-current loci, in the units of
        the plot. If given, the loci are drawn on the surface instead of the wireframe.
        The map is evaluated along the loci by the lookup function of `data`.
    axlim_clip : bool, optional
        Whether to hide the parts of the surface and wireframe outside the axis view
        limits, defaults to True. The grid is cropped to the x and y limits, and the
        flag is passed to the `axlim_clip` argument of matplotlib's 3D plotting
        functions.
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    size = setup_map_plot(latex)
    pu = base is not None
    base = base if base is not None else BaseValues.unity()
    _, ax = plt.subplots(figsize=size, subplot_kw={"projection": "3d"})
    i_s_dq, psi_s_dq = data.i_s_dq / base.i, data.psi_s_dq / base.psi
    x, y, z = _get_xyz(data.type, i_s_dq, psi_s_dq, component)
    x_lims, y_lims = _configure_3d_axes(ax, x, y, lims, ticks)

    # Crop the grid to the axis limits, since clipping cuts the surface at its polygons
    inside = (x >= x_lims[0]) & (x <= x_lims[1]) & (y >= y_lims[0]) & (y <= y_lims[1])
    if axlim_clip and inside.any():
        rows = np.flatnonzero(inside.any(axis=1))
        cols = np.flatnonzero(inside.any(axis=0))
        crop = np.s_[rows[0] : rows[-1] + 1, cols[0] : cols[-1] + 1]
        x, y, z = x[crop], y[crop], z[crop]

    if raw_data is not None:
        datas = raw_data if isinstance(raw_data, list) else [raw_data]
        for k, raw in enumerate(datas):
            _filter_and_plot_raw_data(
                ax, raw, base, data.type, component, x_lims, y_lims, "br"[k % 2]
            )

    # Plot a transparent surface
    ax.plot_surface(  # type: ignore
        x, y, z, cmap="viridis", alpha=0.5, axlim_clip=axlim_clip
    )
    # Add constant-current loci or wireframe on top
    if current_loci is not None:
        _plot_current_loci(ax, data, base, component, current_loci, axlim_clip)
    else:
        ax.plot_wireframe(  # type: ignore
            x, y, z, color="black", linewidth=0.5, alpha=0.5, axlim_clip=axlim_clip
        )

    xlabel, ylabel, zlabel = _get_labels(data.type, component, pu)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.zaxis.set_rotate_label(False)  # type: ignore
    ax.set_zlabel(zlabel, rotation=90)  # type: ignore
    ax.view_init(elev=15, azim=-135 if component == "d" else -45)  # type: ignore

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
        Base values for scaling the maps. If not given, the maps are plotted in SI
        units.
    x_lims : tuple[float, float], optional
        x-axis limits. The q-axis curves are plotted at the d-axis currents closest to
        these limits, or at the extreme d-axis currents of the data if not given.
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
    set_style(latex)
    pu = base is not None
    base = base if base is not None else BaseValues.unity()
    i_s_dq, psi_s_dq = data.i_s_dq / base.i, data.psi_s_dq / base.psi

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

    configure_axes([ax], x_lims, x_ticks, [y_lims], [y_ticks])
    ax.set_xlabel(rf"$i_\mathrm{{d}}$, $i_\mathrm{{q}}$ ({'p.u.' if pu else 'A'})")
    ax.set_ylabel(
        rf"$\psi_\mathrm{{d}}$, $\psi_\mathrm{{q}}$ ({'p.u.' if pu else 'Vs'})"
    )
    ax.legend()

    save_and_show(save_path, **savefig_kwargs)
