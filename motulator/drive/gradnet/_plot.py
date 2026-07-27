"""Plotting utilities for GradNet examples."""

# ruff: noqa: PLR0912, PLR0915, E501
# pylint: disable=too-many-branches,too-many-statements

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Literal

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import tri as mtri
from numpy.typing import NDArray

from motulator.common.utils._plotting import (
    save_and_show,
    set_latex_style,
    set_screen_style,
)
from motulator.common.utils._utils import BaseValues

LINE_ALPHA = 0.7


# %%
@dataclass
class MapGrid:
    """Sampled rectilinear-grid map data used for plotting in GradNet examples."""

    psi_s_dq: NDArray[np.complexfloating]
    i_s_dq: NDArray[np.complexfloating]
    map_type: Literal["current_map", "flux_map"]


def sample_map_on_grid(
    map_fcn: Callable[[complex | np.ndarray], complex | np.ndarray],
    map_type: Literal["current_map", "flux_map"],
    d_range: np.ndarray,
    q_range: np.ndarray,
) -> MapGrid:
    """
    Sample a (measured or learned) map callable on a rectilinear grid.

    For `map_type="current_map"`, (`d_range`, `q_range`) are in (`psi_d`, `psi_q`)
    units. For `map_type="flux_map"`, (`d_range`, `q_range`) are in (`i_d`, `i_q`)
    units.

    Parameters
    ----------
    map_fcn : Callable[[complex | ndarray], complex | ndarray]
        Callable map function.
    map_type : {"current_map", "flux_map"}
        Type of the map.
    d_range : ndarray
        Range of values for the d-axis.
    q_range : ndarray
        Range of values for the q-axis.

    Returns
    -------
    MapGrid
        Sampled grid data.

    """
    d_range = np.asarray(d_range, dtype=float)
    q_range = np.asarray(q_range, dtype=float)
    d_grid, q_grid = np.meshgrid(d_range, q_range, indexing="ij")
    inp = (d_grid + 1j * q_grid).astype(np.complex128)
    out = np.asarray(map_fcn(inp.ravel()), dtype=np.complex128).reshape(inp.shape)

    if map_type == "current_map":
        psi_s_dq, i_s_dq = inp, out
    else:
        i_s_dq, psi_s_dq = inp, out

    return MapGrid(psi_s_dq=psi_s_dq, i_s_dq=i_s_dq, map_type=map_type)


# %%
def plot_maps(
    data: MapGrid,
    component: Literal["d", "q"],
    base: BaseValues | None = None,
    lims: dict[str, tuple[float, float]] | None = None,
    ticks: dict[str, list[float]] | None = None,
    raw_data: tuple[np.ndarray, ...] | list[tuple[np.ndarray, ...]] | None = None,
    surface_cmap: str = "viridis",
    current_loci: bool = False,
    current_loci_levels: Any | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs: Any,
) -> None:  # noqa: PLR0912, PLR0915
    """
    Plot flux linkage or current maps.

    Parameters
    ----------
    data : MapGrid
        Sampled rectilinear-grid map data.
    component : {"d", "q"}
        Component of the flux linkage or current to plot.
    base : BaseValues | None, optional
        Base values for per-unit conversion. If None, unity base values are used.
    lims : dict[str, tuple[float, float]] | None, optional
        Axis limits as {'x': (xmin, xmax), 'y': (ymin, ymax), 'z': (zmin, zmax)}.
    ticks : dict[str, list[float]] | None, optional
        Axis ticks as {'x': [x1, x2, ...], 'y': [y1, y2, ...], 'z': [z1, z2, ...]}.
    raw_data : tuple | list[tuple] | None, optional
        Raw validation/training data scatter overlays.
    surface_cmap : str, optional
        Colormap for the surface plot, defaults to "viridis".
    current_loci : bool, optional
        Plot constant-current loci overlays, defaults to False.
    current_loci_levels : list | ndarray | tuple | None, optional
        Levels for the constant-current loci overlays. If list or array, the same
        levels are used for `i_d` and `i_q`. If a tuple of two lists/arrays,
        `(i_d_levels, i_q_levels)`. Defaults to None.
    latex : bool, optional
        Use LaTeX fonts if True, defaults to False.
    save_path : str | Path | None, optional
        Path to save the figure, defaults to None (not saved).
    savefig_kwargs : Any
        Additional keyword arguments passed to `plt.savefig()`.

    """
    width, height = setup_plot_style(latex)
    pu_vals = base is not None
    if base is None:
        base = BaseValues.unity()

    # Normalize to p.u. (or unity)
    i_s_dq = np.asarray(data.i_s_dq) / base.i
    psi_s_dq = np.asarray(data.psi_s_dq) / base.psi

    fig = plt.figure(figsize=(width, height))
    ax = fig.add_subplot(111, projection="3d")

    if data.map_type == "flux_map":
        x, y = i_s_dq.real, i_s_dq.imag
        z = psi_s_dq.real if component == "d" else psi_s_dq.imag
        xlabel = r"$i_\mathrm{d}$ (p.u.)" if pu_vals else r"$i_\mathrm{d}$ (A)"
        ylabel = r"$i_\mathrm{q}$ (p.u.)" if pu_vals else r"$i_\mathrm{q}$ (A)"
        zlabel = (
            r"$\psi_\mathrm{d}$ (p.u.)"
            if (component == "d" and pu_vals)
            else r"$\psi_\mathrm{d}$ (Vs)"
            if component == "d"
            else r"$\psi_\mathrm{q}$ (p.u.)"
            if pu_vals
            else r"$\psi_\mathrm{q}$ (Vs)"
        )
    else:
        x, y = psi_s_dq.real, psi_s_dq.imag
        z = i_s_dq.real if component == "d" else i_s_dq.imag
        xlabel = r"$\psi_\mathrm{d}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{d}$ (Vs)"
        ylabel = r"$\psi_\mathrm{q}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{q}$ (Vs)"
        if component == "d":
            zlabel = r"$i_\mathrm{d}$ (p.u.)" if pu_vals else r"$i_\mathrm{d}$ (A)"
        else:
            zlabel = r"$i_\mathrm{q}$ (p.u.)" if pu_vals else r"$i_\mathrm{q}$ (A)"

    # Surface and wireframe
    ax.plot_surface(x, y, z, cmap=surface_cmap, alpha=0.5)

    # Constant-current loci overlays: i_d=const and i_q=const.
    # For flux maps, axes are (i_d, i_q) so these are surface intersection lines.
    # For current maps, axes are (psi_d, psi_q) so these are contour families of i_d/i_q
    # projected onto the (psi_d, psi_q) plane and lifted to the 3D surface.
    if current_loci:
        id_levs, iq_levs = None, None

        if isinstance(current_loci_levels, tuple) and len(current_loci_levels) == 2:
            id_levs, iq_levs = current_loci_levels
        elif current_loci_levels is not None:
            id_levs = iq_levs = current_loci_levels

        def _default_levels_for_axis(axis: Literal["x", "y", "z"]) -> list[float]:
            if ticks is not None and ticks.get(axis) is not None:
                return list(ticks[axis])
            if lims is not None and lims.get(axis) is not None:
                amin, amax = lims[axis]
                return np.linspace(amin, amax, 5).tolist()
            arr = x if axis == "x" else y if axis == "y" else z
            return np.linspace(np.nanmin(arr), np.nanmax(arr), 5).tolist()

        # Choose sensible defaults depending on how currents appear in this plot
        if data.map_type == "flux_map":
            # Currents are the x/y axes.
            if id_levs is None:
                id_levs = _default_levels_for_axis("x")
            if iq_levs is None:
                iq_levs = _default_levels_for_axis("y")

            # x and y are rectilinear grids produced by meshgrid with indexing="ij"
            d_vals = np.asarray(x[:, 0], dtype=float)
            q_vals = np.asarray(y[0, :], dtype=float)

            def _interp_1d(x0: float, xp: np.ndarray, fp: np.ndarray) -> float:
                mask = np.isfinite(fp)
                if mask.sum() < 2:
                    return float("nan")
                return float(np.interp(x0, xp[mask], fp[mask]))

            def _plot_id_slice(id0: float) -> None:
                if id0 < float(np.nanmin(d_vals)) or id0 > float(np.nanmax(d_vals)):
                    return
                zs = np.array(
                    [_interp_1d(id0, d_vals, z[:, k]) for k in range(z.shape[1])]
                )
                mask = np.isfinite(zs)
                if mask.sum() >= 2:
                    ax.plot(
                        np.full_like(q_vals[mask], id0),
                        q_vals[mask],
                        zs[mask],
                        color="black",
                        linewidth=0.5,
                        alpha=LINE_ALPHA,
                    )

            def _plot_iq_slice(iq0: float) -> None:
                if iq0 < float(np.nanmin(q_vals)) or iq0 > float(np.nanmax(q_vals)):
                    return
                zs = np.array(
                    [_interp_1d(iq0, q_vals, z[k, :]) for k in range(z.shape[0])]
                )
                mask = np.isfinite(zs)
                if mask.sum() >= 2:
                    ax.plot(
                        d_vals[mask],
                        np.full_like(d_vals[mask], iq0),
                        zs[mask],
                        color="black",
                        linewidth=0.5,
                        alpha=LINE_ALPHA,
                    )

            for lev in id_levs:
                _plot_id_slice(float(lev))
            for lev in iq_levs:
                _plot_iq_slice(float(lev))

        elif data.map_type == "current_map":
            # Currents are the z-values; contours are defined in the (x,y) plane.
            if id_levs is None:
                id_levs = _default_levels_for_axis("z")
            if iq_levs is None:
                iq_levs = _default_levels_for_axis("z")

            tri = mtri.Triangulation(x.ravel(), y.ravel())
            z_interp = mtri.LinearTriInterpolator(tri, z.ravel())
            xy = np.column_stack((x.ravel(), y.ravel()))
            z_flat = np.asarray(z.ravel())

            def _fill_masked_with_nearest(
                xs: np.ndarray, ys: np.ndarray, zs: Any
            ) -> np.ndarray:
                """Fill masked/NaN interpolated values using nearest neighbor."""
                # Matplotlib's `LinearTriInterpolator` masks points outside the
                # triangulation hull, which would otherwise truncate the overlay line
                # when we drop masked samples.
                if np.ma.is_masked(zs):
                    z_arr = np.asarray(np.ma.filled(zs, np.nan), dtype=float)
                else:
                    z_arr = np.asarray(zs, dtype=float)

                bad = ~np.isfinite(z_arr)
                if not bad.any():
                    return z_arr

                bad_idx = np.where(bad)[0]
                for j in bad_idx:
                    dx = xy[:, 0] - xs[j]
                    dy = xy[:, 1] - ys[j]
                    nn = int(np.argmin(dx * dx + dy * dy))
                    z_arr[j] = float(z_flat[nn])
                return z_arr

            def _plot_family(field: np.ndarray, is_z: bool, levels: Any) -> None:
                tmp_fig = plt.figure()
                tmp_ax = tmp_fig.add_subplot(111)
                cs = tmp_ax.contour(x, y, field, levels=levels)
                allsegs, lvls = cs.allsegs, cs.levels
                plt.close(tmp_fig)

                for lev, segs in zip(lvls, allsegs, strict=True):
                    for seg in segs:
                        xs, ys = seg[:, 0], seg[:, 1]
                        if is_z:
                            zs = np.full_like(xs, lev)
                        else:
                            zs = _fill_masked_with_nearest(xs, ys, z_interp(xs, ys))
                            finite = np.isfinite(zs)
                            xs, ys, zs = xs[finite], ys[finite], zs[finite]
                        if xs.size >= 2:
                            ax.plot(
                                xs,
                                ys,
                                zs,
                                color="black",
                                linewidth=0.5,
                                alpha=LINE_ALPHA,
                            )

            _plot_family(i_s_dq.real, component == "d", id_levs)
            _plot_family(i_s_dq.imag, component == "q", iq_levs)

    # Raw scatter
    if raw_data is not None:
        datasets = raw_data if isinstance(raw_data, list) else [raw_data]
        markers = [".", "*"]
        colors = ["b", "r"]
        for k, raw in enumerate(datasets):
            psi_raw, i_raw = raw[0], raw[1]
            psi_raw_pu = np.asarray(psi_raw) / base.psi
            i_raw_pu = np.asarray(i_raw) / base.i
            if data.map_type == "flux_map":
                xr, yr = i_raw_pu.real, i_raw_pu.imag
                zr = psi_raw_pu.real if component == "d" else psi_raw_pu.imag
            else:
                xr, yr = psi_raw_pu.real, psi_raw_pu.imag
                zr = i_raw_pu.real if component == "d" else i_raw_pu.imag
            ax.scatter(
                xr,
                yr,
                zr,  # type: ignore
                marker=markers[k % len(markers)],
                color=colors[k % len(colors)],
                depthshade=False,
                alpha=1.0,
                s=5,
            )

    # Limits and ticks
    if lims is not None:
        if "x" in lims:
            ax.set_xlim(lims["x"])
        if "y" in lims:
            ax.set_ylim(lims["y"])
        if "z" in lims:
            ax.set_zlim(lims["z"])  # type: ignore
    if ticks is not None:
        if "x" in ticks:
            ax.set_xticks(ticks["x"])
        if "y" in ticks:
            ax.set_yticks(ticks["y"])
        if "z" in ticks:
            ax.set_zticks(ticks["z"])  # type: ignore

    ax.xaxis.set_tick_params(pad=2)
    ax.yaxis.set_tick_params(pad=2)
    ax.zaxis.set_tick_params(pad=-1)

    ax.set_xlabel(xlabel, labelpad=1)
    ax.set_ylabel(ylabel, labelpad=1)
    ax.zaxis.set_rotate_label(False)  # type: ignore
    ax.set_zlabel(zlabel, rotation=90, labelpad=-5)  # type: ignore

    if component == "d":
        ax.view_init(elev=15, azim=-135)  # type: ignore
    else:
        ax.view_init(elev=15, azim=-45)  # type: ignore

    save_and_show(save_path, **savefig_kwargs)


# %%
def setup_plot_style(latex: bool) -> tuple[float, float]:
    """
    Setup plot style and return figure dimensions.

    Parameters
    ----------
    latex : bool
        Use LaTeX fonts.

    Returns
    -------
    tuple
        Figure width and height.

    """
    if latex:
        set_latex_style()
        width = plt.rcParams["figure.figsize"][0] * 1.4
        height = plt.rcParams["figure.figsize"][1] * 1.4
        plt.rcParams.update({"savefig.pad_inches": 0.3})
    else:
        set_screen_style()
        width, height = plt.rcParams["figure.figsize"]
    return width, height


# %%
@dataclass
class PlotOptions:
    """
    Options for plotting functions.

    Parameters
    ----------
    base : BaseValues | None, optional
        Base values for per-unit conversion. If None, SI values are used.
    lims : dict[str, tuple[float, float]] | None, optional
        Axis limits as {'x': (xmin, xmax), 'y': (ymin, ymax)}.
    ticks : dict[str, list[float]] | None, optional
        Axis ticks as {'x': [x1, x2, ...], 'y': [y1, y2, ...]}.
    surface_cmap : str, optional
        Colormap for the surface plot, defaults to "viridis".
    latex : bool, optional
        Use LaTeX fonts if True, defaults to False.
    save_path : str | Path | None, optional
        Path to save the figure. If None, the figure is not saved.
    savefig_kwargs : Dict[str, Any], optional
        Additional keyword arguments passed to plt.savefig().

    loci_levels_source : {"ticks", "val", "trn"}, optional
        Source for constant-loci levels in `plot_surface_vs_current_and_angle()` when
        plotting over current and angle. If "ticks", uses `ticks["x"]`/`ticks["y"]`
        (with lims/fallback defaults). If "val" or "trn", uses unique x/y values
        present in the corresponding point cloud slice.

    """

    base: BaseValues | None = None
    lims: dict[str, tuple[float, float]] | None = None
    ticks: dict[str, list[float]] | None = None
    surface_cmap: str = "viridis"
    latex: bool = False
    save_path: str | Path | None = None
    savefig_kwargs: Dict[str, Any] = field(default_factory=dict)
    loci_levels_source: Literal["ticks", "val", "trn"] = "ticks"


# %%
def extract_surface_plot_points(
    data: tuple | None,
    input: Literal["i_d", "i_q", "psi_d", "psi_q"],
    d_exact: float | None,
    q_exact: float | None,
    output: Literal["psi_d", "psi_q", "tau_m", "i_d", "i_q"],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Helper to extract points matching the fixed input value."""
    if data is None:
        return np.array([]), np.array([]), np.array([])
    # Check data length to handle cases without theta_m and tau_m
    if len(data) == 4:
        i_s_dq, psi_s_dq, theta_m, tau_m = data
    elif len(data) == 2:
        # Assume data is (psi_s_dq, i_s_dq), only for flux maps case
        psi_s_dq, i_s_dq = data
        theta_m = np.zeros_like(np.real(i_s_dq))  # Dummy theta
        tau_m = np.zeros_like(np.real(i_s_dq))  # Dummy tau
    else:
        raise ValueError(f"Unexpected data tuple length: {len(data)}")

    if input in ("i_d", "i_q"):
        source = i_s_dq
    else:
        source = psi_s_dq

    if input in ("i_d", "psi_d"):
        # Varying d, fixed q
        assert q_exact is not None
        # Using a small tolerance (e.g. 1e-5) is safer than exact equality for float
        mask = np.isclose(source.imag, q_exact, atol=1e-5)
        x = source.real[mask]
    else:
        # Varying q, fixed d
        assert d_exact is not None
        mask = np.isclose(source.real, d_exact, atol=1e-5)
        x = source.imag[mask]

    y = theta_m[mask]
    if output == "tau_m":
        z = tau_m[mask]
    elif output in ("psi_d", "psi_q"):
        z = psi_s_dq.real[mask] if output == "psi_d" else psi_s_dq.imag[mask]
    else:
        z = i_s_dq.real[mask] if output == "i_d" else i_s_dq.imag[mask]
    return x, y, z


# %%
def _get_base_values(opts: PlotOptions) -> tuple[BaseValues, bool]:
    """Return base values and whether per-unit plotting is enabled."""
    if opts.base is None:
        return BaseValues.unity(), False
    return opts.base, True


# %%
def _surface_plot_labels(
    input: Literal["i_d", "i_q", "psi_d", "psi_q"],
    output: Literal["psi_d", "psi_q", "tau_m", "i_d", "i_q"],
    pu_vals: bool,
) -> tuple[str, str]:
    """Generate axis labels for surface plots (flux or torque)."""
    if input == "i_d":
        xlabel = r"$i_\mathrm{d}$ (p.u.)" if pu_vals else r"$i_\mathrm{d}$ (A)"
    elif input == "i_q":
        xlabel = r"$i_\mathrm{q}$ (p.u.)" if pu_vals else r"$i_\mathrm{q}$ (A)"
    elif input == "psi_d":
        xlabel = r"$\psi_\mathrm{d}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{d}$ (Vs)"
    else:
        xlabel = r"$\psi_\mathrm{q}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{q}$ (Vs)"

    if output == "tau_m":
        zlabel = (
            r"$\tau_\mathrm{m}$ (p.u.)"
            if pu_vals
            else r"$\tau_\mathrm{M}/n_\mathrm{p}$ (Nm)"
        )
    elif output == "psi_d":
        zlabel = r"$\psi_\mathrm{d}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{d}$ (Vs)"
    elif output == "psi_q":
        zlabel = r"$\psi_\mathrm{q}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{q}$ (Vs)"
    elif output == "i_d":
        zlabel = r"$i_\mathrm{d}$ (p.u.)" if pu_vals else r"$i_\mathrm{d}$ (A)"
    else:
        zlabel = r"$i_\mathrm{q}$ (p.u.)" if pu_vals else r"$i_\mathrm{q}$ (A)"

    return xlabel, zlabel


# %%
def _apply_3d_axis_opts(ax: Any, opts: PlotOptions) -> None:
    """Apply axis limits and ticks to 3D axes."""
    if opts.lims is not None:
        xlim, ylim, zlim = opts.lims.get("x"), opts.lims.get("y"), opts.lims.get("z")
        if xlim is not None:
            ax.set_xlim(xlim)
        if ylim is not None:
            ax.set_ylim(ylim)
        if zlim is not None:
            ax.set_zlim(zlim)
    if opts.ticks is not None:
        xticks = opts.ticks.get("x")
        yticks = opts.ticks.get("y")
        zticks = opts.ticks.get("z")
        if xticks is not None:
            ax.set_xticks(xticks)
        if yticks is not None:
            ax.set_yticks(yticks)
        if zticks is not None:
            ax.set_zticks(zticks)
    # Fine tuning of tick label padding
    ax.xaxis.set_tick_params(pad=2)
    ax.yaxis.set_tick_params(pad=1)
    ax.zaxis.set_tick_params(pad=-1)


# %%
def plot_surface_vs_current_and_angle(
    current_range: np.ndarray,
    fixed_value: float,
    theta_m_range: np.ndarray,
    map_fcn: Callable[..., tuple[Any, Any]],
    input: Literal["i_d", "i_q", "psi_d", "psi_q"] = "i_q",
    output: Literal["psi_d", "psi_q", "tau_m", "i_d", "i_q"] = "tau_m",
    val_data: tuple | None = None,
    trn_data: tuple | None = None,
    opts: PlotOptions | None = None,
) -> None:  # noqa: PLR0912, PLR0915
    """
    Plot selected output component vs selected input component over angle.

    Parameters
    ----------
    current_range : np.ndarray
        Range of input values to vary.
    fixed_value : float
        Fixed input value for the other axis.
    theta_m_range : np.ndarray
        Electrical rotor angle range (rad).
    map_fcn : Callable
        Flux or current map function with harmonics.
    input : {"i_d", "i_q", "psi_d", "psi_q"}, optional
        Input axis to vary, defaults to "i_q".
    output : {"psi_d", "psi_q", "tau_m", "i_d", "i_q"}, optional
        Output quantity to plot, defaults to "tau_m".
    val_data : tuple, optional
        Validation data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
    trn_data : tuple, optional
        Training data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
    opts : PlotOptions, optional
        Plotting options.

    """
    if opts is None:
        opts = PlotOptions()

    width, height = setup_plot_style(opts.latex)
    base, pu_vals = _get_base_values(opts)

    # Build the 2D grid for evaluation
    exp_j_theta_m = np.exp(1j * theta_m_range)
    x_grid, theta_grid = np.meshgrid(current_range, exp_j_theta_m, indexing="ij")

    # Construct input vector based on requested input type
    if input == "i_d":  # Vary i_d, fix i_q
        inp_s_dq = x_grid.ravel() + 1j * fixed_value
        d_exact, q_exact = None, fixed_value
        is_cur_input = True
    elif input == "i_q":  # Vary i_q, fix i_d
        inp_s_dq = fixed_value + 1j * x_grid.ravel()
        d_exact, q_exact = fixed_value, None
        is_cur_input = True
    elif input == "psi_d":  # Vary psi_d, fix psi_q
        inp_s_dq = x_grid.ravel() + 1j * fixed_value
        d_exact, q_exact = None, fixed_value
        is_cur_input = False
    else:  # Vary psi_q, fix psi_d
        inp_s_dq = fixed_value + 1j * x_grid.ravel()
        d_exact, q_exact = fixed_value, None
        is_cur_input = False

    # Evaluate model on 2D slice
    if is_cur_input:
        # Map: Currents -> (Fluxes, Torque)
        psi_s_dq, tau_m = map_fcn(inp_s_dq, theta_grid.ravel())
        psi_pred = np.asarray(psi_s_dq).reshape(x_grid.shape)
        # For Flux Map: i is input, so i_pred is just the input grid
        i_pred = np.asarray(inp_s_dq).reshape(x_grid.shape)
    else:
        # Map: Fluxes -> (Currents, Torque)
        i_s_dq, tau_m = map_fcn(inp_s_dq, theta_grid.ravel())
        # For Current Map: psi is input, so psi_pred is just the input grid
        psi_pred = np.asarray(inp_s_dq).reshape(x_grid.shape)
        i_pred = np.asarray(i_s_dq).reshape(x_grid.shape)

    tau_pred = np.asarray(tau_m).reshape(x_grid.shape)

    # Extract optional point clouds from validation and training data
    x_trn, y_trn, z_trn = extract_surface_plot_points(
        trn_data, input, d_exact, q_exact, output
    )
    x_val, y_val, z_val = extract_surface_plot_points(
        val_data, input, d_exact, q_exact, output
    )

    # Create figure and 3D axes
    fig = plt.figure(figsize=(width, height))
    ax = fig.add_subplot(111, projection="3d")

    # Build grid and z values
    y_grid = np.angle(theta_grid)

    if output == "tau_m":
        z_grid = tau_pred
    elif output in ("psi_d", "psi_q"):
        z_grid = np.real(psi_pred) if output == "psi_d" else np.imag(psi_pred)
    else:
        z_grid = np.real(i_pred) if output == "i_d" else np.imag(i_pred)

    z_scale = base.tau if output == "tau_m" else base.psi
    if output in ("i_d", "i_q"):
        z_scale = base.i

    # x-axis scale: depends on input type
    x_scale = base.psi if input.startswith("psi") else base.i

    ax.plot_surface(
        x_grid / x_scale,
        np.rad2deg(y_grid),
        (base.n_p * z_grid / z_scale) if output == "tau_m" else (z_grid / z_scale),
        cmap=opts.surface_cmap,
        alpha=0.5,
    )

    # Replace the dense wireframe with cleaner constant-input loci lines
    if input in ("i_d", "i_q"):
        x_axis = current_range / x_scale
        y_axis = np.rad2deg(theta_m_range)
        z_plot_grid = (
            base.n_p * z_grid / z_scale if output == "tau_m" else (z_grid / z_scale)
        )

        def _unique_levels(arr: np.ndarray) -> list[float]:
            arr = np.asarray(arr, dtype=float)
            arr = arr[np.isfinite(arr)]
            if arr.size == 0:
                return []
            # Float-safe unique (dataset values may not be bitwise identical)
            return np.unique(np.round(arr, 6)).astype(float).tolist()

        x_levels: list[float]
        if opts.loci_levels_source == "val" and val_data is not None:
            x_levels = _unique_levels(x_val / x_scale)
        elif opts.loci_levels_source == "trn" and trn_data is not None:
            x_levels = _unique_levels(x_trn / x_scale)
        elif opts.ticks is not None and opts.ticks.get("x") is not None:
            x_levels = list(opts.ticks["x"])
        elif opts.lims is not None and opts.lims.get("x") is not None:
            xmin, xmax = opts.lims["x"]
            x_levels = np.linspace(xmin, xmax, 5).tolist()
        else:
            x_levels = np.linspace(
                float(np.nanmin(x_axis)), float(np.nanmax(x_axis)), 5
            ).tolist()

        def _interp_1d(x0: float, xp: np.ndarray, fp: np.ndarray) -> float:
            mask = np.isfinite(fp)
            if mask.sum() < 2:
                return float("nan")
            return float(np.interp(x0, xp[mask], fp[mask]))

        for x0 in x_levels:
            if x0 < float(np.nanmin(x_axis)) or x0 > float(np.nanmax(x_axis)):
                continue
            zs = np.array(
                [
                    _interp_1d(float(x0), x_axis, z_plot_grid[:, k])
                    for k in range(z_plot_grid.shape[1])
                ]
            )
            mask = np.isfinite(zs)
            if mask.sum() >= 2:
                ax.plot(
                    np.full_like(y_axis[mask], float(x0)),
                    y_axis[mask],
                    zs[mask],
                    color="k",
                    linewidth=0.5,
                    alpha=LINE_ALPHA,
                )

        # Constant-angle loci (y = const)
        y_levels: list[float]
        if opts.loci_levels_source == "val" and val_data is not None:
            y_levels = _unique_levels(np.rad2deg(y_val))
        elif opts.loci_levels_source == "trn" and trn_data is not None:
            y_levels = _unique_levels(np.rad2deg(y_trn))
        elif opts.ticks is not None and opts.ticks.get("y") is not None:
            y_levels = list(opts.ticks["y"])
        elif opts.lims is not None and opts.lims.get("y") is not None:
            ymin, ymax = opts.lims["y"]
            y_levels = np.linspace(ymin, ymax, 5).tolist()
        else:
            y_levels = np.linspace(
                float(np.nanmin(y_axis)), float(np.nanmax(y_axis)), 5
            ).tolist()

        for y0 in y_levels:
            if y0 < float(np.nanmin(y_axis)) or y0 > float(np.nanmax(y_axis)):
                continue
            zs = np.array(
                [
                    _interp_1d(float(y0), y_axis, z_plot_grid[i, :])
                    for i in range(z_plot_grid.shape[0])
                ]
            )
            mask = np.isfinite(zs)
            if mask.sum() >= 2:
                ax.plot(
                    x_axis[mask],
                    np.full_like(x_axis[mask], float(y0)),
                    zs[mask],
                    color="k",
                    linewidth=0.5,
                    alpha=LINE_ALPHA,
                )
    else:
        ax.plot_wireframe(
            x_grid / x_scale,
            np.rad2deg(y_grid),
            (base.n_p * z_grid / z_scale) if output == "tau_m" else (z_grid / z_scale),
            color="k",
            linewidth=0.5,
            alpha=LINE_ALPHA,
        )

    if val_data is not None:
        if output == "tau_m":
            z_val_scaled = base.n_p * z_val / z_scale
        else:
            z_val_scaled = z_val / z_scale
        ax.scatter(
            x_val / x_scale,
            np.rad2deg(y_val),
            z_val_scaled,  # type: ignore
            marker=".",
            color="b",
            label="Validation",
            axlim_clip=True,
            depthshade=False,
            s=5,
        )
    if trn_data is not None:
        if output == "tau_m":
            z_trn_scaled = base.n_p * z_trn / z_scale
        else:
            z_trn_scaled = z_trn / z_scale
        ax.scatter(
            x_trn / x_scale,
            np.rad2deg(y_trn),
            z_trn_scaled,  # type: ignore
            marker="*",
            color="r",
            label="Training",
            axlim_clip=True,
            depthshade=False,
            s=5,
        )

    xlabel, zlabel = _surface_plot_labels(input, output, pu_vals)
    ax.set_xlabel(xlabel, labelpad=-1)
    ax.set_ylabel(r"$\vartheta_\mathrm{m}$ (deg)", labelpad=-1)
    ax.view_init(elev=15, azim=-135)
    ax.zaxis.set_rotate_label(False)
    ax.set_zlabel(zlabel, labelpad=-3, rotation=90)
    _apply_3d_axis_opts(ax, opts)
    save_and_show(opts.save_path, **opts.savefig_kwargs)


# %%
def plot_output_vs_angle(
    fixed_value: complex,
    theta_m_range: np.ndarray,
    map_fcn: Callable[..., tuple[Any, Any]],
    output: Literal["psi_d", "psi_q", "tau_m"] = "tau_m",
    input_type: Literal["i_s_dq", "psi_s_dq"] = "i_s_dq",
    val_data: tuple | None = None,
    trn_data: tuple | None = None,
    opts: PlotOptions | None = None,
) -> None:  # noqa: PLR0912, PLR0915
    """
    Plot torque or flux linkage as a function of angle at a fixed input.

    Parameters
    ----------
    fixed_value : complex
        Fixed complex input value (e.g., i_s_dq or psi_s_dq).
    theta_m_range : np.ndarray
        Electrical rotor angle range (rad).
    map_fcn : Callable
        Flux or current map function with harmonics.
    output : {"psi_d", "psi_q", "tau_m"}, optional
        Output quantity to plot, defaults to "tau_m".
    input_type : {"i_s_dq", "psi_s_dq"}, optional
        Type of the fixed input value, defaults to "i_s_dq".
    val_data : tuple, optional
        Validation data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
    trn_data : tuple, optional
        Training data tuple containing (i_s_dq, psi_s_dq, theta_m, tau_m).
    opts : PlotOptions, optional
        Plotting options.

    """
    if opts is None:
        opts = PlotOptions()

    setup_plot_style(opts.latex)
    base, pu_vals = _get_base_values(opts)

    # Create plot
    fig, ax = plt.subplots()

    exp_j_theta_m = np.exp(1j * theta_m_range)

    # Evaluate model at fixed input over angle range
    inp_s_dq = np.full_like(theta_m_range, fixed_value, dtype=complex)
    if input_type == "i_s_dq":
        psi_s_dq_slice, tau_m_slice = map_fcn(inp_s_dq, exp_j_theta_m)
    else:
        out1_slice, tau_m_slice = map_fcn(inp_s_dq, exp_j_theta_m)
        psi_s_dq_slice = out1_slice

    if output == "tau_m":
        y_slice = np.asarray(tau_m_slice)
        y_scale = base.tau
        y_plot = base.n_p * y_slice / y_scale
    else:
        y_slice = np.asarray(psi_s_dq_slice)
        y_scale = base.psi
        y_plot = (
            np.real(y_slice) / y_scale
            if output == "psi_d"
            else np.imag(y_slice) / y_scale
        )
    ax.plot(np.rad2deg(theta_m_range), y_plot, color="m", linewidth=1.5, label="Model")

    # Extract and plot validation/training data
    def extract_points(data: tuple | None) -> tuple[np.ndarray, np.ndarray]:
        if data is None:
            return np.array([]), np.array([])
        # Assumption: data is always (i, psi, theta, tau)
        i_s_dq_data, psi_s_dq_data, theta_m_data, tau_m_data = data
        d_exact, q_exact = fixed_value.real, fixed_value.imag

        # Select target array for filtering based on input_type
        if input_type == "i_s_dq":
            target_array = i_s_dq_data
        else:
            target_array = psi_s_dq_data

        mask = np.isclose(target_array.real, d_exact, atol=1e-5) & np.isclose(
            target_array.imag, q_exact, atol=1e-5
        )

        if output == "tau_m":
            y_data = tau_m_data[mask]
        elif output == "psi_d":
            # If input is flux, output psi_d is trivial (it's the input),
            # but function allows plotting it.
            y_data = psi_s_dq_data.real[mask]
        else:
            y_data = psi_s_dq_data.imag[mask]

        return theta_m_data[mask], y_data

    # Plot validation data
    if val_data is not None:
        x_val, y_val = extract_points(val_data)
        if len(x_val) > 0:
            y_scale = base.tau if output == "tau_m" else base.psi
            y_plot = (
                base.n_p * y_val / y_scale if output == "tau_m" else y_val / y_scale
            )
            ax.scatter(
                np.rad2deg(x_val),
                y_plot,
                color="b",
                marker=".",
                label="Validation",
                alpha=0.6,
            )

    # Plot training data
    if trn_data is not None:
        x_trn, y_trn = extract_points(trn_data)
        if len(x_trn) > 0:
            y_scale = base.tau if output == "tau_m" else base.psi
            y_plot = (
                base.n_p * y_trn / y_scale if output == "tau_m" else y_trn / y_scale
            )
            ax.scatter(
                np.rad2deg(x_trn),
                y_plot,
                color="r",
                marker="*",
                label="Training",
                alpha=0.6,
            )

    # Limits and ticks
    if opts.lims is not None:
        ax.set_xlim(opts.lims.get("x", ax.get_xlim()))
        ax.set_ylim(opts.lims.get("y", ax.get_ylim()))
    if opts.ticks is not None:
        ax.set_xticks(opts.ticks.get("x", ax.get_xticks()))
        ax.set_yticks(opts.ticks.get("y", ax.get_yticks()))

    ax.set_xlabel(r"$\vartheta_\mathrm{m}$ (deg)")
    if output == "tau_m":
        ylabel = (
            r"$\tau_\mathrm{m}$ (p.u.)"
            if pu_vals
            else r"$\tau_\mathrm{M}/n_\mathrm{p}$ (Nm)"
        )
    elif output == "psi_d":
        ylabel = r"$\psi_\mathrm{d}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{d}$ (Vs)"
    else:
        ylabel = r"$\psi_\mathrm{q}$ (p.u.)" if pu_vals else r"$\psi_\mathrm{q}$ (Vs)"
    ax.set_ylabel(ylabel)
    ax.legend()

    save_and_show(opts.save_path, **opts.savefig_kwargs)
