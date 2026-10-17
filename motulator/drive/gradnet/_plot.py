"""Plotting utilities for GradNet examples."""

# ruff: noqa: PLR0912, PLR0915, E501
# pylint: disable=too-many-branches,too-many-statements

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
from matplotlib import pyplot as plt

from motulator.common.utils._plotting import save_and_show, setup_map_plot
from motulator.common.utils._utils import BaseValues

LINE_ALPHA = 0.7


# %%
@dataclass
class PlotOptions:
    """
    Options for plotting functions.

    Parameters
    ----------
    base : ~motulator.drive.utils.BaseValues | None, optional
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
    savefig_kwargs: dict[str, Any] = field(default_factory=dict)
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
    # Data is (psi_s_dq, i_s_dq, theta_m, tau_m) or (psi_s_dq, i_s_dq), in the order of
    # get_training_data
    if len(data) == 4:
        psi_s_dq, i_s_dq, theta_m, tau_m = data
    elif len(data) == 2:
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
        Validation data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
    trn_data : tuple, optional
        Training data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
    opts : PlotOptions, optional
        Plotting options.

    """
    if opts is None:
        opts = PlotOptions()

    width, height = setup_map_plot(opts.latex)
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
        Validation data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
    trn_data : tuple, optional
        Training data tuple containing (psi_s_dq, i_s_dq, theta_m, tau_m).
    opts : PlotOptions, optional
        Plotting options.

    """
    if opts is None:
        opts = PlotOptions()

    setup_map_plot(opts.latex)
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
        # Data is (psi_s_dq, i_s_dq, theta_m, tau_m), in the order of get_training_data
        psi_s_dq_data, i_s_dq_data, theta_m_data, tau_m_data = data
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
