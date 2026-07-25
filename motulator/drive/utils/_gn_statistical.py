"""Statistical utilities for GradNet examples."""

import numpy as np

from motulator.common.utils._utils import BaseValues


def _get_base_values(base) -> BaseValues:
    """Return base values."""
    if base is None:
        return BaseValues.unity()
    return base


def compute_errors(y_true, y_pred, scale: float) -> dict[str, float]:
    err = np.abs(np.asarray(y_pred) - np.asarray(y_true)) / float(scale)
    return {
        "rmse": float(np.sqrt(np.mean(err**2))),
        "max": float(np.max(err)),
        "std": float(np.std(err)),
    }


def print_errors(name: str, y_true, y_pred, scale: float, unit: str = " p.u.") -> None:
    """Print RMSE/max/std of |y_pred - y_true|/scale with a unit suffix."""
    e = compute_errors(y_true, y_pred, scale=scale)
    print(
        f"{name}: "
        f"rmse={e['rmse']:.3f}{unit}, "
        f"max={e['max']:.3f}{unit}, "
        f"std={e['std']:.3f}{unit}"
    )


def print_errors_pu(name: str, y_true, y_pred, scale: float) -> None:
    """Print per-unit RMSE/max/std of |y_pred - y_true|/scale."""
    print_errors(name, y_true, y_pred, scale=scale, unit=" p.u.")


def _split_meas_tuple_or_dict(data) -> tuple[np.ndarray, np.ndarray]:
    """Return (psi_s_dq, i_s_dq) from either a tuple or an np.load dict-like."""
    if isinstance(data, tuple) or isinstance(data, list):
        psi_s_dq, i_s_dq = data
        return psi_s_dq, i_s_dq
    return data["psi_s_dq"], data["i_s_dq"]


def print_meas_current_map_error_metrics(
    current_map, data, base: BaseValues | None = None, name: str = "val"
) -> None:
    """Print per-unit error metrics for a measured current map: i_s(psi_s)."""
    base = _get_base_values(base)
    psi_s_dq, i_s_dq = _split_meas_tuple_or_dict(data)
    i_hat_s_dq = current_map(psi_s_dq)

    print(f"Error metrics ({name} set):")
    print_errors_pu("Current map: i_s(psi_s)", i_s_dq, i_hat_s_dq, scale=base.i)


def print_meas_flux_map_error_metrics(
    flux_map, data, base: BaseValues | None = None, name: str = "val"
) -> None:
    """Print per-unit error metrics for a measured flux map: psi_s(i_s)."""
    base = _get_base_values(base)
    psi_s_dq, i_s_dq = _split_meas_tuple_or_dict(data)
    psi_hat_s_dq = flux_map(i_s_dq)

    print(f"Error metrics ({name} set):")
    print_errors_pu("Flux map: psi_s(i_s)", psi_s_dq, psi_hat_s_dq, scale=base.psi)


# %%


def stat_fem(map_fcn, raw_data, base) -> None:
    unit = " p.u." if base is not None else ""
    base = _get_base_values(base)
    i_s_dq, psi_s_dq, theta_m, tau_m = (
        raw_data["i_s_dq"],
        raw_data["psi_s_dq"],
        raw_data["theta_m"],
        raw_data["tau_m"],
    )
    print("Error metrics:")
    psi_pred, tau_pred = map_fcn(i_s_dq, np.exp(1j * theta_m))
    print_errors("Flux linkage", psi_s_dq, psi_pred, base.psi, unit=unit)
    print_errors("Torque", tau_m, tau_pred, base.tau, unit=unit)


def stat_fem_curr(map_fcn, raw_data, base) -> None:
    unit = " p.u." if base is not None else ""
    base = _get_base_values(base)
    i_s_dq, psi_s_dq, theta_m, tau_m = (
        raw_data["i_s_dq"],
        raw_data["psi_s_dq"],
        raw_data["theta_m"],
        raw_data["tau_m"],
    )
    print("Error metrics:")
    i_pred, tau_pred = map_fcn(psi_s_dq, np.exp(1j * theta_m))
    print_errors("Current", i_s_dq, i_pred, base.i, unit=unit)
    print_errors("Torque", tau_m, tau_pred, base.tau, unit=unit)
