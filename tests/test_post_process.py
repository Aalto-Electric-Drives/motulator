"""
Tests for the post-processing of the controller time series.

Some controllers transform their post-processed signals to other coordinates and update
the angle `fbk.theta_c` of the coordinates accordingly. The tests check that the
stationary signals, `exp(1j*fbk.theta_c)` times each complex signal, are unchanged.

"""

from collections.abc import Callable
from copy import deepcopy
from math import pi
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from motulator.common.control import TimeSeries
from motulator.drive.control._common import References as DriveReferences
from motulator.drive.control._im_observers import (
    ObserverOutputs as InductionMachineObserverOutputs,
)
from motulator.drive.control._im_observers import to_rotor_flux_coordinates
from motulator.grid import control
from motulator.grid.control._common import References as GridReferences
from motulator.grid.control._gfm_observer import ObserverOutputs as GridObserverOutputs
from motulator.grid.control._gfm_observer import transform_to_v_c_coordinates
from motulator.grid.control._gfm_sequence_observer import (
    SequenceObserverOutputs,
    SequenceReferences,
)

# Angles of the coordinates, which wrap around +-pi after the transformation
THETA_C = np.array([0.0, 3.1, -3.1, 1.2, -2.0])
# Complex signals, including zero
X = np.array([0j, 2 + 3j, 1 - 2j, -2 + 1j, 3j])
# Vectors with which the new coordinates are aligned, including zero
ALIGN = np.array([0j, np.exp(0.8j), np.exp(-0.9j), 2 * np.exp(2j), np.exp(-2.5j)])


def time_series(fbk: Any, ref: Any, align: str) -> TimeSeries:
    """Time series with the default signals, except for the complex signals."""

    def group(signals: Any, **given: np.ndarray) -> SimpleNamespace:
        values = {
            name: np.array([value] * len(X)) for name, value in vars(signals).items()
        }
        values |= {
            name: X
            for name, value in vars(signals).items()
            if isinstance(value, complex)
        }
        return SimpleNamespace(**(values | given))

    return TimeSeries(
        t=np.arange(len(X)),
        fbk=group(fbk, theta_c=THETA_C, **{align: ALIGN}),
        ref=group(ref),
    )


def sequence_post_process(ts: TimeSeries) -> None:
    """Post-process with the sequence-observer-based grid-forming controller."""
    cfg = control.SequenceObserverBasedGridFormingControllerCfg(
        i_max=30, i_d_max=25, L=3e-3, u_nom=325, w_nom=2 * pi * 50, T_s=250e-6
    )
    control.SequenceObserverBasedGridFormingController(cfg).post_process(ts)


CASES: dict[str, tuple[Any, Any, str, Callable[[TimeSeries], None]]] = {
    "im_rotor_flux": (
        InductionMachineObserverOutputs(),
        DriveReferences(),
        "psi_R",
        to_rotor_flux_coordinates,
    ),
    "gfm_v_c": (
        GridObserverOutputs(),
        GridReferences(),
        "v_c",
        transform_to_v_c_coordinates,
    ),
    "gfm_sequence_v_cp": (
        SequenceObserverOutputs(),
        SequenceReferences(),
        "v_cp",
        sequence_post_process,
    ),
}


@pytest.mark.parametrize("case", CASES)
def test_transformation_keeps_stationary_signals(case: str) -> None:
    """All complex signals are rotated consistently with the updated angle."""
    fbk, ref, align, post_process = CASES[case]
    ts = time_series(fbk, ref, align)
    ts_new = deepcopy(ts)
    post_process(ts_new)

    theta_c, theta_c_new = ts.fbk.theta_c, ts_new.fbk.theta_c
    assert np.all(np.abs(theta_c_new) <= pi)
    assert theta_c_new != pytest.approx(theta_c)  # The coordinates changed
    for group, group_new in ((ts.fbk, ts_new.fbk), (ts.ref, ts_new.ref)):
        for name, x in vars(group).items():
            if np.iscomplexobj(x):
                x_new = vars(group_new)[name]
                assert np.exp(1j * theta_c_new) * x_new == pytest.approx(
                    np.exp(1j * theta_c) * x
                ), name
