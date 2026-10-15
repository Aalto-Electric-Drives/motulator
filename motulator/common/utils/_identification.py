"""Common functions for frequency-response identification by signal injection."""

import multiprocessing as mp
from collections.abc import Callable, Sequence
from contextlib import nullcontext
from os import environ
from time import time
from typing import Any

import numpy as np
from scipy.integrate import trapezoid

# Task of a worker process, set by the pool initializer. The task contains the model
# and the control system, which are thus passed to each worker process once, not for
# each excitation. If the worker processes are forked, the task is not pickled at all.
_task: Callable[[Any], Any] | None = None


def _init_worker(task: Callable[[Any], Any]) -> None:
    global _task  # noqa: PLW0603
    _task = task


def _run_task(excitation: Any) -> Any:
    assert _task is not None
    return _task(excitation)


# %%
def dft(
    t: np.ndarray, x: np.ndarray, f: float, T: float, window: bool = False
) -> complex:
    """
    Single-frequency Fourier transform.

    Calculates the complex amplitude of the frequency component `f` of the signal `x`
    over the last time span `T`, which should be an integer number of periods. The
    transform is computed with the trapezoidal rule from the time instants `t`, which
    need not be evenly spaced.

    Parameters
    ----------
    t : ndarray
        Time instants (s).
    x : ndarray
        Signal.
    f : float
        Frequency (Hz).
    T : float
        Time span (s).
    window : bool, optional
        Use the Blackman window, defaults to False. The window suppresses the leakage
        from the components that are not periodic in `T`, e.g., a decaying transient.
        Its main lobe spreads over three frequency bins, so `T` must span at least
        three periods, or the DC component leaks to the result.

    Returns
    -------
    complex
        Complex amplitude of the frequency component.

    """
    if window and round(f * T) < 3:
        raise ValueError("The window needs a time span of at least three periods")
    t_start = t[-1] - T

    # Window of the last time span, starting from the value interpolated at t_start
    k = np.searchsorted(t, t_start, side="right")
    x_start = np.interp(t_start, t, np.real(x)) + 1j * np.interp(t_start, t, np.imag(x))
    t = np.concatenate(([t_start], t[k:]))
    x = np.concatenate(([x_start], x[k:]))

    w = np.ones_like(t)
    if window:
        a = 2 * np.pi * (t - t_start) / T
        w = 0.42 - 0.5 * np.cos(a) + 0.08 * np.cos(2 * a)  # Blackman window

    # The integral of the window normalizes the amplitude
    return 2 * trapezoid(w * x * np.exp(-2j * np.pi * f * t), t) / trapezoid(w, t)


def run_frequency_sweep(
    task: Callable[[Any], Any], excitations: Sequence[Any], multiprocess: bool = True
) -> list[Any]:
    """
    Run the identification task for each excitation and print the progress.

    Parameters
    ----------
    task : Callable[[Any], Any]
        Identification at a single excitation.
    excitations : Sequence[Any]
        Excitations, e.g., frequencies and amplitudes.
    multiprocess : bool, optional
        If True, the excitations are simulated in parallel using all available CPU
        cores, defaults to True.

    Returns
    -------
    list[Any]
        Results in the order of the excitations.

    """
    show_progress = environ.get("BUILDING_DOCS") != "1"
    if show_progress:
        print("Start identification...")
    t_start = time()
    results = []
    with (
        mp.Pool(initializer=_init_worker, initargs=(task,))
        if multiprocess
        else nullcontext()
    ) as pool:
        mapped = pool.imap(_run_task, excitations) if pool else map(task, excitations)
        for k, result in enumerate(mapped, start=1):
            results.append(result)
            if show_progress:
                print(f"\rFrequencies simulated: {k}/{len(excitations)}", end="")
    if show_progress:
        print(f"\nExecution time: {(time() - t_start):.2f} s")
    return results
