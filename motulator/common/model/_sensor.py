"""Sensor model for measured signals."""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from math import inf
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from motulator.common.model._base import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array


# %%
@dataclass
class SensorInputs:
    """Measured signal."""

    x: Any = 0j


@dataclass
class SensorStates:
    """Filtered signal, complex also for a real signal."""

    y: complex = 0j


@dataclass
class SensorOutputs:
    """Sensor output."""

    y: Any = 0j


class Sensor(Subsystem):
    """
    Sensor with a bandwidth, a gain error, an offset, and additive Gaussian noise.

    A sensor is a subsystem that the model connects to the signal `x` it measures, e.g.,
    the stator current of a machine or the DC-bus voltage of a converter, see `Drive`
    and `GridConverterSystem`. Its output `y` follows `x` through a first-order low-pass
    filter with the bandwidth `alpha`, which models the dynamics of the sensor and the
    anti-aliasing filter, or equals `x` if the bandwidth is infinite. The control system
    samples the measurement by `sample`, which converts the output to the measured
    channels (e.g., the phase currents of a space vector) and returns ``gain*y + offset
    + std*n``, where `n` is zero-mean white Gaussian noise with unit variance, drawn
    independently for each channel at each sample. The parameters are scalars or arrays
    with one element per channel, in the units of the measured signal.

    Parameters
    ----------
    alpha : float, optional
        Bandwidth (rad/s), defaults to `inf`, i.e., no dynamics.
    y0 : complex | float, optional
        Initial value of the output, defaults to 0. Relevant with a finite bandwidth,
        e.g., for a DC-bus voltage sensor, whose output otherwise rises from zero.
    gain : array_like, optional
        Gain, defaults to None, i.e., unity gain.
    offset : array_like, optional
        Offset, defaults to None, i.e., no offset.
    std : array_like, optional
        Standard deviation of the noise, defaults to None, i.e., no noise.
    rng : numpy.random.Generator, optional
        Random number generator, defaults to an unseeded generator. For reproducible
        results, pass the same seeded generator to all sensors. Sensors with separate
        generators of the same seed would produce identical noise.
    convert : Callable[[Any], Any], optional
        Function converting the output to the measured channels, e.g., `complex2abc`
        for the phase currents of a space vector or `complex2line` for the line-to-line
        voltages. Defaults to None, i.e., the output is measured as such. The sensors
        of a model default to the conversion of their measurement, see `DriveSensors`
        and `GridConverterSensors`.

    Examples
    --------
    >>> from motulator.common.model import Sensor
    >>> from motulator.common.utils import complex2abc
    >>> sensor = Sensor(gain=[1.02, 1, 1], offset=[0, 0.1, 0], convert=complex2abc)
    >>> sensor.inp.x = 1 + 0j
    >>> sensor.set_outputs(0.0)
    >>> sensor.sample()
    array([ 1.02, -0.4 , -0.5 ])

    """

    def __init__(
        self,
        alpha: float = inf,
        y0: complex | float = 0.0,
        gain: ArrayLike | None = None,
        offset: ArrayLike | None = None,
        std: ArrayLike | None = None,
        rng: np.random.Generator | None = None,
        convert: Callable[[Any], Any] | None = None,
    ) -> None:
        if not alpha > 0:
            raise ValueError("The bandwidth must be positive")
        self.alpha = alpha
        self.gain = gain
        self.offset = offset
        self.std = std
        self.rng = np.random.default_rng() if rng is None else rng
        self.convert = convert
        # Without dynamics, the output depends directly on the measured signal
        self.feedthrough = frozenset() if alpha < inf else frozenset({"x"})
        self.inp: SensorInputs = SensorInputs()
        self.state: SensorStates | None = (
            SensorStates(complex(y0)) if alpha < inf else None
        )
        self.out: SensorOutputs = SensorOutputs(y0)

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        if self.state is None:
            self.out.y = self.inp.x
        else:
            y = self.state.y
            self.out.y = y if isinstance(self.inp.x, complex) else y.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        if self.state is None:
            return []
        return [self.alpha * (self.inp.x - self.state.y)]

    def sample(self) -> Any:
        """Sample the measurement: the channels with the gain, offset, and noise."""
        y = self.out.y if self.convert is None else self.convert(self.out.y)
        if self.gain is not None:
            y = np.multiply(self.gain, y)
        if self.offset is not None:
            y = np.add(y, self.offset)
        if self.std is not None:
            y = y + np.multiply(self.std, self.rng.standard_normal(np.shape(y)))
        return y

    def create_time_series(self, t: np.ndarray) -> "SensorTimeSeries":
        """Create the time series."""
        return SensorTimeSeries(t, self)


@dataclass
class SensorTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[Sensor]
    # Input
    x: np.ndarray = field(default_factory=empty_array)
    # Output
    y: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: Sensor) -> None:
        """Compute the output time series from the state."""
        if subsystem.state is not None:
            y = np.array(subsystem._history["y"])
            # Real output of a real signal, as in `Sensor.set_outputs`
            self.y = y if isinstance(subsystem.inp.x, complex) else np.real(y)

    def set_outputs(self, t: np.ndarray, subsystem: Sensor) -> None:
        """Pass the input through without dynamics."""
        if subsystem.state is None:
            self.y = self.x
