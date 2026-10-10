"""Sensor model for measured signals."""

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import ArrayLike


# %%
@dataclass
class Sensor:
    """
    Sensor with a gain error, an offset, and additive Gaussian noise.

    The measured signal is ``y = gain*x + offset + std*n``, where `x` is the actual
    signal and `n` is zero-mean white Gaussian noise with unit variance, drawn
    independently for each channel at each sample. The parameters are scalars or arrays
    with one element per channel (e.g., per phase), in the units of the measured
    signal. The noise represents the sampled signal, i.e., `std` is the standard
    deviation of the samples after the anti-aliasing filter and the A/D conversion.

    Parameters
    ----------
    gain : array_like, optional
        Gain, defaults to 1.
    offset : array_like, optional
        Offset, defaults to 0.
    std : array_like, optional
        Standard deviation of the noise, defaults to 0.
    rng : numpy.random.Generator, optional
        Random number generator, defaults to an unseeded generator. For reproducible
        results, pass the same seeded generator to all sensors. Sensors with separate
        generators of the same seed would produce identical noise.

    Examples
    --------
    >>> from motulator.common.model import Sensor
    >>> sensor = Sensor(gain=[1.02, 1, 1], offset=[0, 0.1, 0])
    >>> sensor([1.0, -0.5, -0.5])
    array([ 1.02, -0.4 , -0.5 ])

    """

    gain: ArrayLike = 1.0
    offset: ArrayLike = 0.0
    std: ArrayLike = 0.0
    rng: np.random.Generator = field(default_factory=np.random.default_rng)

    def __call__(self, x: ArrayLike) -> Any:
        """Return the measured signal."""
        y = np.multiply(self.gain, x) + self.offset
        if np.any(self.std):
            y = y + np.multiply(self.std, self.rng.standard_normal(np.shape(x)))
        return y
