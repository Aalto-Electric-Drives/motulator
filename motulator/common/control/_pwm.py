"""
Pulse-width modulation (PWM) for three-phase converters.

This module contains implementations of space-vector PWM with different overmodulation
strategies.

"""

from cmath import exp, phase
from collections.abc import Callable
from math import acos, floor, pi, sqrt
from typing import Literal

import numpy as np

from motulator.common.utils import abc2complex, complex2abc


# %%
class PWM:
    """
    Duty ratios and realized voltage for three-phase space-vector PWM.

    This computes the duty ratios corresponding to standard space-vector PWM and
    overmodulation [#Hav1999]_. The realized voltage is computed from the duty ratios
    and the measured DC-bus voltage. The digital delay effects are taken into account
    in the realized voltage [#Bae2003]_.

    Optionally, the duty-ratio error caused by the inverter nonlinearities (such as the
    dead time and the voltage drops of the power devices) is modeled as a function of
    the phase currents and duty ratios. The realized voltage is corrected for this
    error using the measured currents, which correspond to the same instant as the
    realized voltage, and the duty ratios of the corresponding sampling periods.
    Furthermore, the error can be compensated for by feedforward, in which case the
    currents are predicted to the middle of the sampling period in which the duty ratios
    are applied, assuming that they rotate at the angular speed of the synchronous
    coordinates. Near the duty-ratio limits, the feedforward may not fully cancel a
    duty-dependent error.

    Optionally, the pulses are limited to a minimum width, e.g., due to the gate
    drivers [#Wel2006]_: the duty ratio of a switching leg is in the range
    `[d_min, 1 - d_min]`, while the duty ratios 0 and 1 (a leg not switching) are not
    limited. A duty ratio in the range `(0, d_min)` is rounded to 0 (the pulse is
    dropped) or to `d_min`, and in the range `(1 - d_min, 1)` to `1 - d_min` or 1,
    whichever gives the realized duty ratio nearer to the duty ratio of the voltage
    reference. If the duty-ratio error is compensated for, the realized duty ratios are
    those given by `d_err`, so that the compensation also accounts for the minimum
    pulses. The limited voltage reference and the realized voltage are computed from
    the limited duty ratios.

    Parameters
    ----------
    k_comp : float, optional
        Compensation factor for the angular delay effect on the voltage reference,
        defaults to 1.5. Use 0 if the controller compensates for the delays itself,
        e.g., in direct discrete-time designs.
    overmodulation : Literal["MPE", "MME", "six_step"], optional
        Overmodulation method, defaults to "MPE". Valid options are:
        - "MPE": minimum phase error
        - "MME": minimum magnitude error
        - "six_step": six-step operation
    d_err : Callable[[np.ndarray, np.ndarray], np.ndarray], optional
        Duty-ratio error as a function of the phase currents (A) and duty ratios,
        i.e., the realized duty ratios are `d_abc - d_err(i_abc, d_abc)`, defaults
        to None (no error). For the dead time, use
        `motulator.common.utils.dead_time_error`, whose sampling period must equal
        that of the control system.
    feedforward : bool, optional
        Compensate for `d_err` in the duty ratios, defaults to True.
    k_pred : float, optional
        Prediction factor of the currents for the feedforward compensation, defaults to
        1.5, which corresponds to the middle of the sampling period in which the duty
        ratios are applied after the computational delay of one sampling period.
    d_min : float, optional
        Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0
        (no limit). The duty ratio applies to a sampling period (a half of the carrier
        period), so the shortest pulse is `d_min*T_s` when a leg starts or stops
        switching and about `2*d_min*T_s` otherwise.

    References
    ----------
    .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
       triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
       https://doi.org/10.1109/28.777199

    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
       synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.810660

    .. [#Wel2006] Welchko, Schulz, Hiti, "Effects and compensation of dead-time and
       minimum pulse-width limitations in two-level PWM voltage source inverters,"
       Proc. IEEE IAS Annu. Meeting, 2006, https://doi.org/10.1109/IAS.2006.256630

    """

    def __init__(
        self,
        k_comp: float = 1.5,
        overmodulation: Literal["MPE", "MME", "six_step"] = "MPE",
        d_err: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None,
        feedforward: bool = True,
        k_pred: float = 1.5,
        d_min: float = 0.0,
    ) -> None:
        if not 0 <= d_min < 0.5:
            raise ValueError("d_min must be in the range [0, 0.5)")
        self.k_comp = k_comp
        self.k_pred = k_pred
        self.overmodulation = overmodulation
        self.d_err = d_err
        self.feedforward = feedforward
        self.d_min = d_min
        self.limited_voltage = 0j
        self._i_c_ab = 0j
        # Duty ratios and voltages per DC-bus voltage of the previous and the ongoing
        # sampling periods
        self._d_abc = [np.zeros(3), np.zeros(3)]
        self._q_ab = [0j, 0j]

    @staticmethod
    def six_step_overmodulation(u_c_ref_ab: complex, u_dc: float) -> complex:
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached [#Bol1997]_.

        Parameters
        ----------
        u_c_ref_ab : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        u_c_ref_ab : complex
            Modified converter voltage reference (V) in stationary coordinates.

        References
        ----------
        .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of SVM
           inverters in the overmodulation range," IEEE Trans. Ind. Appl., 1997,
           https://doi.org/10.1109/28.568019

        """
        # Limited magnitude
        r = min([abs(u_c_ref_ab), 2 / 3 * u_dc])

        if sqrt(3) * r > u_dc:
            # Angle and sector of the reference vector
            theta = phase(u_c_ref_ab)
            sector = floor(3 * theta / pi)

            # Angle reduced to the first sector (at which sector == 0)
            theta0 = theta - sector * pi / 3

            # Intersection angle, see Eq. (9)
            alpha_g = pi / 6 - acos(u_dc / (sqrt(3) * r))

            # Modify the angle according to Eq. (4)
            if alpha_g <= theta0 <= pi / 6:
                theta0 = alpha_g
            elif pi / 6 <= theta0 <= pi / 3 - alpha_g:
                theta0 = pi / 3 - alpha_g

            # Modified reference voltage
            u_c_ref_ab = r * exp(1j * (theta0 + sector * pi / 3))

        return u_c_ref_ab

    def duty_ratios(self, u_c_ref_ab: complex, u_dc: float) -> list[float]:
        """
        Compute the duty ratios for three-phase space-vector PWM.

        Parameters
        ----------
        u_c_ref_ab : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        d_abc : list[float]
            Duty ratios.

        """
        # Phase voltages without the zero-sequence voltage
        u_abc = complex2abc(u_c_ref_ab)

        # Zero-sequence voltage resulting in space-vector PWM
        u_0 = 0.5 * (max(u_abc) + min(u_abc))
        u_abc -= u_0

        if self.overmodulation == "MPE":
            m = (2.0 / u_dc) * max(u_abc)
            if m > 1:
                u_abc = u_abc / m

        # Duty ratios
        d_abc = u_abc / u_dc + 0.5

        # MME overmodulation (does nothing if MPE already used)
        d_abc = [max(min(d, 1.0), 0.0) for d in d_abc]
        return d_abc

    def compute_output(
        self, T_s: float, u_c_ref_ab: complex, u_dc: float, w: float
    ) -> tuple[list[float], complex]:
        """
        Compute the duty ratios and the limited voltage reference.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_c_ref_ab : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).
        w : float
            Angular speed of synchronous coordinates (rad/s).

        Returns
        -------
        d_abc : list[float]
            Duty ratios for the next sampling period.
        u_c_ab : complex
            Limited voltage reference (V) in stationary coordinates.

        """
        # Advance the angle due to the computational delay (N*T_s) and the ZOH (PWM)
        # delay (0.5*T_s), typically 1.5*T_s*w
        theta_comp = self.k_comp * T_s * w
        u_c_ref_ab = exp(1j * theta_comp) * u_c_ref_ab

        # Modify angle in the overmodulation region
        if self.overmodulation == "six_step":
            u_c_ref_ab = self.six_step_overmodulation(u_c_ref_ab, u_dc)

        # Duty ratios of the voltage reference, excluding the compensation
        d_abc = self.duty_ratios(u_c_ref_ab, u_dc)
        d_ref = np.array(d_abc)

        # Compensate for the duty-ratio error using the predicted currents
        i_c_abc = None
        if self.d_err is not None and self.feedforward:
            i_c_abc = complex2abc(exp(1j * self.k_pred * T_s * w) * self._i_c_ab)
            d_abc = list(np.clip(d_ref + self.d_err(i_c_abc, d_ref), 0, 1))

        # Minimum pulses and the duty ratios realized with them
        d_realized = d_ref
        if self.d_min > 0:
            d_abc, d_realized = self.limit_pulses(d_abc, d_ref, i_c_abc)

        # Limited voltage reference, excluding and including the compensation
        self.limited_voltage = abc2complex(d_realized) * u_dc
        u_c_ab = abc2complex(d_abc) * u_dc

        return d_abc, u_c_ab

    def limit_pulses(
        self, d_abc: list[float], d_ref: np.ndarray, i_abc: np.ndarray | None = None
    ) -> tuple[list[float], np.ndarray]:
        """
        Limit the duty ratios of the switching legs to `[d_min, 1 - d_min]`.

        A duty ratio in the range `(0, d_min)` is replaced by 0 or `d_min`, and in the
        range `(1 - d_min, 1)` by `1 - d_min` or 1, whichever gives the realized duty
        ratio nearer to the reference (if equally near, the leg switches). The duty
        ratios 0 and 1 are not changed.

        Parameters
        ----------
        d_abc : list[float]
            Duty ratios, including the compensation of the duty-ratio error.
        d_ref : ndarray, shape (3,)
            Duty ratios of the voltage reference, i.e., the realized duty ratios aimed
            at.
        i_abc : ndarray, shape (3,), optional
            Phase currents (A) for the duty-ratio error `d_err`, if it is compensated
            for, in which case the realized duty ratios are `d_abc - d_err(i_abc,
            d_abc)`. Defaults to None, in which case the realized duty ratios are
            `d_abc`.

        Returns
        -------
        d_abc : list[float]
            Limited duty ratios.
        d_realized : ndarray, shape (3,)
            Realized duty ratios: `d_ref` for the legs whose duty ratios were not
            changed and the realized duty ratios of the limited legs.

        """
        d = np.array(d_abc, dtype=float)
        d_realized = np.array(d_ref, dtype=float)
        for k in range(3):
            if 0 < d[k] < self.d_min:
                candidates = (self.d_min, 0.0)
            elif 1 - self.d_min < d[k] < 1:
                candidates = (1 - self.d_min, 1.0)
            else:
                continue
            realized = []
            for d_k in candidates:
                d[k] = d_k
                r = d_k
                if i_abc is not None and self.d_err is not None:
                    r -= self.d_err(i_abc, d)[k]
                realized.append(r)
            errors = [abs(r - d_ref[k]) for r in realized]
            j = 1 if errors[1] < errors[0] else 0
            d[k], d_realized[k] = candidates[j], realized[j]
        return list(d), d_realized

    def get_realized_voltage(
        self, i_c_ab: complex, u_dc: float, *, average: bool = True
    ) -> complex:
        """
        Get the realized voltage.

        This method is to be called at the sampling instant before the next duty ratios
        are computed, and a computational delay of one sampling period is assumed. The
        voltage is computed from the duty ratios of the previous and the ongoing
        sampling periods and the measured DC-bus voltage. The measured currents are also
        stored for the feedforward compensation of the next duty ratios.

        Parameters
        ----------
        i_c_ab : complex
            Measured converter current (A) in stationary coordinates.
        u_dc : float
            Measured DC-bus voltage (V).
        average : bool, optional
            If True, the average voltage of the previous and the ongoing sampling
            periods is returned. If False, the voltage of the ongoing sampling period
            is returned. Defaults to True.

        Returns
        -------
        complex
            Realized converter voltage (V) in stationary coordinates. If `d_err` is
            given, the voltage is corrected for it using the measured currents.

        """
        self._i_c_ab = i_c_ab
        k = 0 if average else 1
        q_ab = self._q_ab[k:]
        if self.d_err is not None:
            i_abc = complex2abc(i_c_ab)
            q_ab = [
                q - abc2complex(self.d_err(i_abc, d))
                for q, d in zip(q_ab, self._d_abc[k:], strict=True)
            ]
        return u_dc * sum(q_ab) / len(q_ab)

    def update(self, d_abc: list[float]) -> None:
        """Store the duty ratios of the next sampling period."""
        self._d_abc = [self._d_abc[1], np.asarray(d_abc)]
        self._q_ab = [self._q_ab[1], abc2complex(d_abc)]

    def __call__(
        self, T_s: float, u_c_ref_ab: complex, u_dc: float, w: float
    ) -> list[float]:
        d_abc, _ = self.compute_output(T_s, u_c_ref_ab, u_dc, w)
        self.update(d_abc)
        return d_abc
