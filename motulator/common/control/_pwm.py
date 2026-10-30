"""
Pulse-width modulation (PWM) for three-phase converters.

This module contains implementations of space-vector PWM and discontinuous PWM with
different overmodulation strategies.

"""

from cmath import exp, phase
from collections.abc import Callable
from math import acos, floor, pi, sqrt
from typing import Literal

import numpy as np

from motulator.common.utils import abc2complex, complex2abc
from motulator.common.utils._dead_time import averaged_gate_signals

# %%
DutyRatioError = Callable[[np.ndarray, np.ndarray, float, float], np.ndarray]
Method = Literal["SVPWM", "DPWM0", "DPWM1", "DPWM2", "DPWM3"]


class InverterNonlinearity:
    """
    Duty-ratio error due to the dead time and the threshold voltages.

    The dead time blanks a leg for `t_d` after each switching instant, and the current
    direction determines the leg state during the blanking. Away from the duty-ratio
    limits, the resulting error is `sign(i)*t_d/(2*T_half)`, where `T_half` is the
    half carrier period. Pulses shorter than the dead time are suppressed, and clamped
    legs have no dead-time error. The threshold voltage `u_th` of the power devices,
    assumed equal for the switches and the diodes, causes the voltage drop
    `sign(i)*u_th` in each phase, i.e., the duty-ratio error `sign(i)*u_th/u_dc`,
    also in the clamped legs. The realized duty ratios are `d_abc - d_err`. This model
    is consistent with `motulator.common.model.VoltageSourceConverter`.

    Parameters
    ----------
    t_d : float, optional
        Dead time (s), defaults to 0.
    u_th : float, optional
        Threshold voltage (V) of the power devices, defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Current-direction function, defaults to `np.sign`. A smooth approximation may
        be used, matching the converter model.

    Notes
    -----
    This switching-cycle averaged model assumes constant duty ratios and currents
    over a carrier period. It does not include transients due to changes in the
    switching commands.

    """

    def __init__(
        self,
        t_d: float = 0.0,
        u_th: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
    ) -> None:
        self.t_d = t_d
        self.u_th = u_th
        self.sign = sign

    def __call__(
        self, i_abc: np.ndarray, d_abc: np.ndarray, u_dc: float, T_half: float
    ) -> np.ndarray:
        """
        Compute the duty-ratio error.

        Parameters
        ----------
        i_abc : ndarray, shape (3,)
            Phase currents (A).
        d_abc : ndarray, shape (3,)
            Duty ratios in the range [0, 1].
        u_dc : float
            DC-bus voltage (V).
        T_half : float
            Half carrier period (s).

        Returns
        -------
        ndarray, shape (3,)
            Duty-ratio errors.

        """
        s = self.sign(i_abc)
        q_abc, b_abc = averaged_gate_signals(d_abc, self.t_d, T_half)
        return d_abc - q_abc - 0.5 * b_abc * (1 - s) + s * self.u_th / u_dc


# %%
class PWM:
    """
    Duty ratios and realized voltage for three-phase carrier-based PWM.

    This computes the duty ratios corresponding to standard space-vector PWM and
    overmodulation [#Hav1999]_. The realized voltage is reconstructed from the duty
    ratios and the measured DC-bus voltage, taking into account the computational delay
    of one sampling period. The angle of the voltage reference is not advanced here:
    the control systems compensate for the delays, if needed by the controller.

    Optionally, a discontinuous PWM (DPWM) method clamps one leg at a time to the
    positive or negative DC rail, so that each leg stops switching for 120° of the
    fundamental period in the linear modulation range [#Hav1999b]_. The average
    switching frequency is then 2/3 of the carrier frequency. The switching losses are
    minimized if the clamped leg conducts the largest current: DPWM1 suits a current in
    phase with the voltage, DPWM2 a current lagging by about 30°, DPWM0 a current
    leading by about 30°, and DPWM3 a nearly reactive current [#Hav1998]_. At the same
    average switching frequency, the current ripple of DPWM is lower than that of
    space-vector PWM in the high modulation range and higher in the low modulation
    range. The leg is clamped after the overmodulation, i.e., the voltage is the same
    for all the methods.

    Optionally, the duty-ratio error caused by the inverter nonlinearities (such as the
    dead time and the threshold voltages of the power devices) is modeled as a function
    of the phase currents, duty ratios, DC-bus voltage, and half carrier period. The
    realized voltage is corrected for this error using the measured currents, which
    correspond to the same instant as the realized voltage, and the duty ratios of the
    corresponding sampling periods.
    Furthermore, the error can be compensated for by feedforward, in which case the
    currents are predicted to the middle of the sampling period in which the duty ratios
    are applied, assuming that they rotate at the angular speed of the synchronous
    coordinates. Near the duty-ratio limits, the feedforward may not fully cancel a
    duty-dependent error. In DPWM, the error of the clamped leg is compensated for by
    shifting the other legs.

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
    method : Literal["SVPWM", "DPWM0", "DPWM1", "DPWM2", "DPWM3"], optional
        PWM method, defaults to "SVPWM". Valid options are:

        - "SVPWM": space-vector PWM
        - "DPWM0": clamping in the 60° segments preceding the peaks of the phase
          voltage reference
        - "DPWM1": clamping in the 60° segments centered at the peaks
        - "DPWM2": clamping in the 60° segments following the peaks
        - "DPWM3": clamping in the 30° segments from 30° to 60° on both sides of the
          peaks
    overmodulation : Literal["MPE", "MME", "six_step"], optional
        Overmodulation method, defaults to "MPE". Valid options are:

        - "MPE": minimum phase error
        - "MME": minimum magnitude error
        - "six_step": six-step operation
    d_err : Callable[[np.ndarray, np.ndarray, float, float], np.ndarray], optional
        Duty-ratio error as a function of the phase currents (A), duty ratios, DC-bus
        voltage (V), and half carrier period (s), i.e., the realized duty ratios are
        `d_abc - d_err(i_abc, d_abc, u_dc, T_half)`, defaults to None (no error). See
        `InverterNonlinearity` for the dead time and the threshold voltages.
    feedforward : bool, optional
        Compensate for `d_err` in the duty ratios, defaults to True.
    k_pred : float, optional
        Prediction factor of the currents for the feedforward compensation, defaults to
        1.5, which corresponds to the middle of the sampling period in which the duty
        ratios are applied after the computational delay of one sampling period.
    d_min : float, optional
        Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0
        (no limit). The duty ratio applies to a half carrier period, so the shortest
        pulse is `d_min` times the half carrier period when a leg starts or stops
        switching and about twice that otherwise.
    single_update : bool, optional
        Duty ratios are updated once per carrier period, i.e., the half carrier period
        is `T_s/2`, defaults to False (twice per carrier period, the half carrier
        period is `T_s`). Must match the system model.

    References
    ----------
    .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
       triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
       https://doi.org/10.1109/28.777199

    .. [#Hav1999b] Hava, Kerkman, Lipo, "Simple analytical and graphical methods for
       carrier-based PWM-VSI drives," IEEE Trans. Power Electron., 1999,
       https://doi.org/10.1109/63.737592

    .. [#Hav1998] Hava, Kerkman, Lipo, "A high-performance generalized discontinuous
       PWM algorithm," IEEE Trans. Ind. Appl., 1998,
       https://doi.org/10.1109/28.720446

    .. [#Wel2006] Welchko, Schulz, Hiti, "Effects and compensation of dead-time and
       minimum pulse-width limitations in two-level PWM voltage source inverters,"
       Proc. IEEE IAS Annu. Meeting, 2006, https://doi.org/10.1109/IAS.2006.256630

    """

    def __init__(
        self,
        method: Method = "SVPWM",
        overmodulation: Literal["MPE", "MME", "six_step"] = "MPE",
        d_err: DutyRatioError | None = None,
        feedforward: bool = True,
        k_pred: float = 1.5,
        d_min: float = 0.0,
        single_update: bool = False,
    ) -> None:
        if not 0 <= d_min < 0.5:
            raise ValueError("d_min must be in the range [0, 0.5)")
        if method not in ("SVPWM", "DPWM0", "DPWM1", "DPWM2", "DPWM3"):
            raise ValueError(f"Invalid PWM method: {method}")
        if overmodulation not in ("MPE", "MME", "six_step"):
            raise ValueError(f"Invalid overmodulation method: {overmodulation}")
        self.method = method
        self.k_pred = k_pred
        self.overmodulation = overmodulation
        self.d_err = d_err
        self.feedforward = feedforward
        self.d_min = d_min
        self.single_update = single_update
        self.limited_voltage = 0j
        self._i_c_ab = 0j
        # Duty ratios of the previous and the ongoing sampling periods and the half
        # carrier period of the previous one, assumed to apply also to the ongoing one.
        # Before the first duty ratios are computed, the legs do not switch, and the
        # infinite period gives no dead-time error.
        self._d_abc = [np.zeros(3), np.zeros(3)]
        self._T_half = np.inf

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

    def _duty_ratios(self, u_c_ref_ab: complex, u_dc: float) -> list[float]:
        """
        Compute the duty ratios for three-phase carrier-based PWM.

        The angle of the reference is modified before this in six-step overmodulation,
        see `compute_output`.

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
        d_abc = np.clip(d_abc, 0, 1)
        return list(self._clamp(u_c_ref_ab, d_abc))

    def _clamp(self, u_c_ref_ab: complex, d_abc: np.ndarray) -> np.ndarray:
        """
        Clamp a leg to the DC rail in discontinuous PWM.

        The duty ratios are shifted equally, which does not change the voltage, so that
        the duty ratio of the phase with the largest voltage reference becomes one or
        that of the phase with the smallest zero. The rail is selected by the magnitude
        tests of [#Hav1999b]_ for the phase voltage references: the phase with the
        maximum magnitude in DPWM1, the one with the maximum magnitude after a phase lag
        of 30° in DPWM2 and a phase lead of 30° in DPWM0, and the one with the
        intermediate magnitude in DPWM3 is clamped to the rail of its sign.

        Parameters
        ----------
        u_c_ref_ab : complex
            Converter voltage reference (V) in stationary coordinates.
        d_abc : ndarray, shape (3,)
            Duty ratios.

        Returns
        -------
        ndarray, shape (3,)
            Shifted duty ratios, unchanged in space-vector PWM. They are not clipped to
            the range [0, 1].

        """
        if self.method == "SVPWM":
            return d_abc
        u_abc = complex2abc(u_c_ref_ab)
        if self.method == "DPWM3":
            positive = max(u_abc) < -min(u_abc)
        else:
            psi = {"DPWM0": 0, "DPWM1": pi / 6, "DPWM2": pi / 3}[self.method]
            u_x = complex2abc(exp(-1j * (psi - pi / 6)) * u_c_ref_ab)
            positive = max(u_x) > -min(u_x)
        # The leg is selected by the reference, since the compensation of the duty-ratio
        # error may change the order of the duty ratios
        k = int(np.argmax(u_abc) if positive else np.argmin(u_abc))
        rail = 1.0 if positive else 0.0
        d_abc = d_abc + rail - d_abc[k]
        d_abc[k] = rail  # Exactly, since the leg must not switch
        return d_abc

    def _half_period(self, T_s: float) -> float:
        """Half carrier period (s) corresponding to the sampling period."""
        return T_s / 2 if self.single_update else T_s

    @staticmethod
    def _bind(
        d_err: DutyRatioError, i_abc: np.ndarray, u_dc: float, T_half: float
    ) -> Callable[[np.ndarray], np.ndarray]:
        """Bind the duty-ratio error to the currents, DC-bus voltage, and period."""
        return lambda d_abc: d_err(i_abc, d_abc, u_dc, T_half)

    def compute_output(
        self, T_s: float, u_c_ref_ab: complex, u_dc: float, w: float
    ) -> tuple[list[float], complex]:
        """
        Compute the duty ratios and the corresponding voltage.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_c_ref_ab : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).
        w : float
            Angular speed of the synchronous coordinates (rad/s), used for predicting
            the currents in the feedforward compensation of `d_err`.

        Returns
        -------
        d_abc : list[float]
            Duty ratios for the next sampling period.
        u_c_ab : complex
            Voltage (V) in stationary coordinates corresponding to the duty ratios
            `d_abc`, i.e., the limited voltage reference with the duty-ratio error
            compensation (if used) included.

        """
        # A discharged DC bus, e.g., with blocked gates before the precharge
        if u_dc <= 0:
            self.limited_voltage = 0j
            return [0.5, 0.5, 0.5], 0j

        # Modify angle in the overmodulation region
        if self.overmodulation == "six_step":
            u_c_ref_ab = self.six_step_overmodulation(u_c_ref_ab, u_dc)

        # Duty ratios of the voltage reference, excluding the compensation
        d_abc = self._duty_ratios(u_c_ref_ab, u_dc)
        d_ref = np.array(d_abc)

        # Compensate for the duty-ratio error using the predicted currents
        d_err = None
        if self.d_err is not None and self.feedforward:
            i_c_abc = complex2abc(exp(1j * self.k_pred * T_s * w) * self._i_c_ab)
            d_err = self._bind(self.d_err, i_c_abc, u_dc, self._half_period(T_s))
            err = d_err(d_ref)
            d_comp = self._clamp(u_c_ref_ab, d_ref + err)
            if self.method != "SVPWM":
                # The error of the clamped leg is compensated for by the other legs,
                # which shifts the realized duty ratios aimed at
                d_ref = d_comp - err
            d_abc = list(np.clip(d_comp, 0, 1))

        # Minimum pulses and the duty ratios realized with them
        d_realized = d_ref
        if self.d_min > 0:
            d_abc, d_realized = self._limit_pulses(d_abc, d_ref, d_err)

        # Limited voltage reference, excluding and including the compensation
        self.limited_voltage = abc2complex(d_realized) * u_dc
        u_c_ab = abc2complex(d_abc) * u_dc

        return d_abc, u_c_ab

    def _limit_pulses(
        self,
        d_abc: list[float],
        d_ref: np.ndarray,
        d_err: Callable[[np.ndarray], np.ndarray] | None = None,
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
        d_err : Callable[[np.ndarray], np.ndarray], optional
            Duty-ratio error as a function of the duty ratios, if it is compensated
            for, in which case the realized duty ratios are `d_abc - d_err(d_abc)`.
            Defaults to None, in which case the realized duty ratios are `d_abc`.

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
                if d_err is not None:
                    r -= d_err(d)[k]
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
        voltage is reconstructed from the duty ratios of the previous and the ongoing
        sampling periods and the measured DC-bus voltage, i.e., the variations of the
        DC-bus voltage and of the currents within the periods are not known. The
        duty-ratio error is evaluated with the half carrier period of the previous
        sampling period, which is assumed to equal that of the ongoing period. The
        measured currents are also stored for the feedforward compensation of the next
        duty ratios.

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
        if u_dc <= 0:  # A discharged DC bus, e.g., with blocked gates
            return 0j
        d_abc = self._d_abc[0 if average else 1 :]
        if self.d_err is not None:
            i_abc = complex2abc(i_c_ab)
            d_abc = [d - self.d_err(i_abc, d, u_dc, self._T_half) for d in d_abc]
        return u_dc * sum(abc2complex(d) for d in d_abc) / len(d_abc)

    def update(self, T_s: float, d_abc: list[float]) -> None:
        """Store the next duty ratios and the ongoing half carrier period."""
        self._d_abc = [self._d_abc[1], np.asarray(d_abc)]
        self._T_half = self._half_period(T_s)

    def __call__(
        self, T_s: float, u_c_ref_ab: complex, u_dc: float, w: float
    ) -> list[float]:
        d_abc, _ = self.compute_output(T_s, u_c_ref_ab, u_dc, w)
        self.update(T_s, d_abc)
        return d_abc
