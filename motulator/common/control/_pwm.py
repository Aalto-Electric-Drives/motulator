"""
Pulse-width modulation (PWM) for three-phase converters.

This module contains implementations of space-vector PWM with different overmodulation
strategies.

"""

from cmath import exp, phase
from math import acos, floor, pi, sqrt
from typing import Literal

from motulator.common.utils import abc2complex, complex2abc


# %%
class PWM:
    """
    Duty ratios and realized voltage for three-phase space-vector PWM.

    This computes the duty ratios corresponding to standard space-vector PWM and
    overmodulation [#Hav1999]_. The realized voltage is computed based on the measured
    DC-bus voltage and the duty ratios. The digital delay effects are taken into account
    in the realized voltage [#Bae2003]_.

    Parameters
    ----------
    k_comp : float, optional
        Compensation factor for the angular delay effect, defaults to 1.5.
    u_c_ab0 : float, optional
        Initial voltage (V) in stationary coordinates. This is used to compute the
        realized voltage, defaults to 0.
    overmodulation : Literal["MPE", "MME", "six_step"], optional
        Overmodulation method, defaults to "MPE". Valid options are:
        - "MPE": minimum phase error
        - "MME": minimum magnitude error
        - "six_step": six-step operation

    References
    ----------
    .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
       triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
       https://doi.org/10.1109/28.777199

    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
       synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.810660

    """

    def __init__(
        self,
        k_comp: float = 1.5,
        u_c_ab0: complex = 0j,
        overmodulation: Literal["MPE", "MME", "six_step"] = "MPE",
    ) -> None:
        self.k_comp = k_comp
        self.overmodulation = overmodulation
        self.realized_voltage = u_c_ab0
        self._old_u_c_ab = u_c_ab0

    @staticmethod
    def six_step_overmodulation(u_c_ab_ref: complex, u_dc: float) -> complex:
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached [#Bol1997]_.

        Parameters
        ----------
        u_c_ab_ref : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        u_c_ab_ref : complex
            Modified converter voltage reference (V) in stationary coordinates.

        References
        ----------
        .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of SVM
           inverters in the overmodulation range," IEEE Trans. Ind. Appl., 1997,
           https://doi.org/10.1109/28.568019

        """
        # Limited magnitude
        r = min([abs(u_c_ab_ref), 2 / 3 * u_dc])

        if sqrt(3) * r > u_dc:
            # Angle and sector of the reference vector
            theta = phase(u_c_ab_ref)
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
            u_c_ab_ref = r * exp(1j * (theta0 + sector * pi / 3))

        return u_c_ab_ref

    def duty_ratios(self, u_c_ab_ref: complex, u_dc: float) -> list[float]:
        """
        Compute the duty ratios for three-phase space-vector PWM.

        Parameters
        ----------
        u_c_ab_ref : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        d_abc : list[float]
            Duty ratios.

        """
        # Phase voltages without the zero-sequence voltage
        u_abc = complex2abc(u_c_ab_ref)

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
        self, T_s: float, u_c_ab_ref: complex, u_dc: float, w: float
    ) -> tuple[list[float], complex]:
        """
        Compute the duty ratios and the limited voltage reference.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_c_ab_ref : complex
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
        u_c_ab_ref = exp(1j * theta_comp) * u_c_ab_ref

        # Modify angle in the overmodulation region
        if self.overmodulation == "six_step":
            u_c_ab_ref = self.six_step_overmodulation(u_c_ab_ref, u_dc)

        # Duty ratios
        d_abc = self.duty_ratios(u_c_ab_ref, u_dc)

        # Limited voltage reference
        u_c_ab = abc2complex(d_abc) * u_dc

        return d_abc, u_c_ab

    def get_realized_voltage(self) -> complex:
        """
        Get the realized voltage.

        Returns
        -------
        realized_voltage : complex
            Realized converter voltage (V) in stationary coordinates. The effect of the
            digital delays on the angle are compensated for.

        """
        return self.realized_voltage

    def update(self, u_c_ab: complex) -> None:
        """Update the realized voltage."""
        self.realized_voltage = 0.5 * (self._old_u_c_ab + u_c_ab)
        self._old_u_c_ab = u_c_ab

    def __call__(
        self, T_s: float, u_c_ab_ref: complex, u_dc: float, w: float
    ) -> list[float]:
        d_abc, u_c_ab = self.compute_output(T_s, u_c_ab_ref, u_dc, w)
        self.update(u_c_ab)
        return d_abc
