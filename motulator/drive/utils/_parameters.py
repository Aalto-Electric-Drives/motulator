"""Common dataclasses usable in models and control of machine drives."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, cast

import numpy as np
from scipy.optimize import root, root_scalar

from motulator.common.utils._utils import get_value

EPS: float = 1e-3


# %%
def _solve_pm_flux(i_d: Callable[[float], float]) -> float:
    """
    Solve the PM-flux linkage from i_d(psi_f) = 0.

    The d-axis current is a monotonically increasing function of the d-axis flux
    linkage. The root is bracketed by doubling the upper bound and then found using
    Brent's method. No derivatives are needed, so the solution does not depend on the
    numerical precision of the map (e.g., single-precision neural networks).

    """
    i_lo = i_d(0.0)
    if i_lo >= 0:  # No permanent magnets
        return 0.0
    psi_lo, psi_hi = 0.0, EPS
    while (i_hi := i_d(psi_hi)) < 0 and psi_hi < 1e3:
        psi_lo, psi_hi = psi_hi, 2 * psi_hi
    # Accept only a converged root within a finite bracket, e.g., not NaN values
    # outside the domain of the map
    if np.isfinite(i_lo) and i_hi >= 0:
        sol = root_scalar(i_d, bracket=(psi_lo, psi_hi), method="brentq")
        if sol.converged:
            return sol.root
    raise ValueError("The PM-flux linkage cannot be solved from the current map")


# %%
def mat_vec(L_s: np.ndarray, x: complex) -> complex:
    """Matrix-vector product ``L_s @ x`` for a symmetric 2x2 matrix."""
    return L_s[0, 0] * x.real + 1j * L_s[1, 1] * x.imag + 1j * L_s[0, 1] * x.conjugate()


def aux_flux(i_s: complex, psi_s: complex, L_s: np.ndarray) -> complex:
    """Auxiliary flux linkage of a synchronous machine in rotor coordinates."""
    return (
        psi_s
        - L_s[1, 1] * i_s.real
        - 1j * L_s[0, 0] * i_s.imag
        + 1j * L_s[0, 1] * i_s.conjugate()
    )


def aux_current(psi_s: complex, i_s: complex, L_s: np.ndarray) -> complex:
    """Auxiliary current of a synchronous machine in rotor coordinates."""
    det_L = L_s[0, 0] * L_s[1, 1] - L_s[0, 1] ** 2
    return mat_vec(L_s, psi_s) / det_L - i_s


# %%
def _torque(i_s: Any, psi_s: Any) -> Any:
    """Electromagnetic torque per pole pair, without spatial harmonics."""
    return 1.5 * (i_s * psi_s.conjugate()).imag


# %%
class BaseSynchronousMachinePars(Protocol):
    """
    Base class for synchronous machine parameters, as used by the machine model.

    The machine model consists of the stator resistance and the magnetic model, which
    maps the stator flux linkage to the stator current. The permanent-magnet flux
    linkage `psi_f` is the stator flux linkage at zero current.

    """

    n_p: int
    R_s: float
    G_c: float
    psi_f: float

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """
        Magnetic map as a function of flux linkage and electrical rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        tuple[complex | ndarray, float | ndarray]
            Stator current (A) in rotor coordinates and electromagnetic torque (Nm) per
            pole pair.

        """
        ...


# %%
@dataclass
class SynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Synchronous machine parameters, without saturation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    L_d : float
        d-axis inductance (H).
    L_q : float
        q-axis inductance (H).
    psi_f : float
        Permanent-magnet flux linkage (Vs).
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    """

    n_p: int
    R_s: float
    L_d: float
    L_q: float
    psi_f: float
    G_c: float = 0.0

    def i_s_dq(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        i_d = (np.real(psi_s_dq) - self.psi_f) / self.L_d
        i_q = np.imag(psi_s_dq) / self.L_q
        i_s_dq = i_d + 1j * i_q
        return i_s_dq

    def psi_s_dq(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the stator current (A)."""
        psi_d = self.L_d * np.real(i_s_dq) + self.psi_f
        psi_q = self.L_q * np.imag(i_s_dq)
        psi_s_dq = psi_d + 1j * psi_q
        return psi_s_dq

    def incr_ind_mat(self, i_s_dq: complex | np.ndarray) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        return np.array([[self.L_d, 0], [0, self.L_q]])

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        """Compute the current from the flux linkage in closed form."""
        return complex(self.i_s_dq(psi_s_dq))

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        i_s_dq = self.i_s_dq(psi_s_dq)
        return i_s_dq, _torque(i_s_dq, psi_s_dq)


# %%
@dataclass
class SaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine.

    The saturation model is specified as a current map (current as a function of the
    flux linkage). Optionally, to be used only in control systems, a flux map (flux
    linkage as a function of the current) can be provided. For convenience, this class
    also provides the incremental inductance matrix, which can be used in control and
    optimal reference generation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    i_s_dq_fcn : Callable[[complex], complex], optional
        Stator current (A) as a function of the stator flux linkage (Vs). Needed in the
        system model and in some control methods.
    psi_s_dq_fcn : Callable[[complex], complex], optional
        Stator flux linkage (Vs) as a function of the stator current (A). This function
        should be differentiable, if incremental inductances are used. Needed only for
        control methods and optimal reference loci, not used in the system model.
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    """

    n_p: int
    R_s: float
    i_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    psi_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    G_c: float = 0.0
    psi_f: float = field(init=False, default=0.0)
    L_d0: float = field(init=False, default=np.nan)
    L_q0: float = field(init=False, default=np.nan)

    def __post_init__(self) -> None:
        if self.i_s_dq_fcn is not None:
            self.psi_f = _solve_pm_flux(
                lambda psi_d: float(np.real(self.i_s_dq(psi_d)))
            )
        elif self.psi_s_dq_fcn is not None:
            self.psi_f = complex(self.psi_s_dq_fcn(0j)).real
            # Following are needed only for iterative current computation, if used
            self.L_d0 = self.incr_ind_mat(0j)[0, 0]
            self.L_q0 = self.incr_ind_mat(0j)[1, 1]
        else:
            raise ValueError("Either i_s_dq_fcn or psi_s_dq_fcn must be provided")
        if self.psi_f < EPS:  # No permanent magnets
            self.psi_f = 0.0

    def i_s_dq(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        if self.i_s_dq_fcn is not None:
            return self.i_s_dq_fcn(psi_s_dq)
        else:
            raise ValueError("i_s_dq_fcn must be provided")

    def psi_s_dq(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the stator current (A)."""
        if self.psi_s_dq_fcn is not None:
            return self.psi_s_dq_fcn(i_s_dq)
        else:
            raise ValueError("psi_s_dq_fcn must be provided")

    def incr_ind_mat(self, i_s_dq: complex | np.ndarray) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        # Use the Jacobian of the flux map, if available (e.g., GradNet flux maps)
        if (jacobian := getattr(self.psi_s_dq_fcn, "jacobian", None)) is not None:
            return np.moveaxis(jacobian(i_s_dq), (-2, -1), (0, 1))
        pts = np.array(
            [i_s_dq + EPS, i_s_dq - EPS, i_s_dq + 1j * EPS, i_s_dq - 1j * EPS]
        )
        psi = cast(np.ndarray, self.psi_s_dq(pts))
        psi_dev_d = psi[0] - psi[1]
        psi_dev_q = psi[2] - psi[3]
        L_dd = np.real(psi_dev_d) / (2 * EPS)
        L_qq = np.imag(psi_dev_q) / (2 * EPS)
        L_dq = np.real(psi_dev_q) / (2 * EPS)
        return np.array([[L_dd, L_dq], [L_dq, L_qq]])

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        """
        Compute the current from the flux linkage.

        The current map is used if available. Otherwise, the current is computed
        iteratively from the flux map using a root-finding algorithm, which is less
        efficient but may be convenient in some special cases.

        """
        if self.i_s_dq_fcn is not None:
            return complex(self.i_s_dq(psi_s_dq))

        def error(x: list[float]) -> list[float]:
            i_s = x[0] + 1j * x[1]
            err = complex(self.psi_s_dq(i_s)) - psi_s_dq
            return [err.real, err.imag]

        def jac(x: list[float]) -> np.ndarray:
            return self.incr_ind_mat(x[0] + 1j * x[1])

        i_s0 = (psi_s_dq.real - self.psi_f) / self.L_d0 + 1j * psi_s_dq.imag / self.L_q0
        sol = root(error, [i_s0.real, i_s0.imag], jac=jac, method="hybr")
        # The residual is checked instead of sol.success, since the solver reports no
        # progress at the rounding error, e.g., of single-precision maps
        if np.hypot(*sol.fun) > 1e-5 * max(abs(psi_s_dq), 0.1):
            raise ValueError(f"Flux linkage {psi_s_dq} not invertible: {sol.message}")
        return sol.x[0] + 1j * sol.x[1]

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        i_s_dq = self.i_s_dq(psi_s_dq)
        return i_s_dq, _torque(i_s_dq, psi_s_dq)


# %%
@dataclass
class SpatialSaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine with spatial harmonics.

    This saturation model contains spatial harmonics in addition to the saturation
    effects. This version is intended to parametrize high-fidelity machine models for
    simulation purposes, while control methods do not support spatial harmonics.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    magnetic_map_fcn : Callable[[complex, complex], tuple[complex, float]]
        Stator current (A) and electromagnetic torque (Nm) per pole pair as functions of
        the stator flux linkage (Vs) and the complex exponential of the electrical rotor
        angle.
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    """

    n_p: int
    R_s: float
    magnetic_map_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray],
        tuple[complex | np.ndarray, float | np.ndarray],
    ]
    G_c: float = 0.0
    psi_f: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        self.psi_f = _solve_pm_flux(
            lambda psi_d: float(np.real(self.magnetic_map_fcn(psi_d, 1.0)[0]))
        )
        if self.psi_f < EPS:  # No permanent magnets
            self.psi_f = 0.0

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        return self.magnetic_map_fcn(psi_s_dq, exp_j_theta_m)


# %%
class CageImpedance:
    """
    Impedance of the rotor cage as a ladder circuit.

    The impedance of the cage in rotor coordinates is::

        Z(s) = R[0] + 1/(1/(s*L[0]) + 1/(R[1] + 1/(1/(s*L[1]) + ...)))

    i.e., a ladder of series resistances and shunt inductances, terminated with the
    last resistance [#Mol2020]_. Hence, ``R[0]`` is the DC resistance and ``L[0]`` is
    the DC inductance of the cage. The ladder models the deep-bar effect, i.e., the
    rotor resistance increasing and the rotor leakage inductance decreasing with the
    frequency of the rotor current. Without inductances, the impedance is the
    resistance ``R[0]``. The flux linkages of the inductances are the states.

    Parameters
    ----------
    R : Sequence[float]
        Series resistances (Ω), one more than the inductances. The first one is
        nonnegative and the others are positive.
    L : Sequence[float], optional
        Shunt inductances (H), positive, defaults to none.

    References
    ----------
    .. [#Mol2020] Mölsä, Saarakkala, Hinkkanen, Arkkio, Routimo, "A dynamic model for
       saturated induction machines with closed rotor slots and deep bars," IEEE Trans.
       Energy Convers., 2020, https://doi.org/10.1109/TEC.2019.2950810

    """

    def __init__(
        self, R: Sequence[float] | np.ndarray, L: Sequence[float] | np.ndarray = ()
    ) -> None:
        self.R = np.array(R, dtype=float)
        self.L = np.array(L, dtype=float)
        if self.R.ndim != 1 or self.L.ndim != 1 or self.R.size != self.L.size + 1:
            raise ValueError("R must have one more element than L")
        R = self.R
        if not (np.isfinite(R).all() and R[0] >= 0 and (R[1:] > 0).all()):
            raise ValueError("R must be finite, R[0] nonnegative, and R[1:] positive")
        if not (np.isfinite(self.L).all() and (self.L > 0).all()):
            raise ValueError("L must be finite and positive")

    @classmethod
    def from_rectangular_bars(cls, R_0: float, L_0: float, N: int) -> "CageImpedance":
        """
        Ladder of rectangular rotor bars.

        The elements are ``R[n] = (4*n + 1)*R_0`` and ``L[n] = 3*L_0/(4*n + 3)``
        [#Mol2020]_. The impedance of the end rings is omitted.

        Parameters
        ----------
        R_0 : float
            DC resistance (Ω).
        L_0 : float
            DC inductance (H).
        N : int
            Number of the inductances, i.e., the order of the ladder.

        Returns
        -------
        CageImpedance
            Ladder of the order `N`.

        """
        n = np.arange(N + 1)
        return cls(R=(4 * n + 1) * R_0, L=3 * L_0 / (4 * n[:N] + 3))

    def Z(self, s: complex | np.ndarray) -> complex | np.ndarray:
        """
        Impedance.

        Parameters
        ----------
        s : complex | ndarray
            Complex frequency (rad/s).

        Returns
        -------
        complex | ndarray
            Impedance (Ω).

        """
        Z = self.R[-1] + 0 * s
        for R_n, L_n in zip(self.R[-2::-1], self.L[::-1], strict=True):
            Z = R_n + s * L_n * Z / (s * L_n + Z)
        return Z

    def rhs(
        self, psi_c: np.ndarray, i_r: complex
    ) -> tuple[complex, Sequence[complex] | np.ndarray]:
        """
        Compute the voltage and the state derivatives.

        Parameters
        ----------
        psi_c : ndarray
            Flux linkages (Vs) of the inductances.
        i_r : complex
            Rotor current (A).

        Returns
        -------
        tuple[complex, Sequence[complex] | ndarray]
            Voltage (V) across the impedance and time derivatives of `psi_c`.

        """
        R, L = self.R, self.L
        if not L.size:
            return float(R[0]) * i_r, ()
        # Currents of the resistances R[1], R[2], ..., and the voltages across the
        # inductances, each being the sum of the resistive voltages behind it
        i = i_r - np.cumsum(psi_c / L)
        d_psi_c = np.cumsum((R[1:] * i)[::-1])[::-1]
        return R[0] * i_r + d_psi_c[0], d_psi_c


# %%
class BaseInductionMachinePars(Protocol):
    """
    Base class for induction machine parameters, as used by the machine model.

    The machine model consists of the stator resistance, the magnetic model, and the
    rotor cage. The magnetic model maps the stator and rotor flux linkages to the stator
    and rotor currents. The rotor quantities can be scaled freely, e.g., as in the Γ
    model or in the inverse-Γ model, provided that the magnetic map and the cage
    impedance use the same scaling.

    """

    n_p: int
    R_s: float
    G_c: float

    @property
    def cage(self) -> CageImpedance:
        """Impedance of the rotor cage, which the machine model reads when created."""
        ...

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        psi_r_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray]:
        """
        Magnetic map as a function of flux linkages and electrical rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        psi_r_dq : complex | ndarray
            Rotor flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        tuple[complex | ndarray, complex | ndarray, float | ndarray]
            Stator current (A) and rotor current (A) in rotor coordinates, both positive
            into the windings, and electromagnetic torque (Nm) per pole pair.

        """
        ...


def _cage_impedance(R_r: float | CageImpedance) -> CageImpedance:
    """Cage impedance given as a resistance or as an impedance."""
    return R_r if isinstance(R_r, CageImpedance) else CageImpedance([R_r])


def _dc_resistance(R_r: float | CageImpedance) -> float:
    """DC resistance of the cage given as a resistance or as an impedance."""
    return float(R_r.R[0]) if isinstance(R_r, CageImpedance) else R_r


def _dc_inductance(R_r: float | CageImpedance) -> float:
    """DC inductance of the cage given as a resistance or as an impedance."""
    return float(R_r.L[0]) if isinstance(R_r, CageImpedance) and R_r.L.size else 0.0


# %%
@dataclass
class InductionMachineInvGammaPars(BaseInductionMachinePars):
    """
    Inverse-Γ model parameters of an induction machine.

    These constant parameters can be used both in control systems and in the machine
    model. The magnetic model is::

        psi_s = L_sgm*i_s + psi_R
        psi_R = L_M*(i_s + i_R)

    where the rotor flux linkage `psi_R` and the rotor current `i_R` are scaled such
    that the leakage inductance is on the stator side.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_R : float
        Rotor resistance (Ω).
    L_sgm : float
        Leakage inductance (H).
    L_M : float
        Magnetizing inductance (H).
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    Attributes
    ----------
    alpha : float
        Inverse rotor time constant (rad/s).
    w_rb : float
        Breakdown slip angular frequency (rad/s).

    """

    n_p: int
    R_s: float
    R_R: float
    L_sgm: float
    L_M: float
    G_c: float = 0.0

    def update_psi_s(self, psi_s: float) -> None:
        """Update the stator flux linkage magnitude, without effect here."""

    @property
    def alpha(self) -> float:
        return self.R_R / self.L_M

    @property
    def w_rb(self) -> float:
        return self.R_R * (1 / self.L_sgm + 1 / self.L_M) if self.L_sgm > 0 else 0.0

    @property
    def cage(self) -> CageImpedance:
        """Impedance of the rotor cage, i.e., the rotor resistance."""
        return CageImpedance([self.R_R])

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        psi_r_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray]:
        """Magnetic map, the rotor quantities being `psi_R` and `i_R`."""
        i_s_dq = (psi_s_dq - psi_r_dq) / self.L_sgm
        i_r_dq = psi_r_dq / self.L_M - i_s_dq
        return i_s_dq, i_r_dq, _torque(i_s_dq, psi_s_dq)


# %%
@dataclass
class InductionMachinePars(BaseInductionMachinePars):
    """
    Γ-model parameters of an induction machine, with optional main-flux saturation and
    deep-bar effect.

    These parameters can be used both in control systems and in the machine model. The
    magnetic model is [#Sle1989]_::

        psi_s = L_s*(i_s + i_r)
        psi_r = psi_s + L_ell*i_r

    The main-flux saturation is modeled by providing a callable `L_s` and the deep-bar
    effect by providing the cage impedance as `R_r`. For control systems, the class
    provides the corresponding inverse-Γ model parameters, which then depend on the
    stator flux linkage magnitude given to `update_psi_s`. With the cage impedance, they
    are those of its low-frequency equivalent, i.e., the DC resistance of the cage in
    series with its DC inductance, which adds to the leakage inductance.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float | CageImpedance
        Rotor resistance (Ω) or, for modeling the deep-bar effect, the cage impedance.
    L_ell : float
        Leakage inductance (H).
    L_s : float | Callable[[float], float]
        Stator inductance (H). If callable, it should be a function of the stator flux
        linkage magnitude (Vs).
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).
    psi_s : float, optional
        Stator flux linkage magnitude (Vs), at which the inverse-Γ model parameters are
        evaluated, defaults to 0.

    Attributes
    ----------
    gamma : float
        Magnetic coupling factor.
    L_M : float
        Inverse-Γ magnetizing inductance (H).
    L_sgm : float
        Inverse-Γ leakage inductance (H).
    R_R : float
        Inverse-Γ rotor resistance (Ω).
    alpha : float
        Inverse rotor time constant (rad/s).
    w_rb : float
        Breakdown slip angular frequency (rad/s).

    References
    ----------
    .. [#Sle1989] Slemon, "Modelling of induction machines for electric drives," IEEE
       Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251

    """

    n_p: int
    R_s: float
    R_r: float | CageImpedance
    L_ell: float
    L_s: float | Callable[[float], float]
    G_c: float = 0.0
    psi_s: float = 0.0

    def update_psi_s(self, psi_s: float) -> None:
        """Update the stator flux linkage magnitude."""
        self.psi_s = psi_s

    @property
    def _L_ell_dc(self) -> float:
        """Leakage inductance (H) including the DC inductance of the cage."""
        return self.L_ell + _dc_inductance(self.R_r)

    @property
    def gamma(self) -> float:
        L_s = get_value(self.L_s, self.psi_s)
        return L_s / (L_s + self._L_ell_dc)

    @property
    def L_M(self) -> float:
        return self.gamma * get_value(self.L_s, self.psi_s)

    @property
    def L_sgm(self) -> float:
        return self.gamma * self._L_ell_dc

    @property
    def R_R(self) -> float:
        return self.gamma**2 * _dc_resistance(self.R_r)

    @property
    def alpha(self) -> float:
        return self.R_R / self.L_M

    @property
    def w_rb(self) -> float:
        L_ell = self._L_ell_dc
        return _dc_resistance(self.R_r) / L_ell if L_ell > 0 else 0.0

    @property
    def cage(self) -> CageImpedance:
        """Impedance of the rotor cage."""
        return _cage_impedance(self.R_r)

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        psi_r_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        i_r_dq = (psi_r_dq - psi_s_dq) / self.L_ell
        i_s_dq = psi_s_dq / get_value(self.L_s, abs(psi_s_dq)) - i_r_dq
        return i_s_dq, i_r_dq, _torque(i_s_dq, psi_s_dq)


# %%
@dataclass
class SaturatedInductionMachinePars(BaseInductionMachinePars):
    """
    Parameters of an induction machine with a general magnetic model.

    The magnetic model is specified as a current map (stator and rotor currents as
    functions of the stator and rotor flux linkages), which can include the saturation
    of the main flux and of the leakage flux as well as their mutual effects. This
    version is intended to parametrize machine models for simulation purposes, while
    control methods do not support it.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float | CageImpedance
        Rotor resistance (Ω) or, for modeling the deep-bar effect, the cage impedance.
    current_map_fcn : Callable[[complex, complex], tuple[complex, complex]]
        Stator current (A) and rotor current (A) as functions of the stator flux linkage
        (Vs) and the rotor flux linkage (Vs). The quantities are complex space vectors
        in any common coordinates, and both currents are positive into the windings. The
        function should also accept arrays.
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    Notes
    -----
    The magnetic model is lossless, if the currents are the gradient of a scalar
    function `W` of the flux linkages (the field energy multiplied by 2/3) with respect
    to their real and imaginary parts. The Jacobian of the map is then symmetric
    (reciprocity condition). Furthermore, `W` should be invariant under a common
    rotation of the flux linkages, in which case the torque computed from the stator
    quantities equals the torque computed from the rotor quantities. These conditions
    are not checked. As an example, the map of the Γ model with constant inductances is
    the gradient of::

        W = abs(psi_s)**2/(2*L_s) + abs(psi_r - psi_s)**2/(2*L_ell)

    Examples
    --------
    The mutual saturation model in [#Tuo2010]_ fulfills the above conditions. Its
    per-unit parameters below are fitted to the experimental data of a 2.2-kW machine,
    with the exponents ``b = 1``, ``c = 1``, and ``d = 0``:

    >>> from motulator.drive.model import SaturatedInductionMachinePars
    >>> from motulator.drive.utils import BaseValues, NominalValues
    >>> base = BaseValues.from_nominal(NominalValues(U=400, I=5, f=50, P=2.2e3))
    >>> L_su, L_ellu, alpha, beta, gamma, a = 2.28, 0.216, 0.383, 0.511, 3.20, 7.5
    >>> def current_map(psi_s, psi_r):
    ...     psi_ell = psi_r - psi_s  # Leakage flux linkage
    ...     x, y = abs(psi_s) / base.psi, abs(psi_ell) / base.psi
    ...     k_m = 1 + alpha * x**a + gamma * L_su / 2 * x * y**2
    ...     k_r = 1 + beta * y + gamma * L_ellu / 3 * x**3
    ...     i_m = k_m * psi_s / (L_su * base.L)  # Magnetizing current
    ...     i_r = k_r * psi_ell / (L_ellu * base.L)
    ...     return i_m - i_r, i_r
    >>> par = SaturatedInductionMachinePars(
    ...     n_p=2,
    ...     R_s=0.0628 * base.Z,
    ...     R_r=0.0433 * base.Z,
    ...     current_map_fcn=current_map,
    ... )

    References
    ----------
    .. [#Tuo2010] Tuovinen, Hinkkanen, Luomi, "Modeling of saturation due to main and
       leakage flux interaction in induction machines," IEEE Trans. Ind. Appl., 2010,
       https://doi.org/10.1109/TIA.2010.2045210

    """

    n_p: int
    R_s: float
    R_r: float | CageImpedance
    current_map_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray],
        tuple[complex | np.ndarray, complex | np.ndarray],
    ]
    G_c: float = 0.0

    @property
    def cage(self) -> CageImpedance:
        """Impedance of the rotor cage."""
        return _cage_impedance(self.R_r)

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        psi_r_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        i_s_dq, i_r_dq = self.current_map_fcn(psi_s_dq, psi_r_dq)
        return i_s_dq, i_r_dq, _torque(i_s_dq, psi_s_dq)


# %%
@dataclass
class SpatialSaturatedInductionMachinePars(BaseInductionMachinePars):
    """
    Parameters of a saturated induction machine with spatial harmonics.

    The magnetic model contains spatial harmonics (e.g., due to the slots) in addition
    to the saturation effects, i.e., it depends on the rotor angle. This version is
    intended to parametrize high-fidelity machine models for simulation purposes, while
    control methods do not support spatial harmonics.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float | CageImpedance
        Rotor resistance (Ω) or, for modeling the deep-bar effect, the cage impedance.
    magnetic_map_fcn : Callable[[complex, complex, complex], \
            tuple[complex, complex, float]]
        Stator current (A), rotor current (A), and electromagnetic torque (Nm) per pole
        pair as functions of the stator flux linkage (Vs), the rotor flux linkage (Vs),
        and the complex exponential of the electrical rotor angle. The space vectors are
        in rotor coordinates, and both currents are positive into the windings. The
        function should also accept arrays.
    G_c : float, optional
        Core-loss conductance (S), connected between the stator resistance and the
        magnetic model, defaults to 0 (no core losses).

    Notes
    -----
    The magnetic model is lossless, if the currents are the gradient of a scalar
    function `W` of the flux linkages and the electrical rotor angle `theta_m` (the
    field energy multiplied by 2/3) with respect to the real and imaginary parts of the
    flux linkages, and the torque per pole pair is::

        tau_m = 3/2*(imag(i_s*conj(psi_s)) - dW/dtheta_m)

    These conditions are not checked.

    """

    n_p: int
    R_s: float
    R_r: float | CageImpedance
    magnetic_map_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray, complex | np.ndarray],
        tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray],
    ]
    G_c: float = 0.0

    @property
    def cage(self) -> CageImpedance:
        """Impedance of the rotor cage."""
        return _cage_impedance(self.R_r)

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        psi_r_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> tuple[complex | np.ndarray, complex | np.ndarray, float | np.ndarray]:
        """Magnetic map."""
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        return self.magnetic_map_fcn(psi_s_dq, psi_r_dq, exp_j_theta_m)
