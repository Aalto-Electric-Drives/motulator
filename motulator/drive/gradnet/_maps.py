"""
Trained GradNets as numpy functions and the magnetic maps based on them.

A trained network is evaluated in double precision with numpy, so the maps need no
PyTorch. The function `GradNetFunction` holds the parameters of the network, and it is
saved to and loaded from an npz file. The networks are trained with the related
package motulator-gradnet.

"""

from collections.abc import Callable
from dataclasses import dataclass, fields
from pathlib import Path
from typing import cast

import numpy as np

from motulator.drive.utils._sm_flux_maps import SaturationModelBase

# Activation function: the value at z, shape (..., n), and its Jacobian, shape
# (..., n, n), if requested, with the scale beta and the exponent p of the p-norm
Activation = Callable[
    [np.ndarray, bool, float, int], tuple[np.ndarray, np.ndarray | None]
]

# Signs conjugating a (d, q) pair, and a pair with the (cos, sin) inputs
_CONJ = np.array([1.0, -1.0])
_CONJ2 = np.array([1.0, -1.0, 1.0, -1.0])


def _diag(d: np.ndarray) -> np.ndarray:
    """Diagonal matrices, shape (..., n, n), from the diagonals, shape (..., n)."""
    return d[..., :, None] * np.eye(d.shape[-1])


# %%
def softmax(
    z: np.ndarray, jacobian: bool, beta: float, p: int
) -> tuple[np.ndarray, np.ndarray | None]:
    """Softmax activation softmax(beta*z) along the last axis."""
    s = np.exp(beta * (z - z.max(axis=-1, keepdims=True)))
    s /= s.sum(axis=-1, keepdims=True)
    if not jacobian:
        return s, None
    return s, beta * (_diag(s) - s[..., :, None] * s[..., None, :])


def pnorm_gradient(
    z: np.ndarray, jacobian: bool, beta: float, p: int
) -> tuple[np.ndarray, np.ndarray | None]:
    """Gradient of the smooth p-norm (1 + sum((beta*z)**p))**(1/p)/beta."""
    q = p - 1
    z = beta * z
    norm = 1 + np.sum(z ** (q + 1), axis=-1, keepdims=True)
    z_q = z**q
    y = z_q / norm ** (q / (q + 1))
    if not jacobian:
        return y, None
    outer = z_q[..., :, None] * z_q[..., None, :] / norm[..., None]
    scale = (beta * q * norm ** (-q / (q + 1)))[..., None]
    return y, scale * (_diag(z ** (q - 1)) - outer)


def squareplus(
    z: np.ndarray, jacobian: bool, beta: float, p: int
) -> tuple[np.ndarray, np.ndarray | None]:
    """Squareplus activation (z + sqrt(z**2 + beta))/2."""
    r = np.sqrt(z**2 + beta)
    return 0.5 * (z + r), _diag(0.5 * (1 + z / r)) if jacobian else None


def algebraic_sigmoid(
    z: np.ndarray, jacobian: bool, beta: float, p: int
) -> tuple[np.ndarray, np.ndarray | None]:
    """Algebraic sigmoid activation z/sqrt(z**2 + beta)."""
    r = np.sqrt(z**2 + beta)
    return z / r, _diag(beta / r**3) if jacobian else None


# Activations by name, the contract with the related package motulator-gradnet
ACTIVATIONS: dict[str, Activation] = {
    "Softmax": softmax,
    "PNormGradient": pnorm_gradient,
    "Squareplus": squareplus,
    "AlgebraicSigmoid": algebraic_sigmoid,
}


# %%
@dataclass
class GradNetFunction:
    """
    Trained GradNet as a numpy function in double precision.

    The function maps the per-unit input, shape (..., in_dim), to the per-unit output
    of the same shape, which is the gradient of the scalar state function of the
    network. It is created from a trained model by the related package
    motulator-gradnet, saved to an npz file by `save`, and loaded by `load_gradnet`.

    Parameters
    ----------
    W : ndarray, shape (num_modules, embed_dim, in_dim)
        Weights of the modules.
    b : ndarray, shape (num_modules, embed_dim)
        Biases of the modules.
    beta_log : ndarray, shape (num_modules,)
        Logarithms of the scales of the activations of the modules.
    mu_log : ndarray, shape (mu_dim,)
        Logarithms of the coefficients of the linear term.
    bias : ndarray, shape (in_dim,)
        Bias of the output.
    activation : str
        Name of the activation function, see `ACTIVATIONS`.
    p : int
        Exponent of the p-norm gradient activation (0 for the other activations).
    psi_base : float
        Base value of the flux linkage (Vs).
    i_base : float
        Base value of the current (A).

    """

    W: np.ndarray
    b: np.ndarray
    beta_log: np.ndarray
    mu_log: np.ndarray
    bias: np.ndarray
    activation: str
    p: int
    psi_base: float
    i_base: float

    def __post_init__(self) -> None:
        self._act = ACTIVATIONS[self.activation]
        self._beta = np.exp(self.beta_log)
        self._mu = np.pad(np.exp(self.mu_log), (0, self.bias.size - self.mu_log.size))

    def save(self, path: Path | str) -> None:
        """Save the parameters to an npz file."""
        np.savez(path, **{f.name: getattr(self, f.name) for f in fields(self)})

    def __call__(
        self, x: np.ndarray, jacobian: bool = False
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """
        Evaluate the function.

        Parameters
        ----------
        x : ndarray, shape (..., in_dim)
            Per-unit input.
        jacobian : bool, optional
            Compute also the Jacobian matrix, defaults to False.

        Returns
        -------
        y : ndarray, shape (..., in_dim)
            Per-unit output.
        jac : ndarray, shape (..., in_dim, in_dim) | None
            Jacobian matrix of the output with respect to the input (the Hessian of
            the underlying scalar state function), if requested.

        """
        y = self._mu * x + self.bias
        jac = _diag(np.broadcast_to(self._mu, x.shape)) if jacobian else None
        for W, b, beta in zip(self.W, self.b, self._beta, strict=True):
            h, jac_act = self._act(x @ W.T + b, jacobian, beta, self.p)
            y = y + h @ W
            if jac is not None:
                jac = jac + W.T @ jac_act @ W
        return y, jac


def load_gradnet(model_path: Path | str) -> GradNetFunction:
    """
    Load a trained GradNet from an npz file, see `GradNetFunction`.

    Parameters
    ----------
    model_path : Path | str
        Path to the saved model file.

    Returns
    -------
    GradNetFunction
        Trained network as a numpy function.

    """
    with np.load(model_path) as data:
        par = {
            name: value.item() if value.ndim == 0 else value
            for name, value in data.items()
        }
    return GradNetFunction(**par)


# %%
class CurrentMap(SaturationModelBase):
    """
    Callable wrapper for GradNet current map models.

    The map is symmetrized about the d-axis to ensure physical consistency. The map can
    be sampled on a grid by `as_magnetic_model()`, e.g., for plotting.

    Parameters
    ----------
    model : GradNetFunction
        Trained GradNet for the current map, see `load_gradnet`.

    """

    def __init__(self, model: GradNetFunction) -> None:
        super().__init__(map_type="current_map")
        self.fcn = model
        self.in_base, self.out_base = self.fcn.psi_base, self.fcn.i_base

    def _evaluate(
        self, x_dq: complex | np.ndarray, jacobian: bool
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """Symmetrized value and Jacobian of the map in per-unit quantities."""
        x = np.asarray(x_dq) / self.in_base
        xy = np.stack([x.real, x.imag], axis=-1)
        # Evaluate at the input and its conjugate
        y, jac = self.fcn(np.stack([xy, xy * _CONJ]), jacobian)
        y = 0.5 * (y[0] + y[1] * _CONJ)
        if jac is not None:
            jac = 0.5 * (jac[0] + _CONJ[:, None] * jac[1] * _CONJ[None, :])
        return y[..., 0] + 1j * y[..., 1], jac

    def __call__(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """
        Evaluate the stator current at given stator flux linkage.

        Parameters
        ----------
        psi_s_dq : complex | np.ndarray
            Stator flux linkage (Vs).

        Returns
        -------
        complex | np.ndarray
            Stator current (A).

        """
        i_s_dq = self.out_base * self._evaluate(psi_s_dq, jacobian=False)[0]
        return i_s_dq.item() if i_s_dq.size == 1 else i_s_dq

    def jacobian(self, x_dq: complex | np.ndarray) -> np.ndarray:
        """
        Jacobian matrix of the symmetrized map.

        Parameters
        ----------
        x_dq : complex | np.ndarray
            Input of the map (flux linkage in Vs or current in A).

        Returns
        -------
        np.ndarray, shape (..., 2, 2)
            Jacobian matrix [[dy_d/dx_d, dy_d/dx_q], [dy_q/dx_d, dy_q/dx_q]], where y is
            the output of the map, in SI units.

        """
        jac = cast(np.ndarray, self._evaluate(x_dq, jacobian=True)[1])
        return jac * (self.out_base / self.in_base)


# %%
class FluxMap(CurrentMap):
    """
    Callable wrapper for GradNet flux-linkage map models.

    The map returns the stator flux linkage (Vs) at the given stator current (A). It
    is symmetrized about the d-axis to ensure physical consistency.

    Parameters
    ----------
    model : GradNetFunction
        Trained GradNet for the flux-linkage map, see `load_gradnet`.

    """

    def __init__(self, model: GradNetFunction) -> None:
        super().__init__(model)
        self._type = "flux_map"
        self.in_base, self.out_base = self.fcn.i_base, self.fcn.psi_base


# %%
class CurrentMapWithHarmonics:
    """
    Callable wrapper for GradNet current maps with spatial harmonics.

    Parameters
    ----------
    model : GradNetFunction
        Trained GradNet for the current map with harmonics, see `load_gradnet`.
    k : int, optional
        Spatial harmonic order, defaults to 6.

    """

    def __init__(self, model: GradNetFunction, k: int = 6) -> None:
        self.fcn = model
        self.i_base, self.psi_base = self.fcn.i_base, self.fcn.psi_base
        self.tau_base = 1.5 * self.psi_base * self.i_base
        self.in_base, self.out_base = self.psi_base, self.i_base
        # Harmonic order
        self.k = k

    def _torque(
        self, x: np.ndarray, y: np.ndarray, dW_dtheta: np.ndarray
    ) -> np.ndarray:
        """Torque (p.u.) from the flux linkage x, current y, and the magnetic energy."""
        return np.imag(y * x.conj()) - dW_dtheta

    def __call__(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m: complex | np.ndarray
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """
        Evaluate the current and torque at given flux linkage and rotor position.

        Parameters
        ----------
        psi_s_dq : complex | np.ndarray
            Stator flux linkage (Vs).
        exp_j_theta_m : complex | np.ndarray
            Exponential of the rotor electrical angle.

        Returns
        -------
        tuple[complex | np.ndarray, float | np.ndarray]
            Stator current (A) and electromagnetic torque (Nm) per pole pair.

        """
        # Per-unit input and the rotor-position input exp(j*k*theta_m) as (d, q, cos,
        # sin), evaluated together with their conjugates
        x, exp_j_k_theta = np.broadcast_arrays(
            np.asarray(psi_s_dq) / self.in_base, np.asarray(exp_j_theta_m) ** self.k
        )
        xy = np.stack(
            [np.real(x), np.imag(x), np.real(exp_j_k_theta), np.imag(exp_j_k_theta)],
            axis=-1,
        )
        y, _ = self.fcn(np.stack([xy, xy * _CONJ2]))

        # Symmetrize the output and dW/dcos + j*dW/dsin of the state function W
        y = 0.5 * (y[0] + y[1] * _CONJ2)
        y, dW = y[..., 0] + 1j * y[..., 1], y[..., 2] + 1j * y[..., 3]

        # Torque in per-unit
        dW_dtheta = self.k * (exp_j_k_theta.conj() * dW).imag
        tau_m = self._torque(x, y, dW_dtheta)

        # Scale back to physical units
        y = self.out_base * y
        tau_m = self.tau_base * tau_m
        return (y.item(), tau_m.item()) if y.size == 1 else (y, tau_m)


# %%
class FluxMapWithHarmonics(CurrentMapWithHarmonics):
    """
    Callable wrapper for GradNet flux maps with spatial harmonics.

    Parameters
    ----------
    model : GradNetFunction
        Trained GradNet for the flux map with harmonics, see `load_gradnet`.
    k : int, optional
        Spatial harmonic order, defaults to 6.

    """

    def __init__(self, model: GradNetFunction, k: int = 6) -> None:
        super().__init__(model, k)
        self.in_base, self.out_base = self.i_base, self.psi_base

    def _torque(
        self, x: np.ndarray, y: np.ndarray, dW_dtheta: np.ndarray
    ) -> np.ndarray:
        """Torque (p.u.) from the current x, flux linkage y, and the co-energy."""
        return np.imag(x * y.conj()) + dW_dtheta

    def __call__(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m: complex | np.ndarray
    ) -> tuple[complex | np.ndarray, float | np.ndarray]:
        """
        Evaluate the flux linkage and torque at given current and rotor position.

        Parameters
        ----------
        i_s_dq : complex | np.ndarray
            Stator current (A).
        exp_j_theta_m : complex | np.ndarray
            Exponential of the rotor electrical angle.

        Returns
        -------
        tuple[complex | np.ndarray, float | np.ndarray]
            Stator flux linkage (Vs) and electromagnetic torque (Nm) per pole pair.

        """
        return super().__call__(i_s_dq, exp_j_theta_m)
