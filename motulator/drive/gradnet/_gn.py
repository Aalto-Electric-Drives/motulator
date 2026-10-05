"""
Gradient networks (GradNets) for magnetics modeling.

This module contains GradNet architecture to model the current and flux linkage maps of
synchronous machines [#Li2026]_. The GradNets allow modeling conservative vector fields
by construction [#Cha2025]_. In our case, the scalar state function is either the
magnetic energy or co-energy, depending on whether the current map or flux map is
modeled. The monotonicity of the flux-linkage--current map is also ensured.

References
----------
.. [#Li2026] Li, Foissner, Martin, Piippo, Hinkkanen, "Gradient networks for universal
   magnetic modeling of synchronous machines," 2026, https://arxiv.org/abs/2602.14947

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient networks," IEEE Trans. Signal
   Process., 2025, https://doi.org/10.1109/TSP.2024.3496692

"""

from collections.abc import Callable
from pathlib import Path
from typing import Any, cast

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn
from torch.nn import init


# %%
class Softmax(nn.Module):
    """Softmax activation function."""

    def __init__(
        self, dim: int = -1, beta_log0: float = 2.0, freeze_beta: bool = False
    ) -> None:
        super().__init__()
        self.dim = dim
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))
        if freeze_beta:
            self.beta_log.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return F.softmax(torch.exp(self.beta_log) * x, dim=self.dim)

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """Jacobian matrix, shape (..., n, n), for inputs of shape (..., n)."""
        beta = torch.exp(self.beta_log)
        s = F.softmax(beta * x, dim=-1)
        return beta * (torch.diag_embed(s) - s.unsqueeze(-1) * s.unsqueeze(-2))


# %%
class PNormGradient(nn.Module):
    """
    p-norm gradient activation function.

    Defined as the gradient of S(z) = (1 + sum(z_n**p))**(1/p)/beta, where p is a
    positive even integer. This potential function corresponds to a smooth p-norm, which
    is convex, thus guaranteeing monotonicity.

    """

    def __init__(
        self,
        dim: int = -1,
        p: int = 8,
        beta_log0: float = 0.0,
        freeze_beta: bool = False,
    ) -> None:
        super().__init__()
        if p < 2 or p % 2 != 0:
            raise ValueError("p must be a positive even integer")
        self.dim = dim
        self.q = p - 1
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))
        if freeze_beta:
            self.beta_log.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = torch.exp(self.beta_log) * x
        norm = 1 + torch.sum(x.pow(self.q + 1), dim=self.dim, keepdim=True)
        norm = norm.pow(self.q / (self.q + 1))
        return x.pow(self.q) / norm

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """Jacobian matrix, shape (..., n, n), for inputs of shape (..., n)."""
        beta = torch.exp(self.beta_log)
        x = beta * x
        q = self.q
        norm = 1 + torch.sum(x.pow(q + 1), dim=-1, keepdim=True)
        x_q = x.pow(q)
        outer = x_q.unsqueeze(-1) * x_q.unsqueeze(-2) / norm.unsqueeze(-1)
        scale = (beta * q * norm.pow(-q / (q + 1))).unsqueeze(-1)
        return scale * (torch.diag_embed(x.pow(q - 1)) - outer)


# %%
class Squareplus(nn.Module):
    """Rectifier-type activation function with one learnable parameter."""

    def __init__(self, beta_log0: float = -2.0) -> None:
        super().__init__()
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return 0.5 * (x + torch.sqrt(x**2 + torch.exp(self.beta_log)))

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """Jacobian matrix, shape (..., n, n), for inputs of shape (..., n)."""
        return torch.diag_embed(
            0.5 * (1 + x / torch.sqrt(x**2 + torch.exp(self.beta_log)))
        )


# %%
class AlgebraicSigmoid(nn.Module):
    """Sigmoid-type activation function with one learnable parameter."""

    def __init__(self, beta_log0: float = -3.0) -> None:
        super().__init__()
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x / torch.sqrt(x**2 + torch.exp(self.beta_log))

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """Jacobian matrix, shape (..., n, n), for inputs of shape (..., n)."""
        c = torch.exp(self.beta_log)
        return torch.diag_embed(c / (x**2 + c).pow(1.5))


# %%
class GradNetModule(nn.Module):
    """
    Gradient network module.

    Parameters
    ----------
    in_dim : int
        Input dimension.
    embed_dim : int
        Embedding dimension.
    activation : Callable[[], nn.Module]
        Activation factory.

    """

    def __init__(
        self, in_dim: int, embed_dim: int, activation: Callable[[], nn.Module]
    ) -> None:
        super().__init__()
        self.W = nn.Parameter(init.xavier_normal_(torch.empty(embed_dim, in_dim)))
        self.b = nn.Parameter(torch.zeros(embed_dim))
        self.act = activation()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        z = F.linear(x, weight=self.W, bias=self.b)
        z = self.act(z)
        z = F.linear(z, weight=self.W.T)
        return z

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """Jacobian matrix W^T J_act W, shape (..., in_dim, in_dim)."""
        z = F.linear(x, weight=self.W, bias=self.b)
        act = cast(Any, self.act)  # Activations provide the jacobian method
        return self.W.T @ act.jacobian(z) @ self.W


# %%
class GradNet(nn.Module):
    """
    Modular and monotonous gradient network.

    Parameters
    ----------
    in_dim : int, optional
        Input dimension, defaults to 2.
    mu_dim : int, optional
        Dimension of the linear term, defaults to 2.
    num_modules : int, optional
        Number of GradNet modules, defaults to 1.
    embed_dim : int, optional
        Embedding dimension for the GradNet modules, defaults to 12.
    activation : Callable[[], nn.Module], optional
        Activation factory used by GradNet modules, defaults to Softmax.
    mu_log0 : float, optional
        Initial value for the linear term coefficients in log-domain, defaults to 1.0.

    """

    psi_base: torch.Tensor
    i_base: torch.Tensor

    def __init__(
        self,
        in_dim: int = 2,
        mu_dim: int = 2,
        num_modules: int = 1,
        embed_dim: int = 12,
        activation: Callable[[], nn.Module] = Softmax,
        mu_log0: float = 1.0,
        psi_base: float = 1.0,
        i_base: float = 1.0,
    ) -> None:
        super().__init__()
        self.num_modules = num_modules
        self.blocks = nn.ModuleList(
            [GradNetModule(in_dim, embed_dim, activation) for i in range(num_modules)]
        )
        self.in_dim = in_dim
        self.bias = nn.Parameter(torch.zeros(in_dim))
        self.mu_log = nn.Parameter(torch.full((mu_dim,), mu_log0))
        self.non_mu_dim = in_dim - mu_dim
        # Base values for scaling
        self.register_buffer("psi_base", torch.tensor(psi_base, dtype=torch.float32))
        self.register_buffer("i_base", torch.tensor(i_base, dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Add linear term and bias
        mu = F.pad(torch.exp(self.mu_log), (0, self.non_mu_dim))
        z = mu * x + self.bias
        # Add modules
        for block in self.blocks:
            z += block(x)
        return z

    def jacobian(self, x: torch.Tensor) -> torch.Tensor:
        """
        Jacobian matrix of the output with respect to the input.

        Parameters
        ----------
        x : torch.Tensor, shape (..., in_dim)
            Input.

        Returns
        -------
        torch.Tensor, shape (..., in_dim, in_dim)
            Jacobian matrix (the Hessian of the underlying scalar state function).

        """
        mu = F.pad(torch.exp(self.mu_log), (0, self.non_mu_dim))
        jac = torch.diag(mu).expand(*x.shape[:-1], -1, -1)
        for block in self.blocks:
            jac = jac + cast(GradNetModule, block).jacobian(x)
        return jac


# %%
def load_gradnet(
    model_path: Path | str, activation: Callable[[], nn.Module] | None = None
) -> GradNet:
    """
    Load a GradNet model, inferring dimensions from the saved weights.

    Parameters
    ----------
    model_path : Path | str
        Path to the saved model file.
    activation : Callable[[], nn.Module] | None, optional
        Activation function factory, defaults to PNormGradient.

    Returns
    -------
    GradNet
        Loaded GradNet model.

    """
    # Load state dict
    state_dict = torch.load(model_path, map_location="cpu")
    # Dimensions inferred from state dict
    in_dim = int(state_dict["blocks.0.W"].shape[1])
    embed_dim = int(state_dict["blocks.0.W"].shape[0])
    mu_dim = int(state_dict["mu_log"].numel())
    num_modules = len(
        [k for k in state_dict if k.startswith("blocks.") and k.endswith(".W")]
    )
    # Choose activation
    if activation is None:
        activation = PNormGradient

    # Create and load model
    model = GradNet(in_dim, mu_dim, num_modules, embed_dim, activation)
    model.load_state_dict(state_dict, strict=False)
    model.eval()
    return model


# %%
class CurrentMap:
    """
    Callable wrapper for GradNet current map models.

    The map is symmetrized about the d-axis to ensure physical consistency.

    Parameters
    ----------
    model : GradNet
        Trained GradNet model for the current map.

    """

    def __init__(self, model: GradNet) -> None:
        self.model = model
        self.in_base = model.psi_base.item()
        self.out_base = model.i_base.item()

    def _inputs(self, x_dq: complex | np.ndarray) -> torch.Tensor:
        """Per-unit inputs and their conjugates as (d, q) pairs, shape (2, ..., 2)."""
        x = np.asarray(np.divide(x_dq, self.in_base), dtype=np.complex64)
        return torch.view_as_real(torch.from_numpy(np.array([x, x.conj()])))

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
        with torch.inference_mode():
            y = torch.view_as_complex(self.model(self._inputs(psi_s_dq))).numpy()

        # Symmetrize
        i_s_dq = 0.5 * self.out_base * (y[0] + y[1].conj())
        return i_s_dq.item() if i_s_dq.size == 1 else i_s_dq

    def jacobian(self, x_dq: complex | np.ndarray) -> np.ndarray:
        """
        Jacobian matrix of the symmetrized map.

        The Jacobian is computed analytically, which is accurate also in single
        precision (unlike finite differences).

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
        with torch.inference_mode():
            jac = self.model.jacobian(self._inputs(x_dq)).numpy().astype(float)

        # Symmetrize: the conjugation corresponds to S = diag(1, -1) on both sides
        s = np.array([1.0, -1.0])
        jac = 0.5 * (jac[0] + s[:, None] * jac[1] * s[None, :])
        return jac * (self.out_base / self.in_base)


# %%
class FluxMap(CurrentMap):
    """
    Callable wrapper for GradNet flux-linkage map models.

    The map is symmetrized about the d-axis to ensure physical consistency.

    Parameters
    ----------
    model : GradNet
        Trained GradNet model for the flux-linkage map.

    Returns
    -------
    complex | np.ndarray
        Stator flux linkage (Vs).

    """

    def __init__(self, model: GradNet) -> None:
        super().__init__(model)
        self.in_base = model.i_base.item()
        self.out_base = model.psi_base.item()


# %%
class CurrentMapWithHarmonics:
    """
    Callable wrapper for GradNet current maps with spatial harmonics.

    Parameters
    ----------
    model : GradNet
        Trained GradNet model for the current map with harmonics.
    k : int, optional
        Spatial harmonic order, defaults to 6.

    """

    def __init__(self, model: GradNet, k: int = 6) -> None:
        self.i_base = model.i_base.item()
        self.psi_base = model.psi_base.item()
        self.tau_base = 1.5 * self.psi_base * self.i_base
        self.in_base, self.out_base = self.psi_base, self.i_base
        self.model = model
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
        # sin), together with their conjugates
        x, exp_j_k_theta = np.broadcast_arrays(
            np.asarray(np.divide(psi_s_dq, self.in_base), dtype=np.complex64),
            np.asarray(np.power(exp_j_theta_m, self.k), dtype=np.complex64),
        )
        z = np.stack([x, exp_j_k_theta], axis=-1)
        inputs = torch.view_as_real(torch.from_numpy(np.array([z, z.conj()])))

        with torch.inference_mode():
            outputs = self.model(inputs.flatten(-2)).unflatten(-1, (2, 2))
            outputs = torch.view_as_complex(outputs).numpy()

        # Symmetrize the output and dW/dcos + j*dW/dsin of the state function W
        outputs = 0.5 * (outputs[0] + outputs[1].conj())
        y, dW = outputs[..., 0], outputs[..., 1]

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
    model : GradNet
        Trained GradNet model for the flux map with harmonics.
    k : int, optional
        Spatial harmonic order, defaults to 6.

    """

    def __init__(self, model: GradNet, k: int = 6) -> None:
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
