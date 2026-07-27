"""
Gradient networks (GradNets) for magnetics modeling.

This module contains GradNet architecture to model the current and flux linkage maps of
synchronous machines [#Li2026]_. The GraNets allow modeling conservative vector fields
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

from pathlib import Path
from typing import Callable

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


# %%
class Squareplus(nn.Module):
    """Rectifier-type activation function with one learnable parameter."""

    def __init__(self, beta_log0: float = -2.0) -> None:
        super().__init__()
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return 0.5 * (x + torch.sqrt(x**2 + torch.exp(self.beta_log)))


# %%
class AlgebraicSigmoid(nn.Module):
    """Sigmoid-type activation function with one learnable parameter."""

    def __init__(self, beta_log0: float = -3.0) -> None:
        super().__init__()
        self.beta_log = nn.Parameter(torch.tensor(beta_log0, dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x / torch.sqrt(x**2 + torch.exp(self.beta_log))


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
        mu = torch.cat([torch.exp(self.mu_log), x.new_zeros(self.non_mu_dim)], dim=0)
        z = mu.view(1, -1) * x + self.bias
        # Add modules
        for i in range(self.num_modules):
            out = self.blocks[i](x)
            z += out
        return z


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
def _complex_to_torch_inputs(z: complex | np.ndarray, ndmin: int = 1) -> torch.Tensor:
    """Convert complex inputs to torch tensor with real and imaginary parts."""
    z = np.array(z, ndmin=ndmin, dtype=np.complex64)
    d = torch.from_numpy(np.real(z).astype(np.float32))
    q = torch.from_numpy(np.imag(z).astype(np.float32))
    return torch.stack((d, q), dim=-1)


# %%
class CurrentMap:
    """
    Callable wrapper for GradNet current map models.

    The map is symmetrized along to the d-axis to ensure physical consistency.

    Parameters
    ----------
    model : GradNet
        Trained GradNet model for the current map.

    """

    def __init__(self, model: GradNet) -> None:
        self.model = model
        self.in_base = model.psi_base.item()
        self.out_base = model.i_base.item()

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
        # Create a batch of inputs and their conjugates
        psi_s_dq = np.array(psi_s_dq, ndmin=1, dtype=np.complex64) / self.in_base
        inputs_combined = np.concatenate([psi_s_dq, np.conj(psi_s_dq)], axis=0)
        inputs = _complex_to_torch_inputs(inputs_combined)

        with torch.no_grad():
            outputs = self.model(inputs)

        # Unpack outputs
        i_d = outputs[..., 0].cpu().numpy()
        i_q = outputs[..., 1].cpu().numpy()
        i_s_dq = i_d + 1j * i_q

        # Symmetrize
        shape = np.shape(psi_s_dq)
        n = shape[0] if shape else 1
        i_s_dq = 0.5 * (i_s_dq[:n] + np.conj(i_s_dq[n:]))
        # i_s_dq = i_s_dq[: psi_s_dq.shape[0]]  # No symmetrization
        i_s_dq *= self.out_base

        return i_s_dq[0] if i_s_dq.size == 1 else i_s_dq


# %%
class FluxMap(CurrentMap):
    """
    Callable wrapper for GradNet current map models.

    The map is symmetrized along to the q-axis to ensure physical consistency.

    Parameters
    ----------
    model : GradNet
        Trained GradNet model for the current map.

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
        self.model = model
        # Harmonic order
        self.k = k

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
        k = self.k
        psi_s_dq = np.array(psi_s_dq, ndmin=1, dtype=np.complex64) / self.psi_base
        exp_j_k_theta = np.array(exp_j_theta_m, ndmin=1, dtype=np.complex64) ** k
        psi_inputs = _complex_to_torch_inputs(psi_s_dq)
        cos_k_theta = torch.from_numpy(np.real(exp_j_k_theta).astype(np.float32))
        sin_k_theta = torch.from_numpy(np.imag(exp_j_k_theta).astype(np.float32))
        inputs = torch.cat(
            (psi_inputs, cos_k_theta.unsqueeze(-1), sin_k_theta.unsqueeze(-1)), dim=-1
        )
        with torch.no_grad():
            outputs = self.model(inputs)
        i_d = outputs[..., 0].cpu().numpy()
        i_q = outputs[..., 1].cpu().numpy()
        i_s_dq = i_d + 1j * i_q
        dW_dcos = outputs[..., 2].cpu().numpy()
        dW_dsin = outputs[..., 3].cpu().numpy()
        dW_dtheta = k * (exp_j_k_theta.real * dW_dsin - exp_j_k_theta.imag * dW_dcos)
        # Torque in per-unit
        tau_m = np.imag(i_s_dq * np.conj(psi_s_dq)) - dW_dtheta
        # Scale back to physical units
        i_s_dq *= self.i_base
        tau_m *= self.tau_base
        return (
            i_s_dq[0] if i_s_dq.size == 1 else i_s_dq,
            tau_m[0] if tau_m.size == 1 else tau_m,
        )


# %%
class FluxMapWithHarmonics:
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
        self.i_base = model.i_base.item()
        self.psi_base = model.psi_base.item()
        self.tau_base = 1.5 * self.psi_base * self.i_base
        self.model = model
        # Harmonic order
        self.k = k

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
        k = self.k
        i_s_dq = np.array(i_s_dq, ndmin=1, dtype=np.complex64) / self.i_base
        exp_j_k_theta = np.array(exp_j_theta_m, ndmin=1, dtype=np.complex64) ** k
        i_inputs = _complex_to_torch_inputs(i_s_dq)
        cos_k_theta = torch.from_numpy(np.real(exp_j_k_theta).astype(np.float32))
        sin_k_theta = torch.from_numpy(np.imag(exp_j_k_theta).astype(np.float32))
        inputs = torch.cat(
            (i_inputs, cos_k_theta.unsqueeze(-1), sin_k_theta.unsqueeze(-1)), dim=-1
        )
        with torch.no_grad():
            outputs = self.model(inputs)
        psi_d = outputs[..., 0].cpu().numpy()
        psi_q = outputs[..., 1].cpu().numpy()
        psi_s_dq = psi_d + 1j * psi_q
        dW_dcos = outputs[..., 2].cpu().numpy()
        dW_dsin = outputs[..., 3].cpu().numpy()
        dW_dtheta = k * (exp_j_k_theta.real * dW_dsin - exp_j_k_theta.imag * dW_dcos)
        # Torque in per-unit
        tau_m = np.imag(i_s_dq * np.conj(psi_s_dq)) + dW_dtheta
        # Scale back to physical units
        psi_s_dq *= self.psi_base
        tau_m *= self.tau_base
        return (
            psi_s_dq[0] if psi_s_dq.size == 1 else psi_s_dq,
            tau_m[0] if tau_m.size == 1 else tau_m,
        )
