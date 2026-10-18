"""
Gradient networks (GradNets) for magnetics modeling.

This module contains GradNet architecture to model the current and flux linkage maps of
synchronous machines [#Li2026]_. The GradNets allow modeling conservative vector fields
by construction [#Cha2025]_. In our case, the scalar state function is either the
magnetic energy or co-energy, depending on whether the current map or flux map is
modeled. The monotonicity of the flux-linkage--current map is also ensured.

The networks are defined and trained with PyTorch. A trained network is evaluated in
double precision with numpy, see `GradNetFunction` and the maps (e.g., `CurrentMap`).

References
----------
.. [#Li2026] Li, Foissner, Martin, Piippo, Hinkkanen, "Gradient networks for universal
   magnetic modeling of synchronous machines," 2026, https://arxiv.org/abs/2602.14947

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient networks," IEEE Trans. Signal
   Process., 2025, https://doi.org/10.1109/TSP.2024.3496692

"""

from collections.abc import Callable

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

    Defined as the gradient of S(z) = (1 + sum((beta*z_n)**p))**(1/p)/beta, where p is a
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
    psi_base : float, optional
        Base value of the flux linkage (Vs), defaults to 1.0. The network operates in
        per-unit quantities. The base values are stored in the model and used by the
        map wrappers (e.g., `CurrentMap`) to convert between SI units and per-unit
        quantities.
    i_base : float, optional
        Base value of the current (A), defaults to 1.0.

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
