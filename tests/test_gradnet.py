"""Tests for GradNet models."""

import pytest
import torch

import motulator.drive.gradnet as gn
from motulator.drive.gradnet._gn import GradNet


@pytest.mark.parametrize(
    "activation", [gn.Softmax, gn.PNormGradient, gn.Squareplus, gn.AlgebraicSigmoid]
)
def test_jacobian(activation: type[torch.nn.Module]) -> None:
    """The analytic Jacobian equals the one computed with automatic differentiation."""
    torch.manual_seed(0)
    model = GradNet(in_dim=4, mu_dim=2, num_modules=2, activation=activation).double()
    x = torch.randn(5, 4, dtype=torch.float64)
    jac = torch.autograd.functional.jacobian(lambda z: model(z).sum(dim=0), x)
    assert torch.allclose(model.jacobian(x), jac.permute(1, 0, 2))
