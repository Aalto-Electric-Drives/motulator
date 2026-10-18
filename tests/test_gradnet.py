"""Tests for GradNet models."""

import numpy as np
import pytest
import torch

import motulator.drive.gradnet as gn
from motulator.drive.gradnet import training
from motulator.drive.gradnet.training import GradNet

ACTIVATIONS = [
    training.Softmax,
    training.PNormGradient,
    training.Squareplus,
    training.AlgebraicSigmoid,
]


@pytest.mark.parametrize("activation", ACTIVATIONS)
def test_function(activation: type[torch.nn.Module], tmp_path) -> None:
    """
    The numpy function equals the model, and its analytic Jacobian equals the one
    computed with automatic differentiation, also after saving and loading.
    """
    torch.manual_seed(0)
    model = GradNet(in_dim=4, mu_dim=2, num_modules=2, activation=activation)
    fcn = gn.GradNetFunction.from_model(model)
    fcn.save(tmp_path / "model.npz")
    loaded = gn.load_gradnet(tmp_path / "model.npz")
    # The model in double precision has the parameters of the function
    model = model.double()
    x = torch.randn(5, 4, dtype=torch.float64)
    y = model(x).detach().numpy()
    jac = torch.autograd.functional.jacobian(lambda z: model(z).sum(dim=0), x)
    jac = jac.permute(1, 0, 2).numpy()
    for f in (fcn, loaded):
        y_np, jac_np = f(x.numpy(), jacobian=True)
        assert np.allclose(y_np, y)
        assert jac_np is not None and np.allclose(jac_np, jac)


@pytest.mark.parametrize("activation", ACTIVATIONS)
def test_map_jacobian(activation: type[torch.nn.Module]) -> None:
    """The Jacobian of the symmetrized map equals the finite differences."""
    torch.manual_seed(1)
    model = GradNet(in_dim=2, mu_dim=2, activation=activation, psi_base=0.5, i_base=5)
    current_map = gn.CurrentMap(model)
    psi = np.array([0.3 + 0.2j, -0.1 - 0.4j, 0.05j])
    h = 1e-6
    d_d = (current_map(psi + h) - current_map(psi - h)) / (2 * h)
    d_q = (current_map(psi + 1j * h) - current_map(psi - 1j * h)) / (2 * h)
    jac_fd = np.stack(
        [np.stack([d_d.real, d_q.real], -1), np.stack([d_d.imag, d_q.imag], -1)], -2
    )
    assert np.allclose(current_map.jacobian(psi), jac_fd, rtol=1e-6, atol=1e-6)
    # Symmetry about the d-axis
    assert np.allclose(current_map(psi.conj()), np.conj(current_map(psi)))


def test_unsupported_models() -> None:
    """Models that cannot be converted to the numpy function are rejected."""
    exponents = iter((2, 8))
    for activation in (
        torch.nn.Softplus,  # Not a GradNet activation
        lambda: training.Softmax(dim=0),  # Not along the last axis
        lambda: training.PNormGradient(p=next(exponents)),  # Different modules
    ):
        model = training.GradNet(num_modules=2, activation=activation)
        with pytest.raises(ValueError):
            gn.GradNetFunction.from_model(model)
