"""Tests for the GradNet functions and maps."""

import numpy as np
import pytest

import motulator.drive.gradnet as gn


def random_function(
    activation: str, in_dim: int, num_modules: int = 2, embed_dim: int = 6
) -> gn.GradNetFunction:
    """Function with random parameters."""
    rng = np.random.default_rng(0)
    return gn.GradNetFunction(
        W=rng.normal(size=(num_modules, embed_dim, in_dim)),
        b=rng.normal(size=(num_modules, embed_dim)),
        beta_log=rng.normal(size=num_modules),
        mu_log=rng.normal(size=2),
        bias=rng.normal(size=in_dim),
        activation=activation,
        p=8 if activation == "PNormGradient" else 0,
        psi_base=0.5,
        i_base=5.0,
    )


@pytest.mark.parametrize("activation", gn.ACTIVATIONS)
def test_function(activation: str, tmp_path) -> None:
    """
    The analytic Jacobian of the function equals the finite differences, and the
    function is unchanged by saving and loading.
    """
    fcn = random_function(activation, in_dim=4)
    x = np.random.default_rng(1).normal(size=(5, 4))
    y, jac = fcn(x, jacobian=True)
    h = 1e-6
    jac_fd = np.stack(
        [(fcn(x + h * e)[0] - fcn(x - h * e)[0]) / (2 * h) for e in np.eye(4)], -1
    )
    assert jac is not None and np.allclose(jac, jac_fd, rtol=1e-6, atol=1e-6)
    fcn.save(tmp_path / "model.npz")
    y_loaded, jac_loaded = gn.load_gradnet(tmp_path / "model.npz")(x, jacobian=True)
    assert jac_loaded is not None and np.array_equal(jac_loaded, jac)
    assert np.array_equal(y_loaded, y)


@pytest.mark.parametrize("activation", gn.ACTIVATIONS)
def test_map_jacobian(activation: str) -> None:
    """The Jacobian of the symmetrized map equals the finite differences."""
    current_map = gn.CurrentMap(random_function(activation, in_dim=2))
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


def test_map_with_harmonics() -> None:
    """The map with harmonics is symmetric about the d-axis, including the torque."""
    harm_map = gn.CurrentMapWithHarmonics(random_function("Softmax", in_dim=4), k=6)
    psi = np.array([0.3 + 0.2j, -0.1 - 0.4j, 0.05j])
    theta_m = np.array([0.1, 0.5, 1.0])
    i_s, tau_m = harm_map(psi, np.exp(1j * theta_m))
    i_s_conj, tau_m_conj = harm_map(psi.conj(), np.exp(-1j * theta_m))
    assert np.allclose(i_s_conj, np.conj(i_s))
    assert np.allclose(tau_m_conj, -tau_m)
