"""Classes and functions for training GradNet magnetic models."""

import random
from pathlib import Path
from typing import Callable, Literal, cast

import numpy as np
import torch
import torch.nn.functional as F
from torch import Tensor
from torch.utils.data import DataLoader
from tqdm import trange

import motulator.drive.gradnet._gn as gn
from motulator.common.utils._utils import BaseValues
from motulator.drive.gradnet._dataset import (
    BaseDataset,
    SpatialHarmonicsDataset,
    get_loader,
)


# %%
class Trainer:
    """
    Trainer class for GradNet models.

    Parameters
    ----------
    model : GradNet
        The GradNet model to be trained.
    data_loader : DataLoader
        DataLoader providing the training data.
    dataset : BaseDataset | SpatialHarmonicsDataset
        Dataset used for training.
    mode : {"current_map", "flux_map"}
        Which map to learn.
    k : int | None
        Spatial harmonics order. If None, no harmonics are used.
    lr : float
        Learning rate for the optimizer.

    """

    def __init__(
        self,
        model: gn.GradNet,
        data_loader: DataLoader,
        dataset: BaseDataset | SpatialHarmonicsDataset,
        mode: Literal["current_map", "flux_map"],
        k: int | None,
        lr: float,
    ) -> None:
        self.model = model
        self.data_loader = data_loader
        self.dataset = dataset
        self.mode = mode
        self.k = k
        self.optimizer = torch.optim.AdamW(model.parameters(), lr=lr)

    def train_epoch(self) -> float:
        """Train the model for one epoch and return total loss."""
        self.model.train()
        total_loss = 0.0
        for batch in self.data_loader:
            self.optimizer.zero_grad()
            loss = self._compute_batch_loss(batch)
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()
        return total_loss

    def _compute_batch_loss(self, batch):
        # Move batch to the same device as the model
        batch = tuple(x.to(next(self.model.parameters()).device) for x in batch)

        if self.k is None:
            return self._compute_loss_no_harmonics(batch)
        return self._compute_loss_with_harmonics(batch)

    def _compute_loss_no_harmonics(self, batch: tuple[Tensor, ...]) -> Tensor:
        """Compute loss for a batch without spatial harmonics."""
        mode = cast(Literal["current_map", "flux_map"], self.mode)
        inputs, targets = self.dataset.prepare_batch(batch, mode)
        output = self.model(inputs)
        return F.mse_loss(output, targets)

    def _compute_loss_with_harmonics(self, batch: tuple[Tensor, ...]) -> Tensor:
        """Compute loss for a batch with spatial harmonics."""
        mode = cast(Literal["current_map", "flux_map"], self.mode)
        k = cast(int, self.k)
        inputs, targets = self.dataset.prepare_batch(batch, mode, k)
        _, _, _, _, theta_m, tau_m_theta = batch
        cos_t = torch.cos(k * theta_m)
        sin_t = torch.sin(k * theta_m)
        output = self.model(inputs)
        dW_dcos = output[:, 2]
        dW_dsin = output[:, 3]
        dW_dtheta = k * (cos_t * dW_dsin - sin_t * dW_dcos)

        # Dataset is normalized to per-unit values
        if mode == "current_map":
            tau_m_theta_pred = -dW_dtheta
        else:
            tau_m_theta_pred = dW_dtheta

        loss_main = F.mse_loss(output[:, :2], targets)
        loss_tau = F.mse_loss(tau_m_theta_pred, tau_m_theta)
        return loss_main + loss_tau


# %%
def train_gradnet(
    dataset_path: str | Path,
    base: BaseValues,
    is_flux_map=False,
    k: int | None = None,
    num_modules: int = 1,
    embed_dim: int = 12,
    batch_size: int = 128,
    epochs: int = 2000,
    lr: float = 1e-3,
    save_model_path: str | Path | None = None,
    subsample: int = 1,
    activation: Callable[[], torch.nn.Module] | None = None,
    device: torch.device | None = None,
) -> None:
    """
    Train and save the GradNet model.

    Parameters
    ----------
    dataset_path : str | Path
        Path to the training data file (npz format).
    is_flux_map : bool, optional
        Whether the model is a flux map or current map, defaults to False.
    k : int | None, optional
        Spatial harmonics order. If None, no harmonics are used, defaults to None.
    num_modules : int, optional
        Number of GradNet modules, defaults to 1.
    embed_dim : int, optional
        Embedding dimension for the GradNet modules, defaults to 12.
    batch_size : int, optional
        Batch size for training, defaults to 128.
    epochs : int, optional
        Number of training epochs, defaults to 2000.
    lr : float, optional
        Learning rate for the optimizer, defaults to 1e-3.
    save_model_path : str | Path | None, optional
        Path to save the trained model. If None, saves to `model.pth` in current
        directory.
    subsample : int, optional
        Subsampling factor for the training data, defaults to 1 (uses all data).
    activation : Callable[[], torch.nn.Module] | None, optional
        Activation function factory, defaults to PNormGradient.
    device : torch.device | None, optional
        Device to use for training. If None, automatically selects CUDA if available,
        otherwise CPU.

    """
    # Set random seed for reproducibility
    seed = 42
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Initialize model and data
    mode = "flux_map" if is_flux_map else "current_map"
    if activation is None:
        activation = gn.PNormGradient
    in_dim = 4 if k is not None else 2
    mu_dim = 2

    # Base values are inferred from the dataset
    data_loader = get_loader(
        dataset_path=dataset_path,
        base=base,
        k=k,
        batch_size=batch_size,
        subsample=subsample,
    )
    dataset = cast(BaseDataset | SpatialHarmonicsDataset, data_loader.dataset)

    model = gn.GradNet(
        in_dim,
        mu_dim,
        num_modules,
        embed_dim,
        activation,
        mu_log0=-1.0,
        psi_base=dataset.psi_base,
        i_base=dataset.i_base,
    ).to(device)
    _run_training_loop(
        model, data_loader, mode, k, lr, epochs, activation, description="Training"
    )

    # Print model parameters and weights
    print("\nModel parameters and weights:\n" + "-" * 40)
    for name, param in model.named_parameters():
        print(f"{name}: {param.shape}\n{param.data}\n{'-' * 40}")

    # Save model
    if save_model_path is None:
        save_model_path = Path.cwd() / "model.pth"
    save_model_path = Path(save_model_path)
    save_model_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), save_model_path)


def _run_training_loop(
    model, data_loader, mode, k, lr, epochs, activation, description="Epoch"
) -> None:
    """Helper to run training loop for a given data loader."""
    dataset = cast(BaseDataset | SpatialHarmonicsDataset, data_loader.dataset)
    _print_training_info(model, dataset, activation)
    trainer = Trainer(model, data_loader, dataset, mode, k, lr)
    for epoch in trange(epochs, desc=description):
        total_loss = trainer.train_epoch()
        if epoch % (epochs // 10 if epochs > 10 else 1) == 0 or epoch == epochs - 1:
            avg_loss = total_loss / len(data_loader)
            print(f"[{description} {epoch}] Loss: {avg_loss:.6f}")


# %%
def _print_training_info(
    model: gn.GradNet,
    dataset: BaseDataset | SpatialHarmonicsDataset,
    activation: Callable[[], torch.nn.Module],
) -> None:
    """Print information about the training configuration."""
    print(f"Activation: {activation.__name__}")
    n_trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Trainable parameters: {n_trainable}")

    print(
        f"Normalization factors: "
        f"i_base={dataset.i_base:.2f}, "
        f"psi_base={dataset.psi_base:.2f}, "
        f"tau_base={1.5 * dataset.psi_base * dataset.i_base:.2f}"
    )
