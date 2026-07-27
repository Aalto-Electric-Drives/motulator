"""Dataset classes and data loaders for GradNet training."""

from pathlib import Path
from typing import Any, Literal

import numpy as np
import torch
from torch import Tensor
from torch.utils.data import DataLoader, Dataset

from motulator.common.utils._utils import BaseValues


# %%
class BaseDataset(Dataset):
    """
    Base dataset class handling common fields.

    Parameters
    ----------
    data_path : str
        Path to the npz file containing the dataset.
    base : BaseValues
        Base values of target machine
    subsample : int, optional
        Subsampling rate, defaults to 1 (no subsampling).

    """

    def __init__(self, data_path: str, base: BaseValues, subsample: int = 1) -> None:
        with np.load(data_path) as data:
            i_full = data["i_s_dq"].flatten()
            psi_full = data["psi_s_dq"].flatten()
            # Apply subsampling
            indices = np.arange(0, len(i_full), subsample)
            self.indices = indices
            # Load main fields
            i_s_dq = i_full[indices]
            psi_s_dq = psi_full[indices]
            # Tensor conversion
            self.psi_d = torch.tensor(psi_s_dq.real, dtype=torch.float32)
            self.psi_q = torch.tensor(psi_s_dq.imag, dtype=torch.float32)
            self.i_d = torch.tensor(i_s_dq.real, dtype=torch.float32)
            self.i_q = torch.tensor(i_s_dq.imag, dtype=torch.float32)
            # Get base values before calling _load_extra
            self.i_base = float(base.i)
            self.psi_base = float(base.psi)
            # Load additional fields if needed
            self._load_extra(data, indices)

        # Normalize data to per-unit values
        self.i_d /= self.i_base
        self.i_q /= self.i_base
        self.psi_d /= self.psi_base
        self.psi_q /= self.psi_base

    def _load_extra(self, data: Any, indices: np.ndarray) -> None:
        pass

    def __len__(self) -> int:
        return len(self.psi_d)

    def __getitem__(self, index: int) -> tuple[Tensor, ...]:
        return self.psi_d[index], self.psi_q[index], self.i_d[index], self.i_q[index]

    def prepare_batch(
        self,
        batch: tuple[Tensor, ...],
        mode: Literal["current_map", "flux_map"],
        k: int | None = None,
    ) -> tuple[Tensor, Tensor]:
        """Prepare a batch for training."""
        if k is None:
            psi_d, psi_q, i_d, i_q = batch
            if mode == "flux_map":
                inputs = torch.stack((i_d, i_q), dim=1)
                targets = torch.stack((psi_d, psi_q), dim=1)
            else:
                inputs = torch.stack((psi_d, psi_q), dim=1)
                targets = torch.stack((i_d, i_q), dim=1)
        else:
            psi_d, psi_q, i_d, i_q, theta_m, _ = batch
            cos_t = torch.cos(k * theta_m)
            sin_t = torch.sin(k * theta_m)
            if mode == "flux_map":
                inputs = torch.stack((i_d, i_q, cos_t, sin_t), dim=1)
                targets = torch.stack((psi_d, psi_q), dim=1)
            else:
                inputs = torch.stack((psi_d, psi_q, cos_t, sin_t), dim=1)
                targets = torch.stack((i_d, i_q), dim=1)

        return inputs, targets


# %%
class SpatialHarmonicsDataset(BaseDataset):
    """
    Spatial harmonics dataset.

    Parameters
    ----------
    data_path : str
        Path to the npz file containing the dataset.
    base : BaseValues
        Base values of target machine
    subsample : int, optional
        Subsampling rate, defaults to 1 (no subsampling).

    """

    def _load_extra(self, data: Any, indices: np.ndarray) -> None:
        # Load additional fields
        i_s_dq = data["i_s_dq"].flatten()[indices] / self.i_base
        psi_s_dq = data["psi_s_dq"].flatten()[indices] / self.psi_base
        theta_m = data["theta_m"].flatten()[indices]
        # Normalize torque by per-pole-pair base torque
        tau_base = 1.5 * self.i_base * self.psi_base
        tau_m = data["tau_m"].flatten()[indices] / tau_base
        tau_m_theta = tau_m - (i_s_dq * psi_s_dq.conj()).imag
        # Tensor conversion
        self.theta_m = torch.tensor(theta_m, dtype=torch.float32)
        self.tau_m_theta = torch.tensor(tau_m_theta, dtype=torch.float32)

    def __getitem__(self, index: int) -> tuple[Tensor, ...]:
        return (
            self.psi_d[index],
            self.psi_q[index],
            self.i_d[index],
            self.i_q[index],
            self.theta_m[index],  # Electrical angle (rad)
            self.tau_m_theta[index],  # -dW_dtheta per pole pair (Nm)
        )


# %%
def get_loader(
    dataset_path: str | Path,
    base: BaseValues,
    k: int | None,
    batch_size: int = 16,
    shuffle: bool = True,
    subsample: int = 1,
) -> DataLoader:
    """Create a DataLoader for GradNet training data."""
    dataset_path = Path(dataset_path)
    dataset_cls = SpatialHarmonicsDataset if k is not None else BaseDataset
    dataset = dataset_cls(data_path=str(dataset_path), base=base, subsample=subsample)
    return DataLoader(dataset=dataset, batch_size=batch_size, shuffle=shuffle)


# %%
def get_training_data(
    dataset_path: str,
    base: BaseValues,
    subsample: int = 1,
    other_keys: list[str] | None = None,
) -> tuple[tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    """
    Get the training and validation data used by the model.

    This function re-instantiates the dataset with the same parameters to reproduce
    the training set. Validation data is the set difference (complement).

    Returns
    -------
    tuple
         ((train_psi, train_i, ...), (val_psi, val_i, ...))

    """
    if other_keys is None:
        other_keys = []

    # Re-create the dataset to match training
    ds = BaseDataset(dataset_path, base=base, subsample=subsample)

    # Load full data
    keys = ["psi_s_dq", "i_s_dq"] + other_keys
    with np.load(dataset_path) as data:
        full_data = [data[k].flatten() for k in keys]

    train_indices = ds.indices

    # Validation data is the complement
    mask = np.ones(len(full_data[0]), dtype=bool)
    mask[train_indices] = False

    train_out = tuple(arr[train_indices] for arr in full_data)
    val_out = tuple(arr[mask] for arr in full_data)

    return train_out, val_out
