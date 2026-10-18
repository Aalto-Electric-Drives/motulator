"""
Training of GradNets.

The networks are defined and trained with PyTorch, which is installed as an optional
dependency with `pip install "motulator[gradnet]"`. The trained networks are saved
as numpy functions, see `motulator.drive.gradnet.load_gradnet`.

"""

from motulator.drive.gradnet._dataset import get_training_data
from motulator.drive.gradnet._gn import (
    AlgebraicSigmoid,
    GradNet,
    PNormGradient,
    Softmax,
    Squareplus,
)
from motulator.drive.gradnet._train import train_gradnet

__all__ = [
    "AlgebraicSigmoid",
    "GradNet",
    "PNormGradient",
    "Softmax",
    "Squareplus",
    "get_training_data",
    "train_gradnet",
]
