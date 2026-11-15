"""Tests of the public paths in the API reference, see docs/source/api_paths.py."""

import importlib.util
from pathlib import Path

import pytest

_path = Path(__file__).parents[1] / "docs" / "source" / "api_paths.py"
_spec = importlib.util.spec_from_file_location("api_paths", _path)
assert _spec is not None and _spec.loader is not None
api_paths = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(api_paths)


@pytest.mark.parametrize(
    ("path", "context", "expected"),
    [
        (  # Class
            "motulator.common.control._base.ConverterControlSystem",
            "motulator.drive.control.im",
            "motulator.common.control.ConverterControlSystem",
        ),
        (  # Method
            "motulator.common.control._base.ControlSystem.save",
            "motulator.grid.control",
            "motulator.common.control.ControlSystem.save",
        ),
        (  # Type alias
            "motulator.common.model._pwm.GateStates",
            "motulator.common.model",
            "motulator.common.model.GateStates",
        ),
        (  # Exported from several modules, the module of the page preferred
            "motulator.common.control._controllers.PIController",
            "motulator.drive.control.sm",
            "motulator.drive.control.sm.PIController",
        ),
        (  # Exported from several modules, the closest module preferred
            "motulator.common.control._controllers.PIController",
            "motulator.grid.control",
            "motulator.common.control.PIController",
        ),
        (  # Not public
            "motulator.common.model._pwm.Delay",
            "motulator.common.model",
            None,
        ),
    ],
)
def test_public_path(path: str, context: str, expected: str | None) -> None:
    """The private path of an object is mapped to its public path."""
    assert api_paths.public_path(path, context) == expected
