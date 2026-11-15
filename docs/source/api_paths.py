"""
Public paths of the objects in the API reference.

AutoAPI writes the bases and the type hints as the paths of the objects in the private
modules (e.g., ``motulator.common.control._base.TimeSeries``). This extension replaces
each of them by a public path of the same object, preferring the module of the page and
then the module closest to the private one. A path without a public counterpart gives a
warning.

"""

import importlib
import pkgutil
import re
from functools import cache
from types import ModuleType
from typing import TYPE_CHECKING, Any

import motulator

if TYPE_CHECKING:
    from sphinx.application import Sphinx

# Path through a private module, e.g., motulator.common.control._base.TimeSeries
PRIVATE_PATH = re.compile(r"\bmotulator(?:\.\w+)*\._\w+(?:\.\w+)+")


@cache
def _public_modules() -> tuple[ModuleType, ...]:
    """Public modules of the package."""
    return tuple(
        importlib.import_module(info.name)
        for info in pkgutil.walk_packages(motulator.__path__, "motulator.")
        if not any(part.startswith("_") for part in info.name.split("."))
    )


@cache
def _public_paths(path: str) -> tuple[str, ...]:
    """Public paths of an object, or of its member, given by a private path."""
    parts = path.split(".")
    # The module is the longest importable prefix, followed by the object and its
    # members, e.g., a method of a class
    for i in range(len(parts) - 1, 0, -1):
        try:
            module = importlib.import_module(".".join(parts[:i]))
        except ImportError:
            continue
        name, members = parts[i], parts[i + 1 :]
        if not hasattr(module, name):
            return ()
        obj = getattr(module, name)
        return tuple(
            ".".join([public.__name__, name, *members])
            for public in _public_modules()
            if name in getattr(public, "__all__", ()) and getattr(public, name) is obj
        )
    return ()


def _common_parts(a: str, b: str) -> int:
    """Number of the common leading parts of two dotted paths."""
    n = 0
    for x, y in zip(a.split("."), b.split("."), strict=False):
        if x != y:
            break
        n += 1
    return n


def public_path(path: str, context: str) -> str | None:
    """
    Public path of an object given by a private path.

    Parameters
    ----------
    path : str
        Path through a private module.
    context : str
        Path of the page or the object where the path appears. Among several public
        paths, the one closest to `context` is chosen, then the one closest to `path`.

    Returns
    -------
    str | None
        Public path, or None if the object is not public.

    """
    paths = _public_paths(path)
    if not paths:
        return None
    return max(paths, key=lambda p: (_common_parts(p, context), _common_parts(p, path)))


def _replace(text: str, context: str, location: str) -> str:
    """Replace the private paths in a text by public paths."""
    from sphinx.util import logging

    def public(match: re.Match[str]) -> str:
        path = match.group(0)
        new_path = public_path(path, context)
        if new_path is None:
            logging.getLogger(__name__).warning(
                f"No public path for {path}", location=location
            )
            return path
        return new_path

    return PRIVATE_PATH.sub(public, text)


def _replace_in_source(app: "Sphinx", docname: str, source: list[str]) -> None:
    """Replace the private paths in the pages of the API reference."""
    if docname.startswith("autoapi/"):
        page = docname.removeprefix("autoapi/").removesuffix("/index")
        source[0] = _replace(source[0], page.replace("/", "."), docname)


def _replace_in_type_hints(app: "Sphinx") -> None:
    """Replace the private paths in the type hints recorded by AutoAPI."""
    for obj_id, hints in getattr(app.env, "autoapi_annotations", {}).items():
        for name, hint in hints.items():
            hints[name] = _replace(hint, obj_id, obj_id)


def setup(app: "Sphinx") -> dict[str, Any]:
    """Connect the replacements, the type hints after AutoAPI has recorded them."""
    app.connect("builder-inited", _replace_in_type_hints, priority=600)
    app.connect("source-read", _replace_in_source)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
