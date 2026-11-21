"""
Public paths of the objects in the API reference.

AutoAPI writes the bases and the type hints as the paths of the objects in the private
modules (e.g., ``motulator.common.control._base.TimeSeries``). This extension replaces
each of them by a public path of the same object, preferring the module of the page and
then the module closest to the private one. A path without a public counterpart stops
the build, since the object should then be exported. A bare name that is not found on
its page (e.g., a class referred to in a docstring) is resolved to the object of that
name documented on another page, the one closest to the page.

"""

import importlib
import pkgutil
import re
from functools import cache
from types import ModuleType
from typing import TYPE_CHECKING, Any

from sphinx.domains.python import PythonDomain
from sphinx.util.nodes import make_refnode

import motulator

if TYPE_CHECKING:
    from docutils.nodes import Element, TextElement, reference
    from sphinx.application import Sphinx
    from sphinx.environment import BuildEnvironment

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

    def public(match: re.Match[str]) -> str:
        path = match.group(0)
        new_path = public_path(path, context)
        if new_path is None:
            raise ValueError(f"No public path for {path} in {location}")
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


def _resolve_bare_name(
    app: "Sphinx", env: "BuildEnvironment", node: "Element", contnode: "TextElement"
) -> "reference | None":
    """Resolve a bare name to the closest module-level object of that name."""
    name, domain = node["reftarget"], env.get_domain("py")
    if node.get("refdomain") != "py" or "." in name:
        return None
    if not isinstance(domain, PythonDomain):
        return None
    objects = domain.objects
    paths = [
        path
        for path, obj in objects.items()
        if path.rsplit(".", 1)[-1] == name
        and obj.objtype in ("class", "function", "data")
    ]
    if not paths:
        return None
    context = node.get("py:module") or ""
    path = max(paths, key=lambda p: _common_parts(p, context))
    obj = objects[path]
    return make_refnode(
        app.builder, node["refdoc"], obj.docname, obj.node_id, contnode, path
    )


def setup(app: "Sphinx") -> dict[str, Any]:
    """Connect the replacements, the type hints after AutoAPI has recorded them, and the
    resolution of the bare names."""
    app.connect("builder-inited", _replace_in_type_hints, priority=600)
    app.connect("source-read", _replace_in_source)
    app.connect("missing-reference", _resolve_bare_name)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
