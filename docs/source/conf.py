# pylint: disable=all

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

import os
import sys

sys.path.insert(0, os.path.abspath("../.."))
sys.path.insert(0, os.path.abspath("../../motulator"))

# -- Project information -----------------------------------------------------

project = "motulator"
copyright = "2024, Aalto Electric Drives"
author = "Aalto Electric Drives"

# The full version, including alpha/beta/rc tags
release = "0.4.0"

# -- General configuration ---------------------------------------------------

# This value contains a list of modules to be mocked up.
# This is useful when some external dependencies are not met at build time and
# break the building process. You may only specify the root package of the
# dependencies themselves and omit the sub-modules:
autodoc_mock_imports = ["numpy", "matplotlib", "scipy", "cycler"]

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named "sphinx.ext.*") or your custom ones.
extensions = [
    "sphinx.ext.napoleon", "sphinx.ext.viewcode", "numpydoc",
    "sphinx_copybutton", "sphinx.ext.mathjax", "sphinx_gallery.gen_gallery"
]

extensions.append("autoapi.extension")
autoapi_type = "python"
autoapi_dirs = ["../../motulator"]
autodoc_typehints = "description"
autoapi_options = [
    "members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
]  # "private-members", "imported-members", "undoc-members", "special-members",

autoapi_python_class_content = "class"  # "both"
autoapi_keep_files = True
autoapi_add_toctree_entry = False

from sphinx_gallery.sorting import ExplicitOrder

sphinx_gallery_conf = {
    "examples_dirs": ["../../examples/drive", "../../examples/grid"],
    "gallery_dirs": ["drive_examples", "grid_examples"],
    "nested_sections":
    True,
    "subsection_order":
    ExplicitOrder([
        "../../examples/drive/vector",
        "../../examples/drive/vhz",
        "../../examples/drive/obs_vhz",
        "../../examples/drive/flux_vector",
        "../../examples/drive/signal_inj",
        "../../examples/grid/grid_following",
        "../../examples/grid/grid_forming",
    ]),
}

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["../../.idea", "../../__pycache__", "../../venv"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_book_theme"
html_theme_options = {
    "repository_url": "https://github.com/Aalto-Electric-Drives/motulator",
    "use_repository_button": True,
    "navigation_with_keys": False,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ["_static"]
