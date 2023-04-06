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
#
import os
import sys

sys.path.insert(0, os.path.abspath('../..'))
sys.path.insert(0, os.path.abspath('../../motulator'))

# -- Project information -----------------------------------------------------

project = 'motulator'
copyright = '2023, Aalto Electric Drives'
author = 'Aalto Electric Drives'

# The full version, including alpha/beta/rc tags
release = '0.1.2'

# -- General configuration ---------------------------------------------------

# This value contains a list of modules to be mocked up.
# This is useful when some external dependencies are not met at build time and
# break the building process. You may only specify the root package of the
# dependencies themselves and omit the sub-modules:
autodoc_mock_imports = ['numpy', 'matplotlib', 'scipy', 'cycler']

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = [
    'sphinx.ext.napoleon', 'sphinx.ext.viewcode', 'numpydoc',
    'sphinx_copybutton', 'sphinx.ext.mathjax', 'sphinx_gallery.gen_gallery'
]

extensions.append('autoapi.extension')
autoapi_type = 'python'
autoapi_dirs = ['../../motulator']
autodoc_typehints = 'description'
autoapi_options = [
    'members',
    'show-inheritance',
    'show-module-summary',
    'imported-members',
    'undoc-members',
    'special-members',
]  # 'private-members',
autoapi_python_class_content = 'both'
autoapi_keep_files = True

from sphinx_gallery.sorting import ExplicitOrder

sphinx_gallery_conf = {
    'examples_dirs':
    '../../examples',  # path to your example scripts
    'gallery_dirs':
    'auto_examples',  # path to where to save gallery generated output
    'nested_sections':
    True,
    'subsection_order':
    ExplicitOrder([
        '../../examples/vector', '../../examples/vhz',
        '../../examples/flux_vector', '../../examples/signal_inj',
        '../../examples/flux_maps', '../../examples/two_mass_systems'
    ]),
}

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['../../.idea', '../../__pycache__', '../../venv']

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'
# html_theme = 'sphinx_rtd_theme'
html_theme = 'sphinx_book_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['_static']
