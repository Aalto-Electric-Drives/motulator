# ruff: noqa

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full list see
# the documentation: https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup ------------------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory, add
# these directories to sys.path here. If the directory is relative to the documentation
# root, use os.path.abspath to make it absolute, like shown here.

import os
import sys

sys.path.insert(0, os.path.abspath("../.."))
sys.path.insert(0, os.path.abspath("../../motulator"))

os.environ["BUILDING_DOCS"] = "1"

# -- Project information ---------------------------------------------------------------

project = "motulator"
copyright = "2025, Aalto Electric Drives"
author = "Aalto Electric Drives"

# The full version, including alpha/beta/rc tags
release = "0.7.1"

# -- General configuration -------------------------------------------------------------

# This value contains a list of modules to be mocked up. This is useful when some
# external dependencies are not met at build time and break the building process. You
# may only specify the root package of the dependencies themselves and omit the
# submodules:
autodoc_mock_imports = ["numpy", "matplotlib", "scipy", "cycler"]

# Add any Sphinx extension module names here, as strings. They can be extensions coming
# with Sphinx (named "sphinx.ext.*") or your custom ones.
extensions = [
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "numpydoc",
    "sphinx_copybutton",
    "sphinx.ext.mathjax",  # MathJax for reliable math rendering
    "sphinx_gallery.gen_gallery",
    "sphinxcontrib.bibtex",
    "myst_parser",
]

# Configure global bibliography
bibtex_bibfiles = ["refs.bib"]  # Path to your .bib file
bibtex_default_style = "plain"
bibtex_reference_style = "author_year"  # or 'label' depending on preference

# Enable MyST extensions for advanced features
myst_enable_extensions = [
    "dollarmath",  # For $...$ and $$...$$ math
    "amsmath",  # For numbered equations
    "deflist",  # For definition lists
    "fieldlist",  # For field lists
    "html_admonition",  # For note/warning boxes
    "colon_fence",  # For ::: fenced blocks
    "attrs_inline",  # For inline attributes
    "substitution",  # For substitutions
]

# MathJax configuration - reliable math rendering with custom macros
mathjax3_config = {
    "loader": {
        "load": ["[tex]/upgreek"]  # Explicitly load upgreek extension
    },
    "tex": {
        "packages": {"[+]": ["upgreek"]},  # Enable upright Greek fonts
        "macros": {
            # Operators and functions
            "jj": "\\mathrm{j}",
            "e": "\\mathrm{e}",
            "D": "\\mathrm{d}",
            "IM": "\\mathrm{Im}",
            "RE": "\\mathrm{Re}",
            "T": "\\intercal",
            # Space vectors
            "uc": "\\mathbf{u}_\\mathrm{c}",
            "ucref": "\\mathbf{u}_\\mathrm{c}^\\mathrm{ref}",
            "ucreflim": "\\bar{\\mathbf{u}}_\\mathrm{c}^\\mathrm{ref}",
            "ucs": "\\mathbf{u}_\\mathrm{c}^\\mathrm{s}",
            "ic": "\\mathbf{i}_\\mathrm{c}",
            "icref": "\\mathbf{i}_\\mathrm{c}^\\mathrm{ref}",
            "ics": "\\mathbf{i}_\\mathrm{c}^\\mathrm{s}",
            "qc": "\\mathbf{q}_\\mathrm{c}",
            "qcs": "\\mathbf{q}_\\mathrm{c}^\\mathrm{s}",
            "dc": "\\mathbf{d}_\\mathrm{c}",
            "dcs": "\\mathbf{d}_\\mathrm{c}^\\mathrm{s}",
            "us": "\\mathbf{u}_\\mathrm{s}",
            "uss": "\\mathbf{u}_\\mathrm{s}^\\mathrm{s}",
            "usref": "\\mathbf{u}_\\mathrm{s}^\\mathrm{ref}",
            "usreflim": "\\bar{\\mathbf{u}}_\\mathrm{s}^\\mathrm{ref}",
            "is": "\\mathbf{i}_\\mathrm{s}",
            "isref": "\\mathbf{i}_\\mathrm{s}^\\mathrm{ref}",
            "iss": "\\mathbf{i}_\\mathrm{s}^\\mathrm{s}",
            "ir": "\\mathbf{i}_\\mathrm{r}",
            "irs": "\\mathbf{i}_\\mathrm{r}^\\mathrm{s}",
            "iR": "\\mathbf{i}_\\mathrm{R}",
            "psis": "\\boldsymbol{\\uppsi}_\\mathrm{s}",
            "psisref": "\\boldsymbol{\\uppsi}_\\mathrm{s}^\\mathrm{ref}",
            "hatpsis": "\\hat{\\boldsymbol{\\uppsi}}_\\mathrm{s}",
            "psiss": "\\boldsymbol{\\uppsi}_\\mathrm{s}^\\mathrm{s}",
            "tildepsis": "\\tilde{\\boldsymbol{\\uppsi}}_\\mathrm{s}",
            "tildepsiR": "\\tilde{\\boldsymbol{\\uppsi}}_\\mathrm{R}",
            "psir": "\\boldsymbol{\\uppsi}_\\mathrm{r}",
            "psirs": "\\boldsymbol{\\uppsi}_\\mathrm{r}^\\mathrm{s}",
            "psiR": "\\boldsymbol{\\uppsi}_\\mathrm{R}",
            "psiRo": "\\boldsymbol{\\uppsi}_\\mathrm{R0}",
            "hatpsiR": "\\hat{\\boldsymbol{\\uppsi}}_\\mathrm{R}",
            "psiaux": "\\boldsymbol{\\uppsi}_\\mathrm{a}",
            "hatpsiaux": "\\hat{\\boldsymbol{\\uppsi}}_\\mathrm{a}",
            "iaux": "\\mathbf{i}_\\mathrm{a}",
            "hatiaux": "\\hat{\\mathbf{i}}_\\mathrm{a}",
            "es": "\\mathbf{e}_\\mathrm{s}",
            "ec": "\\mathbf{e}_\\mathrm{c}",
            "eo": "\\mathbf{e}_\\mathrm{o}",
            # Saturation functions
            "isfcn": "\\mathbf{g}",
            "idfcn": "g_\\mathrm{d}",
            "iqfcn": "g_\\mathrm{q}",
            "psisfcn": "\\mathbf{f}",
            "hatpsisfcn": "\\hat{\\mathbf{f}}",
            "psidfcn": "f_\\mathrm{d}",
            "psiqfcn": "f_\\mathrm{q}",
            # Space-vector magnitudes
            "abspsis": "\\psi_\\mathrm{s}",
            "abspsisref": "\\psi_\\mathrm{s}^\\mathrm{ref}",
            "hatabspsis": "\\hat{\\psi}_\\mathrm{s}",
            "hatabspsiR": "\\hat{\\psi}_\\mathrm{R}",
            "psif": "\\psi_\\mathrm{f}",
            "absis": "i_\\mathrm{s}",
            # Components
            "psid": "\\psi_\\mathrm{d}",
            "psiq": "\\psi_\\mathrm{q}",
            "id": "i_\\mathrm{d}",
            "iq": "i_\\mathrm{q}",
            "tildepsisd": "\\tilde{\\psi}_\\mathrm{sd}",
            "tildepsisq": "\\tilde{\\psi}_\\mathrm{sq}",
            # Resistances and inductances
            "Rs": "R_\\mathrm{s}",
            "Rr": "R_\\mathrm{r}",
            "Ls": "L_\\mathrm{s}",
            "Lsgm": "L_\\upsigma",
            "Rsgm": "R_\\upsigma",
            "LM": "L_\\mathrm{M}",
            "RR": "R_\\mathrm{R}",
            "Ld": "L_\\mathrm{d}",
            "Lq": "L_\\mathrm{q}",
            "Ldd": "L_\\mathrm{dd}",
            "Lqq": "L_\\mathrm{qq}",
            "Ldq": "L_\\mathrm{dq}",
            "Gdd": "\\varGamma_\\mathrm{dd}",
            "Gqq": "\\varGamma_\\mathrm{qq}",
            "Gdq": "\\varGamma_\\mathrm{dq}",
            # Angular frequencies and mechanical quantities
            "omegam": "\\omega_\\mathrm{m}",
            "omegamo": "\\omega_\\mathrm{m0}",
            "tildeomegam": "\\tilde{\\omega}_\\mathrm{m}",
            "omegaM": "\\omega_\\mathrm{M}",
            "omegaMref": "\\omega_\\mathrm{M}^\\mathrm{ref}",
            "omegamref": "\\omega_\\mathrm{m}^\\mathrm{ref}",
            "omegaL": "\\omega_\\mathrm{L}",
            "omegas": "\\omega_\\mathrm{s}",
            "hatomegas": "\\hat{\\omega}_\\mathrm{s}",
            "hatomegar": "\\hat{\\omega}_\\mathrm{r}",
            "omegaso": "\\omega_\\mathrm{s0}",
            "omegar": "\\omega_\\mathrm{r}",
            "omegarb": "\\omega_\\mathrm{rb}",
            "omegaro": "\\omega_\\mathrm{r0}",
            "thetam": "\\vartheta_\\mathrm{m}",
            "thetaM": "\\vartheta_\\mathrm{M}",
            "thetaL": "\\vartheta_\\mathrm{L}",
            "thetaML": "\\vartheta_\\mathrm{ML}",
            "tauM": "\\tau_\\mathrm{M}",
            "hattauM": "\\hat{\\tau}_\\mathrm{M}",
            "hattauL": "\\hat{\\tau}_\\mathrm{L}",
            "tauMref": "\\tau_\\mathrm{M}^\\mathrm{ref}",
            "tauL": "\\tau_\\mathrm{L}",
            "tauLtot": "\\tau_\\mathrm{L,tot}",
            "tauS": "\\tau_\\mathrm{S}",
            "np": "n_\\mathrm{p}",
            "JM": "J_\\mathrm{M}",
            "JL": "J_\\mathrm{L}",
            "BL": "B_\\mathrm{L}",
            "CS": "C_\\mathrm{S}",
            "KS": "K_\\mathrm{S}",
            # Other common symbols
            "Ts": "T_\\mathrm{s}",
            "Tsw": "T_\\mathrm{sw}",
            "ismax": "i_\\mathrm{s}^\\mathrm{max}",
            # Converter quantities
            "dA": "d_\\mathrm{a}",
            "dB": "d_\\mathrm{b}",
            "dC": "d_\\mathrm{c}",
            "qA": "q_\\mathrm{a}",
            "qB": "q_\\mathrm{b}",
            "qC": "q_\\mathrm{c}",
            "udc": "u_\\mathrm{dc}",
            "udcref": "u_\\mathrm{dc}^\\mathrm{ref}",
            "idc": "i_\\mathrm{dc}",
            "udi": "u_\\mathrm{di}",
            "iL": "i_\\mathrm{L}",
            "Cdc": "C_\\mathrm{dc}",
            "hatCdc": "\\hat{C}_\\mathrm{dc}",
            "Ldc": "L_\\mathrm{dc}",
            # Phase currents and voltages
            "iA": "i_\\mathrm{a}",
            "iB": "i_\\mathrm{b}",
            "iC": "i_\\mathrm{c}",
            "uAn": "u_\\mathrm{an}",
            "uBn": "u_\\mathrm{bn}",
            "uCn": "u_\\mathrm{cn}",
            "uAN": "u_\\mathrm{aN}",
            "uBN": "u_\\mathrm{bN}",
            "uCN": "u_\\mathrm{cN}",
            # LC filter
            "Cf": "C_\\mathrm{f}",
            "Lf": "L_\\mathrm{f}",
            "Rf": "R_\\mathrm{f}",
            # Grid converter quantities
            "Lg": "L_\\mathrm{g}",
            "Rg": "R_\\mathrm{g}",
            "Lt": "L_\\mathrm{t}",
            "Rt": "R_\\mathrm{t}",
            "Lfc": "L_\\mathrm{fc}",
            "Rfc": "R_\\mathrm{fc}",
            "Lfg": "L_\\mathrm{fg}",
            "Rfg": "R_\\mathrm{fg}",
            "ug": "\\mathbf{u}_\\mathrm{g}",
            "ugo": "\\mathbf{u}_\\mathrm{g0}",
            "tildeug": "\\tilde{\\mathbf{u}}_\\mathrm{g}",
            "absug": "u_\\mathrm{g}",
            "hatabsug": "\\hat{u}_\\mathrm{g}",
            "hatug": "\\hat{\\mathbf{u}}_\\mathrm{g}",
            "ugs": "\\mathbf{u}_\\mathrm{g}^\\mathrm{s}",
            "ig": "\\mathbf{i}_\\mathrm{g}",
            "igs": "\\mathbf{i}_\\mathrm{g}^\\mathrm{s}",
            "eg": "\\mathbf{e}_\\mathrm{g}",
            "egs": "\\mathbf{e}_\\mathrm{g}^\\mathrm{s}",
            "uf": "\\mathbf{u}_\\mathrm{f}",
            "ufs": "\\mathbf{u}_\\mathrm{f}^\\mathrm{s}",
            "vc": "\\mathbf{v}_\\mathrm{c}",
            "hatvc": "\\hat{\\mathbf{v}}_\\mathrm{c}",
            "omegag": "\\omega_\\mathrm{g}",
            "hatomegag": "\\hat{\\omega}_\\mathrm{g}",
            "thetac": "\\vartheta_\\mathrm{c}",
            "thetag": "\\vartheta_\\mathrm{g}",
            "hatthetag": "\\hat{\\vartheta}_\\mathrm{g}",
            "omegac": "\\omega_\\mathrm{c}",
            "Wdc": "W_\\mathrm{dc}",
            "hatWdc": "\\hat{W}_\\mathrm{dc}",
            "Wdcref": "W_\\mathrm{dc}^\\mathrm{ref}",
            "pg": "p_\\mathrm{g}",
            "hatpg": "\\hat{p}_\\mathrm{g}",
            "pgref": "p_\\mathrm{g}^\\mathrm{ref}",
            "vcref": "v_\\mathrm{c}^\\mathrm{ref}",
            "hatabsvc": "\\hat{v}_\\mathrm{c}",
            # PI controller and other gains
            "kt": "k_\\mathrm{t}",
            "kp": "k_\\mathrm{p}",
            "ki": "k_\\mathrm{i}",
            "alphai": "\\alpha_\\mathrm{i}",
            "ui": "u_\\mathrm{i}",
            "uff": "u_\\mathrm{ff}",
            "kd": "k_\\mathrm{d}",
            "kq": "k_\\mathrm{q}",
            "kotheta": "k_{\\mathrm{o}\\upvartheta}",
            "koomega": "k_{\\mathrm{o}\\upomega}",
            "kotau": "k_{\\mathrm{o}\\uptau}",
            # Complex controller gains
            "kT": "\\mathbf{k}_\\mathrm{t}",
            "kP": "\\mathbf{k}_\\mathrm{p}",
            "kI": "\\mathbf{k}_\\mathrm{i}",
            "uI": "\\mathbf{u}_\\mathrm{i}",
            "uFF": "\\mathbf{u}_\\mathrm{ff}",
            "kV": "\\mathbf{k}_\\mathrm{v}",
            "koa": "\\mathbf{k}_\\mathrm{o1}",
            "kob": "\\mathbf{k}_\\mathrm{o2}",
            # Bandwidths
            "alphac": "\\alpha_\\mathrm{c}",
            "alphas": "\\alpha_\\mathrm{s}",
            "alphai": "\\alpha_\\mathrm{i}",
            "alphao": "\\alpha_\\mathrm{o}",
            "alphadc": "\\alpha_\\mathrm{dc}",
            "alphapll": "\\alpha_\\mathrm{pll}",
            # Estimates
            "hatomegam": "\\hat{\\omega}_\\mathrm{m}",
            "hatthetam": "\\hat{\\vartheta}_\\mathrm{m}",
            "tildethetam": "\\tilde{\\vartheta}_\\mathrm{m}",
            "hatRs": "\\hat{R}_\\mathrm{s}",
            "hatRR": "\\hat{R}_\\mathrm{R}",
            "hatLs": "\\hat{L}_\\mathrm{s}",
            "hatLM": "\\hat{L}_\\mathrm{M}",
            "hatalpha": "\\hat{\\alpha}",
            "hatLsgm": "\\hat{L}_\\sigma",
            "hatRsgm": "\\hat{R}_\\sigma",
            "hatLd": "\\hat{L}_\\mathrm{d}",
            "hatLq": "\\hat{L}_\\mathrm{q}",
        },
    },
}  # Configure file extensions
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}

intersphinx_mapping = {"python": ("https://docs.python.org/3", None)}

extensions.append("autoapi.extension")
autoapi_type = "python"
autoapi_dirs = ["../../motulator"]
autodoc_typehints = "description"
autodoc_typehints_format = "short"
autoapi_options = [
    "members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
]  # "private-members", "imported-members", "undoc-members", "special-members",

# Point AutoAPI to a custom template directory
autoapi_template_dir = "../_templates/autoapi"

autoapi_python_class_content = "class"  # "both"
autoapi_keep_files = True
autoapi_add_toctree_entry = False
autoapi_member_order = "alphabetical"

from sphinx_gallery.sorting import ExplicitOrder

sphinx_gallery_conf = {
    "examples_dirs": ["../../examples/drive", "../../examples/grid"],
    "gallery_dirs": ["drive_examples", "grid_examples"],
    "nested_sections": True,
    "subsection_order": ExplicitOrder(
        [
            "../../examples/drive/flux_vector",
            "../../examples/drive/current_vector",
            "../../examples/drive/vhz",
            "../../examples/drive/signal_inj",
            "../../examples/grid/grid_following",
            "../../examples/grid/grid_forming",
        ]
    ),
}

# List of patterns, relative to source directory, that match files and directories to
# ignore when looking for source files. This pattern also affects html_static_path and
# html_extra_path.
exclude_patterns = [
    "../../.idea",
    "../../__pycache__",
    "../../venv",
    "_build",
    "build",
    "Thumbs.db",
    ".DS_Store",
]

# -- Options for HTML output -----------------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for a list of
# builtin themes.
html_theme = "sphinx_book_theme"
html_theme_options = {
    "repository_url": "https://github.com/Aalto-Electric-Drives/motulator",
    "use_repository_button": True,
    "navigation_with_keys": False,
}

# Add any paths that contain custom static files (such as style sheets) here, relative
# to this directory. They are copied after the builtin static files, so a file named
# "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_css_files = ["css/custom.css"]
