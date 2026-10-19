# Installation

*motulator* requires Python 3.12 or newer. On Windows and macOS, Python can be installed from the [Python official website](https://www.python.org/). On Linux, Python is typically preinstalled.

The plots need a GUI library for *matplotlib*. The *Tkinter* library comes with Python on Windows and macOS. On some Linux systems, it must be installed separately, e.g., on Ubuntu with the command:

```bash
sudo apt install python3-tk
```

Alternatively, another GUI library, such as [PyQt6](https://pypi.org/project/PyQt6/), can be used.

## Option 1: Use pip

Install the package in a [virtual environment](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/). On Linux and macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install motulator
```

On Windows:

```powershell
py -m venv .venv
.venv\Scripts\activate
pip install motulator
```

If PowerShell does not allow running the activation script, allow it with `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`.

## Option 2: Clone the Repository

Clone the repository to run the examples, some of which use the data files in `examples/`, or to modify the system models and controllers. On Linux and macOS:

```bash
git clone https://github.com/Aalto-Electric-Drives/motulator
cd motulator
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev,doc]"
```

On Windows, create and activate the virtual environment as in Option 1. The `dev` dependencies are needed for the tests and the code checks, and the `doc` dependencies for building this documentation. In [VS Code](https://code.visualstudio.com/docs/python/environments), the virtual environment can also be created with the Python extension.

Then run an example, e.g.:

```bash
python examples/drive/flux_vector/plot_2kw_ipmsm_fvc.py
```

The tests are run with `pytest`, and the documentation, which also runs all the examples, is built with the command:

```bash
sphinx-build -b html docs/source docs/build/html
```
