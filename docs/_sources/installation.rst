Installation
============

Option 1: Use pip
-----------------
The simplest way to install *motulator* is to use ``pip``. First, install Python and ``pip`` on your computer (https://www.python.org/).
Ensure that you have the latest ``pip``, which can be updated with the command::

   pip install --upgrade pip

Then, install *motulator*::

   pip install motulator

.. note::
   instead of global installation described above, using a virtual environment is generally recommended. For more information, see https://packaging.python.org/guides/installing-using-pip-and-virtual-environments. Virtual enviroments are easiest to create and manage using some integrated development enviroment (IDE), see Option 2 below.

Option 2: Clone the Repository
------------------------------
For developers and advanced users, we recommended cloning the repository from GitHub. This option allows you to get the most recent version of the repository. Furthermore, you can then modify the system models and example controllers. It is also advisable to use a virtual environment to avoid conflicts with other Python packages. 

Several powerful open-source IDEs are available for Python, one of the most popular being VS Code (https://code.visualstudio.com). The following instructions are for VS Code:

1)	Install VS Code, Python, and ``git`` on your computer. Install also the recommended Python extensions in VS Code.
2) Clone the project::
    
      git clone https://github.com/AaltoElectricDrives/motulator

   This will create a folder called *motulator* in your current directory. 

3) Launch VS Code from the cloned project's root directory on the command line (or choose the proper directory after launching VS Code).
4) Create a virtual environment in the workspace using the instructions provided here: https://code.visualstudio.com/docs/python/environments.
5) Enable installation of suggested requirements in the virtual environment (at least ``requirements.txt``). Alternatively, you may run the command ``pip install -r requirements.txt`` in the VS Code terminal after the virtual environment is created and activated. 

After completing the above steps, the virtual environment can be found in the ``.venv`` directory at the root of the repository. Now you should be able to run all the examples as well as to modify the existing code. If you installed the requirement in ``requirements-dev.txt``, you can also use the interactive IPython console (click on the *Play* button dropdown menu in VS Code). Furthermore, when you start VS Code next time, it will automatically detect the virtual environment and use it.

If you use Windows, you may need to change the default terminal from the PowerShell to the Command Prompt (press Ctrl+Shift+P for the command palette and search for *Terminal: Select Default Profile*). 

We hope that these instructions allow you to create a virtual environment and start working on the project. Similar steps can be followed for other IDEs.
