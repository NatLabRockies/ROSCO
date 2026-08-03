.. toctree::

.. _install:

Installing ROSCO toolset
========================
ROSCO toolsets can be utilized either to run an existing controller or to design and tune a controller from scratch.
We recommend using the instructions provided in the :ref:`full_rosco` to install the full ROSCO toolset.
This allows for full use of the provided functionalities including the controller and toolbox to facilitate controller tuning.
However, if only the ROSCO binary is needed (to run an existing controller, for example), then users should follow the instructions provided in :ref:`rosco_controller`

.. _full_rosco:

Complete ROSCO Installation
---------------------------
Steps for the installation of the complete rosco toolset are:

1. Create a conda environment for ROSCO

.. code-block:: bash

    conda config --add channels conda-forge # (Enable Conda-forge Channel For Conda Package Manager)
    conda create -y --name rosco-env python=3.11 # (Create a new environment named "rosco-env" that contains Python 3.11)
    conda activate rosco-env # (Activate your "rosco-env" environment)

    # Windows users sometimes get an error related to the SSL configuration; in this case, use
    # Be sure to execute commands in an anaconda terminal rather than the windows command prompt
    conda config --set ssl_verify no

    # Install necessary compilers
    conda install -y gfortran gcc libpython m2-pkg-config  # windows
    conda install compilers                     # unix


    # If you intend to use ZeroMQ
    brew install zeromq  # mac
    sudo apt install libzmq3-dev libzmq5 libczmq-dev libczmq4  # linux

2. Clone and Install the ROSCO toolbox with ROSCO controller
    
.. code-block:: bash

    git clone https://github.com/NREL/ROSCO.git
    cd ROSCO
    pip install -e . --no-deps

This step creates the rosco controller binary (:code:`libdiscon.so` (Linux), :code:`libdiscon.dylib` (Mac), or :code:`libdisscon.dll` (Windows)`) in the directory :code:`ROSCO/rosco/lib` and installs the python toolbox in the conda environment in the develop mode.

3.  If for some reason the pip-based installation does not work, the conda environment can be created using

.. code-block:: bash

    conda env update --file environment.yml
    pip install -e . --no-deps

.. _rosco_controller:

Installing only the ROSCO controller
------------------------------------
:numref:`rosco_table` provides an overview of the primary methods available for installing only the ROSCO controller binary.

.. _rosco_table:
.. list-table:: Methods for Installing the ROSCO Controller
   :widths: 30 70
   :header-rows: 1

   * - Method
     - Use Case
   * - :ref:`rosco_direct_download`
     - Best for users who simply want to use a released version of the controller binary without working through the compilation procedures. 
   * - :ref:`rosco_anaconda_download`
     - Best for users who just want to use the controller binary but prefer to download using the Anaconda package manager.
   * - :ref:`cmake_compile`
     - Best for users who need to re-compile the source code often, plan to use non-released versions of ROSCO (including modified source code), or who simply want to compile the controller themselves so they have the full code available locally.

Anaconda is a popular package manager used to distribute software packages of various types.
Anaconda is used to download requisite packages and distribute pre-compiled versions of the ROSCO tools.
CMake is a build configuration system that creates files as input to a build tool like GNU Make, Visual Studio, or Ninja.
CMake does not compile code or run compilers directly, but rather creates the environment needed for another tool to run compilers and create binaries.
CMake is used to ease the processes of compiling the ROSCO controller locally.
For more information on CMake, please see `understanding CMake <https://openfast.readthedocs.io/en/main/source/install/index.html#understanding-cmake>`_ in the OpenFAST documentation.

.. _rosco_direct_download:

Direct Download
................
The most recent tagged version releases of the controller are `available for download <https://github.com/NREL/ROSCO/tags>`_. One can simply download these compiled binary files for their system and point to them in their simulation tools (e.g. through :code:`DLL_FileName` in the ServoDyn input file of OpenFAST).

.. _rosco_anaconda_download:

Anaconda Download
.................
Using the popular package manager, Anaconda_, the tagged 64-bit versions of ROSCO are available through the conda-forge channel. 
In order to download the most recently compiled version release, from an anaconda powershell (Windows) or terminal (Mac/Linux) window, create a new anaconda virtual environment: 

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.11
    conda activate rosco-env

navigate to your desired folder to save the compiled binary using:

.. code-block:: bash

    cd <desired_folder>
    
and download the controller:

.. code-block:: bash

    conda install -y ROSCO

This will download a compiled ROSCO binary file into the default filepath for any dynamic libraries downloaded via anaconda while in the ROSCO-env. 
The ROSCO binary file can be copied to your desired folder using:

.. code-block:: bash

    cp $CONDA_PREFIX/lib/libdiscon.* <desired_folder>

on linux or:

.. code-block:: bash

    copy %CONDA_PREFIX%/lib/libdiscon.* <desired_folder>


on Windows.


.. _cmake_compile:

Compile using CMake
.....................
CMake_ eases the compiling process significantly. We recommend that users use CMake if at all possible, as we cannot guarantee support for the use of other tools to aid with compiling ROSCO.

On Mac/Linux, standard compilers are generally available without any additional downloads. On 32-bit windows, we recommend that you `install MinGW <http://capsis.cirad.fr/capsis/documentation/mingw-installation>`_ (Section 2). On 64-bit Windows, you can simply install the MSYS2 toolchain through Anaconda:

.. code-block:: bash

    conda uninstall gcc gfortran  # if you previously installed via pip above
    conda install m2w64-toolchain libpython
    conda install cmake make  # if Windows users would like to install these in anaconda environment

Once the CMake and the required compilers are downloaded, the following code can be used to compile ROSCO.

.. code-block:: bash

    # Clone ROSCO
    git clone https://github.com/NREL/ROSCO.git

    # Compile ROSCO
    cd ROSCO/rosco/controller
    mkdir build
    cd build
    cmake ..                        # Mac/linux only
    cmake .. -G "MinGW Makefiles"   # Windows only 
    make install

This will generate a file called :code:`libdiscon.so` (Linux), :code:`libdiscon.dylib` (Mac), or :code:`libdisscon.dll` (Windows).




.. _Anaconda: https://www.anaconda.com/
.. _CMake: https://cmake.org/
.. _MinGW: https://mingw-w64.org/
