ROSCO Documentation
===================
:Version: |release|
:Date: |today|

NREL's "Reference Open Source Controller" (ROSCO) is a reference controller framework that facilitates design and implementation of wind turbine and wind farm controllers for fixed and floating offshore wind turbines.

ROSCO frameworks includes a large set of available controllers and advanced functionalities that can be combined in a modular fashion based on the intended application and can be easily adapted to a wide variety wind turbines.
For example, ROSCO can be used to design turbine yaw controller along with an individual blade pitch controller with floating platform feedback for an offshore turbine while simulating a pitch actuator fault and running a user-defined torque controller.

ROSCO provides a single framework for designing controllers for onshore and offshore turbines of varying sizes.
It can be used to run representative dynamic simulations using OpenFAST.
This helps researchers perform 'apples-to-apples' comparison of controller capabilities across turbines.
Control engineers can also design their own controllers and compare them with reference controller design using ROSCO for existing and new turbines.
ROSCO has been used to provide reference controllers for many recent reference turbines including the `IEA 3.4-MW <https://github.com/IEAWindTask37/IEA-3.4-130-RWT>`_ , `IEA 10-MW <https://github.com/IEAWindTask37/IEA-10.0-198-RWT>`_ , `IEA 15-MW <https://github.com/IEAWindTask37/IEA-15-240-RWT>`_ and the upcoming `IEA 22-MW <https://github.com/IEAWindTask37/IEA-22-280-RWT>`_ turbines.

The ROSCO framework also includes a python based toolbox that primarily enables tuning the controllers.
The tuning process is extemely simple where only a tuning parameters need to be provided.
It is not necessary to run aeroelastic simulations or provide linearized state-space models to tune the controller to tune the controllers.
The toolbox has other capabilities like simple 1-DOF turbine simulations for quick controller capability verifications, linear model analysis, and parsing of input and output files.

Source code for ROSCO toolset can be found in this `github repository <https://github.com/NREL/ROSCO>`_ and it can be installed following the instructions provided in :ref:`install`.

Building the Documentation
--------------------------

To build these docs locally, navigate to the :code:`docs` directory and run:

.. code-block:: bash

    sphinx-build . ./_build/

This generates HTML documentation in the :code:`_build` folder.

**Documentation Directory**

.. toctree::
   :maxdepth: 3
   :numbered:
   
   source/standard_use.rst
   source/install.rst
   source/examples.rst
   source/rosco.rst
   source/rosco_toolbox.rst
   source/api_change.rst
   source/toolbox_input.rst
   source/marine_hydro.rst
   source/how_to_contribute_code.rst
   source/ROSCO_instructions_for_Bladed.rst

License
-------
Copyright 2021 NREL

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
