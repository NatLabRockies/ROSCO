"""
17c_zeromq_fastfarm
-------------------
This example demonstrates the wind farm controls capability of ROSCO
using ZeroMQ based communication with a FAST.Farm simulation.

Note that in order to use the wind farm control capability of ROSCO,
the :code:`ZMQ_Mode` must be set to 1 and unique identifiers must be
set under :code:`ZMQ_ID` for each turbine in the :code.
Wind farm level controller should be defined under a class called
:code:`wfc_controller` which should implement a method called :code:`update_setpoints`.
This method should take as arguments the unique turbine identifier, the current
time and measurements, and return the setpoints for the particular turbine.
Please see, the file :code:`rosco/controller/rosco_registry/wfc_interface.yaml`
for the list of available measurements and setpoints.
A ZeroMQ server and FAST.Farm simulation are run in parallel as separate processes.
The :code:`wfc_controller` property of the server must be set to the 
:code:`wfc_controller` class containing the wind farm controller logic.
Note that FAST.Farm requires separate ROSCO libraries for each set of OpenFAST files.
The nacelle yaw positions outputs of the two turbines in the wind farm are shown below.
    
.. image:: ../images/examples/17c_fastfarm_yaw_demo.png

"""

import os
import numpy as np
import multiprocessing as mp
import subprocess
import shutil
import matplotlib.pyplot as plt
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.control_interface import wfc_zmq_server
from rosco import discon_lib_path as lib_name
from rosco.toolbox.ofTools.fast_io import output_processing

TIME_CHECK = 30
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_OUT_DIR = os.path.join(THIS_DIR, "examples_out")
os.makedirs(EXAMPLE_OUT_DIR, exist_ok=True)
DESIRED_YAW_OFFSET = [-20, 20]


def main():
    write_openfast_1()
    write_openfast_2()

    logfile = os.path.join(EXAMPLE_OUT_DIR, os.path.splitext(os.path.basename(__file__))[0] + ".log")
    p0 = mp.Process(target=run_zmq, args=(logfile,))
    p1 = mp.Process(target=run_FF)

    p0.start()
    p1.start()

    p0.join()
    p1.join()
    
    # Check that info is passed to ROSCO for first simulation
    op1 = output_processing.output_processing()
    debug_file1 = os.path.join(
        EXAMPLE_OUT_DIR,
        "17c_FASTFarm.T1.RO.dbg2",
    )
    local_vars1 = op1.load_fast_out(debug_file1, tmin=0)
    
    op2 = output_processing.output_processing()
    debug_file2 = os.path.join(
        EXAMPLE_OUT_DIR,
        "17c_FASTFarm.T2.RO.dbg2",
    )
    local_vars2 = op2.load_fast_out(debug_file2, tmin=0)

    _, axs = plt.subplots(2, 1)
    axs[0].plot(local_vars1[0]["Time"], local_vars1[0]["ZMQ_YawOffset"])
    axs[1].plot(local_vars2[0]["Time"], local_vars2[0]["ZMQ_YawOffset"])

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_ZMQ_Setpoints.png"))
    
    # Spot check input at time = 30 sec.
    ind1_30 = local_vars1[0]["Time"] == TIME_CHECK
    ind2_30 = local_vars2[0]["Time"] == TIME_CHECK

    np.testing.assert_almost_equal(
        local_vars1[0]["ZMQ_YawOffset"][ind1_30], DESIRED_YAW_OFFSET[0]
    )
    np.testing.assert_almost_equal(
        local_vars2[0]["ZMQ_YawOffset"][ind2_30], DESIRED_YAW_OFFSET[1]
    )


def run_zmq(logfile=None):
    """Start the ZeroMQ server for wind farm control"""

    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=False, logfile=logfile)

    # Provide the wind farm control algorithm as the wfc_controller method of the server
    server.wfc_controller = wfc_controller()

    # Run the server to receive measurements and send setpoints
    server.runserver()


class wfc_controller():
    """
    Users needs to define this function to implement wind farm controller.
    The user defined function should take as argument the turbine id, the
    current time and current measurements and return the setpoints
    for the particular turbine for the current time. It should ouput the
    setpoints as a dictionary whose keys should be as defined in
    wfc_zmq_server.wfc_interface. The wfc_controller method of the wfc_zmq_server
    should be overwriten with this fuction, otherwise, an exception is raised and
    the simulation stops.
    """
    
    def __init__(self):
        return None
    
    def update_setpoints(self, id, current_time, measurements):
        if current_time <= 10.0:
            YawOffset = 0.0
        else:
            if id == 1:
                YawOffset = DESIRED_YAW_OFFSET[0]
            else:
                YawOffset = DESIRED_YAW_OFFSET[1]


        setpoints = {}
        setpoints["ZMQ_YawOffset"] = YawOffset
        return setpoints

def write_openfast_1():
    """Run the first OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 20,
    }
    run_dir = os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_OF1")
    r.controller_params = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"] = {}
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 1
    r.controller_params["DISCON"]["Y_ControlMode"] = 1
    r.case_inputs = {}
    r.case_inputs[("ServoDyn","TYCOn")]      = {'vals':[0], 'group':0}
  
    # Use a copy of the discon library for each set of OpenFAST files
    copy_lib = os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_OF1",os.path.basename(lib_name))
    r.rosco_dll =  copy_lib

    r.save_dir = run_dir
    r.execute_fast = False    # execute_fast is set to False to avoid running OpenFAST directly, as simulation will be run using FAST.Farm
    r.run_FAST()
    
    # Copy the discon library
    shutil.copyfile(lib_name, copy_lib)


def write_openfast_2():
    """Run the second OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 20,
    }
    run_dir = os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_OF2")
    r.save_dir = run_dir
    r.controller_params = {}
    r.controller_params["DISCON"] = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 2
    r.controller_params["DISCON"]["Y_ControlMode"] = 1
    r.case_inputs = {}
    r.case_inputs[("ServoDyn","TYCOn")]      = {'vals':[0], 'group':0}
    
    # Use a copy of the discon library for each set of OpenFAST files
    copy_lib = os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_OF2",os.path.basename(lib_name))
    r.rosco_dll =  copy_lib
    
    r.execute_fast = False    # execute_fast is set to False to avoid running OpenFAST directly, as simulation will be run using FAST.Farm
    r.run_FAST()

    # Copy the discon library
    shutil.copyfile(lib_name, copy_lib)


def run_FF():
    fstf_file=os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm.fstf")
    shutil.copyfile(os.path.join(THIS_DIR,"example_inputs","FASTFarm.fstf"), fstf_file)
    shutil.copyfile(os.path.join(THIS_DIR,"example_inputs","FASTFarm_IW.dat"), os.path.join(EXAMPLE_OUT_DIR, "17c_FASTFarm_IW.dat"))

    # Run FAST.Farm with the specified fstf file
    subprocess.run(["FAST.Farm",fstf_file], check=True)


if __name__ == "__main__":
    main()
