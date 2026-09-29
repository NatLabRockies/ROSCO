"""
23_structural_control
---------------------
Run openfast with ROSCO and structural control
Set up and run simulation with pitch offsets, check outputs

ROSCO currently supports user-defined hooks for structural control control actuation, if StC\_Mode = 1.
The control logic can be determined in Controllers.f90 with the StructrualControl subroutine.
In the DISCON input, users must specify StC\_GroupIndex relating to the control ChannelID.  
These indices can be found in the ServoDyn summary file (\*SrvD.sum)

In the example below, we implement a smooth step change mimicing the exchange of ballast from the 
upwind column to the down wind columns

OpenFAST v3.5.0 is required to run this example
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#import numpy as np
from openfast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.controller import OpenLoopControl

TEST_MODE = True # set to False to run full simulation, True runs a short test case for debugging

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW_ballast.yaml')
    run_dir = os.path.join(example_out_dir,'23_structural_control')
    os.makedirs(run_dir,exist_ok=True)

    # Read initial input file
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']

    # Change inputs programatically, read first
    reader = InputReader_OpenFAST()
    reader.FAST_InputFile = path_params['FAST_InputFile']
    reader.FAST_directory = os.path.join(this_dir,'Tune_Cases',path_params['FAST_directory'])
    # reader.FAST_directory = '/Users/dzalkind/Tools/ROSCO1/Test_Cases/ptfm_control_archive/IEA-15-240-RWT-UMaineSemi_ballast'
    reader.execute()

    reader.fst_vt['ServoDyn']['NumSStC'] = 3
    reader.fst_vt['ServoDyn']['SStCfiles'] = ['../StC-Force-Col1.dat', '../StC-Force-Col2.dat', '../StC-Force-Col3.dat']
    # Add SStC file inputs
    for StC_file in reader.fst_vt['ServoDyn']['SStCfiles']:
        reader.fst_vt['SStC'].append(reader.read_StC(StC_file))

    # Set up open loop inputs to ROSCO
    t_trans = 60
    t_sigma = 80
    t_max = 200

    applied_force = [-2e6, 1e6, 1e6]

    olc = OpenLoopControl(t_max=t_max)
    olc.interp_series(
        'struct_control_1', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,applied_force[0]] , 
        'sigma'
        )
    
    olc.interp_series(
        'struct_control_2', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,applied_force[1]] , 
        'sigma'
        )
    
    olc.interp_series(
        'struct_control_3', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,applied_force[2]] , 
        'sigma'
        )

    
    ol_params = olc.write_input(os.path.join(run_dir,'open_loop_ballast.dat'))

    controller_params = {}
    controller_params['open_loop'] = ol_params
    controller_params['StC_Mode'] = 2

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [9],
        'TMax': t_max if not TEST_MODE else 10,
        }
    r.case_inputs = {}
    r.fst_vt            = reader.fst_vt
    r.save_dir          = run_dir
    r.rosco_dir         = rosco_dir
    r.controller_params = controller_params
    r.run_FAST()


if __name__=="__main__":
    main()
