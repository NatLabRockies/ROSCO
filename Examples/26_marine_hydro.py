"""
26_marine_hydro
---------------
Run MHK turbine in OpenFAST with ROSCO torque controller
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl

TEST_RUN = True

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/RM1_MHK.yaml')
    run_dir = os.path.join(example_out_dir,'26_MHK/0_baseline')
    os.makedirs(run_dir,exist_ok=True)

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [2.5],
        'TMax': 100.0,
        }
    if TEST_RUN:
        r.wind_case_opts['TMax'] = 1.0

    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.n_cores = 1

    r.run_FAST()



if __name__=="__main__":
    main()
