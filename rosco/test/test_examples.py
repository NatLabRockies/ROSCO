import os
import unittest
from time import time
import runpy

all_scripts = [
    '01_turbine_model',
    '02_ccblade',
    '03_tune_controller',
    '04_simple_sim',
    '05_openfast_sim',
    '06_peak_shaving',
    '07_openfast_outputs',
    '09_distributed_aero',
    '10_linear_params',
    '11_robust_tuning',
    '12_tune_ipc',
    '14_open_loop_control',
    '15_pass_through',
    '16_external_dll',
    '17a_zeromq_simple',  # only runs on unix in CI
    '17b_zeromq_multi_openfast',  # only runs on unix in CI
    '17c_zeromq_fastfarm',  # only runs on unix in CI
    '18_pitch_faults',
    '19_update_discon_version', 
    '20_active_wake_control',   
    '21_optional_inputs',
    '22_cable_control',
    '23_structural_control',
    '24_floating_feedback',
    '25_rotor_position_control',
    '26_marine_hydro',
    '27_soft_cut_out',
    '28_tower_resonance',
    '29_power_control',
    '30_shutdown',
    '31_fixed_pitch_mhk',
    '32_startup',
    '33_yaw_control',
    '34_discon_windio',
    'update_rosco_discons',     
]

def execute_script(fscript):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    rosco_dir = os.path.dirname( os.path.dirname(this_dir) )
    examples_dir = os.path.join(rosco_dir,'Examples')
    test_case_dir = os.path.join(examples_dir,'Test_Cases')

    # Go to location due to relative path use for airfoil files
    print("\n\n")
    print("NOW RUNNING:", fscript)
    print()

    if fscript in ['update_rosco_discons']:
        run_dir = test_case_dir
    else:
        run_dir = examples_dir

    fullpath = os.path.join(run_dir, fscript + ".py")
    basepath = os.path.dirname(os.path.realpath(fullpath))
    os.chdir(basepath)

    # Use runpy to execute examples
    s = time()
    runpy.run_path(os.path.realpath(fullpath), run_name='__main__')
    print(f"{fscript} took {time() - s} seconds to run")

class TestExamples(unittest.TestCase):

    def test_all_scripts(self):
        for ks, s in enumerate(all_scripts):
            with self.subTest(f"Running: {s}", i=ks):
                try:
                    execute_script(s)
                    self.assertTrue(True)
                except:
                    self.assertEqual(s, "Success")    


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestExamples))
    return suite


if __name__ == "__main__":
    result = unittest.TextTestRunner().run(suite())

    if result.wasSuccessful():
        exit(0)
    else:
        exit(1)
