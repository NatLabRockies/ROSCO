"""
34_discon_windio
----------------

Convert between a DISCON.IN file and a windIO turbine control schema YAML file.

Convert from a windIO turbine control schema yaml to a rosco tuning yaml and a DISCON.IN file

Check against the original DISCON.IN file

"""
import os
import numpy as np
from rosco.toolbox.utilities import read_DISCON
from rosco.toolbox.inputs.windio     import windio_to_discon, discon_to_windio
from wisdem.inputs.validation         import simple_types
import windIO


def main():
    examples_dir = os.path.dirname(os.path.abspath(__file__))

    # Within ROSCO repo
    discon_yaml_map = {
        os.path.join(examples_dir,'Test_Cases','IEA-15-240-RWT', 'IEA-15-240-RWT-UMaineSemi', 'IEA-15-240-RWT-UMaineSemi_DISCON.IN'): os.path.join(examples_dir,'example_inputs', 'windio', 'IEA-15-240-RWT.yaml'),
        os.path.join(examples_dir,'Test_Cases','IEA-15-240-RWT', 'IEA-15-240-RWT-Monopile', 'IEA-15-240-RWT-Monopile_DISCON.IN'): os.path.join(examples_dir,'example_inputs', 'windio', 'IEA-15-240-RWT.yaml'),
    }

    # # For updating windIO repo
    # discon_yaml_map = {
    #     'IEA-15_Repo_Dir/OpenFAST/IEA-15-240-RWT-UMaineSemi/IEA-15-240-RWT-UMaineSemi_DISCON.IN':'windio_repo_dir/windIO/examples/turbine/IEA-15-240-RWT_VolturnUS-S.yaml',
    #     'IEA-15_Repo_Dir/OpenFAST/IEA-15-240-RWT-Monopile/IEA-15-240-RWT-Monopile_DISCON.IN': 'windio_repo_dir/windIO/examples/turbine/IEA-15-240-RWT.yaml',
    #     'IEA-22_Repo_Dir/OpenFAST/IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi_DISCON.IN': 'windio_repo_dir/windIO/examples/turbine/IEA-22-280-RWT_Floater.yaml',
    #     'IEA-22_Repo_Dir/OpenFAST/IEA-22-280-RWT-Monopile/IEA-22-280-RWT_DISCON.IN': 'windio_repo_dir/windIO/examples/turbine/IEA-22-280-RWT.yaml'
    # }
    
    for discon_in_file, windio_yaml in discon_yaml_map.items():

        # Read DISCON.IN file
        rosco_vt = read_DISCON(discon_in_file)

        # Convert to windIO control dictionary
        windio_control = discon_to_windio(rosco_vt)

        # Update windio YAML file
        windio_control = simple_types(windio_control)
        windio_dict = windIO.load_yaml(windio_yaml)
        windio_dict['control'] = windio_control

        windIO.yaml.write_yaml(windio_dict, windio_yaml)

        # Do the opposite of the above and convert a windio yaml to a DISCON.IN file
        windio_loaded = windIO.load_yaml(windio_yaml)
        rosco_vt_from_windio = windio_to_discon(windio_loaded)


        # Compare with original DISCON values
        print(f"Comparing values for {discon_in_file}:")
        error = False
        for key in rosco_vt_from_windio.keys():
            if key in rosco_vt:
                try:
                    np.testing.assert_allclose(rosco_vt[key], rosco_vt_from_windio[key], atol = 1e-3)
                except AssertionError:
                    error = True
                    print(f"{key}: Original={rosco_vt[key]}, Converted={rosco_vt_from_windio[key]}")

        if error:
            raise ValueError("Discrepancies found between original DISCON values and those converted from windIO YAML.")




if __name__ == "__main__":
    main()