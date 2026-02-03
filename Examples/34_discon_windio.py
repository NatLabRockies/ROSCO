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

    discon_yaml_map = {
        '/Users/dzalkind/Tools/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT-UMaineSemi/IEA-15-240-RWT-UMaineSemi_DISCON.IN':'/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-15-240-RWT_VolturnUS-S.yaml',
        '/Users/dzalkind/Tools/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT-Monopile/IEA-15-240-RWT-Monopile_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-15-240-RWT.yaml',
        '/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-22-280-RWT_Floater.yaml',
        '/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Monopile/IEA-22-280-RWT_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-22-280-RWT.yaml'
    }
    
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