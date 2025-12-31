"""
34_discon_windio
----------------

Convert between a DISCON.IN file and a windIO turbine control schema YAML file.

Convert from a windIO turbine control schema yaml to a rosco tuning yaml and a DISCON.IN file

Check against the original DISCON.IN file
"""
import os
import numpy as np
from rosco.toolbox.ofTools.util.FileTools import print_yaml
from rosco.toolbox.utilities import read_DISCON
from wisdem.inputs.validation         import simple_types
import windIO
# from windio import TurbineControlSchema
# from rosco.toolbox.ofTools.discon_io import discon_to_rosco_yaml, rosco_yaml_to_discon

radps2rpm = 30.0 / np.pi

def main():

    discon_yaml_map = {
        '/Users/dzalkind/Tools/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT-UMaineSemi/IEA-15-240-RWT-UMaineSemi_DISCON.IN':'/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-15-240-RWT_VolturnUS-S.yaml',
        '/Users/dzalkind/Tools/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT-Monopile/IEA-15-240-RWT-Monopile_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-15-240-RWT.yaml',
        '/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-22-280-RWT_Floater.yaml',
        '/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Monopile/IEA-22-280-RWT_DISCON.IN': '/Users/dzalkind/Tools/windIO/windIO/examples/turbine/IEA-22-280-RWT.yaml'
    }


    examples_dir = os.path.dirname(os.path.abspath(__file__))
    
    for discon_in_file, windio_yaml in discon_yaml_map.items():
        
    
        rosco_vt = read_DISCON(discon_in_file)

        windio_control = {}
        windio_control['min_rotor_speed'] = rosco_vt['VS_MinOMSpd'] * radps2rpm
        windio_control['rated_rotor_speed'] = rosco_vt['PC_RefSpd'] * radps2rpm
        
        windio_control['max_rotor_speed'] = rosco_vt['SD_MaxGenSpd'] * radps2rpm
        windio_control['max_gen_torque'] = rosco_vt['VS_MaxTq'] / 1000 # Nm in ROSCO, kNm in windIO
        windio_control['max_torque_rate'] = rosco_vt['VS_MaxRat'] / 1000 # Nm/s in ROSCO, kNm/s in windIO
        
        windio_control['fine_pitch'] = rosco_vt['PC_FinePit'] * np.rad2deg(1) # radians to degrees

        windio_control['min_pitch_table'] = {}
        windio_control['min_pitch_table']['wind_speed'] = rosco_vt['PS_WindSpeeds']
        windio_control['min_pitch_table']['min_pitch'] = np.array(rosco_vt['PS_BldPitchMin']) * np.rad2deg(1) # radians to degrees

        windio_control['min_pitch_limit'] = rosco_vt['PC_MinPit'] * np.rad2deg(1) # radians to degrees
        windio_control['max_pitch_limit'] = rosco_vt['PC_MaxPit'] * np.rad2deg(1) # radians to degrees
        windio_control['max_pitch_rate'] = rosco_vt['PC_MaxRat'] * np.rad2deg(1) # radians to degrees per second
        
        windio_control['lpf_frequency'] = rosco_vt['F_LPFCornerFreq']
        windio_control['lpf_damping'] = rosco_vt['F_LPFDamping']

        windio_control['region2_k'] = rosco_vt['VS_Rgn2K'] / radps2rpm**2

        windio_control['gen_torque_kp'] = rosco_vt['VS_KP'] / radps2rpm # TODO: check units
        windio_control['gen_torque_ki'] = rosco_vt['VS_KI'] / radps2rpm # TODO: check units

        windio_control['pitch_kp'] = {}
        windio_control['pitch_kp']['pitch_angle'] = np.array(rosco_vt['PC_GS_angles']) * np.rad2deg(1)  # radians to degrees
        windio_control['pitch_kp']['kp'] = np.array(rosco_vt['PC_GS_KP']) * np.rad2deg(1) / radps2rpm # radians to degrees

        windio_control['pitch_ki'] = {}
        windio_control['pitch_ki']['pitch_angle'] = np.array(rosco_vt['PC_GS_angles']) * np.rad2deg(1)  # radians to degrees
        windio_control['pitch_ki']['ki'] = np.array(rosco_vt['PC_GS_KI']) * np.rad2deg(1) / radps2rpm # radians to degrees

        windio_control['constant_power'] = rosco_vt['VS_ConstPower']

        windio_control['gen_actuator_frequency'] = 10000. # no generator actuator model in ROSCO, assumed to be very fast
        windio_control['gen_actuator_damping'] = 1.0 # no generator actuator model in ROSCO, assumed critically damped

        windio_control['pitch_actuator_frequency'] = rosco_vt['PA_CornerFreq']
        windio_control['pitch_actuator_damping'] = rosco_vt['PA_Damping']

        windio_control['yaw_rate'] = rosco_vt['Y_Rate'] * np.rad2deg(1)  # radians to degrees per second

        # Update windio YAML file
        windio_control = simple_types(windio_control)
        windio_dict = windIO.load_yaml(windio_yaml)
        windio_dict['control'] = windio_control

        windIO.yaml.write_yaml(windio_dict, windio_yaml)

    print('here')



if __name__ == "__main__":
    main()