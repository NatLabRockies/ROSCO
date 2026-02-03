import numpy as np
radps2rpm = 30.0 / np.pi

def windio_to_discon(windio_dict):
    """
    Convert windIO control dictionary to ROSCO format.
    
    Args:
        windio_dict: Dictionary containing windIO turbine data with control section
        
    Returns:
        dict: ROSCO format control parameters
    """
    rosco_vt_from_windio = {}

    # Convert back from windIO to ROSCO units and format
    rosco_vt_from_windio['VS_MinOMSpd']     = windio_dict['control']['min_rotor_speed'] / radps2rpm
    rosco_vt_from_windio['PC_RefSpd']       = windio_dict['control']['rated_rotor_speed'] / radps2rpm
    rosco_vt_from_windio['VS_RtPwr']        = windio_dict['control']['rated_power']  # W in windIO, W in ROSCO
    rosco_vt_from_windio['SD_MaxGenSpd']    = windio_dict['control']['max_rotor_speed'] / radps2rpm
    rosco_vt_from_windio['VS_MaxTq']        = windio_dict['control']['max_gen_torque']
    rosco_vt_from_windio['VS_MaxRat']       = windio_dict['control']['max_torque_rate']

    rosco_vt_from_windio['PC_FinePit']      = windio_dict['control']['fine_pitch'] * np.deg2rad(1)
    rosco_vt_from_windio['VS_TSRopt']       = windio_dict['control']['optimal_tsr']

    rosco_vt_from_windio['PS_WindSpeeds']   = windio_dict['control']['min_pitch_table']['wind_speed']
    rosco_vt_from_windio['PS_BldPitchMin']  = np.array(windio_dict['control']['min_pitch_table']['min_pitch']) * np.deg2rad(1)

    rosco_vt_from_windio['PC_MinPit'] = windio_dict['control']['min_pitch_limit'] * np.deg2rad(1)
    rosco_vt_from_windio['PC_MaxPit'] = windio_dict['control']['max_pitch_limit'] * np.deg2rad(1)
    rosco_vt_from_windio['PC_MaxRat'] = windio_dict['control']['max_pitch_rate'] * np.deg2rad(1)

    rosco_vt_from_windio['F_LPFCornerFreq'] = windio_dict['control']['lpf_frequency']
    rosco_vt_from_windio['F_LPFDamping']    = windio_dict['control']['lpf_damping']

    rosco_vt_from_windio['VS_Rgn2K'] = windio_dict['control']['region2_k'] * radps2rpm**2

    rosco_vt_from_windio['VS_KP'] = windio_dict['control']['gen_torque_kp'] * radps2rpm
    rosco_vt_from_windio['VS_KI'] = windio_dict['control']['gen_torque_ki'] * radps2rpm

    rosco_vt_from_windio['PC_GS_angles']    = np.array(windio_dict['control']['pitch_kp']['pitch_angle']) * np.deg2rad(1)
    rosco_vt_from_windio['PC_GS_KP']        = np.array(windio_dict['control']['pitch_kp']['kp']) * np.deg2rad(1) * radps2rpm
    rosco_vt_from_windio['PC_GS_KI']        = np.array(windio_dict['control']['pitch_ki']['ki']) * np.deg2rad(1) * radps2rpm

    rosco_vt_from_windio['VS_ConstPower'] = windio_dict['control']['constant_power']

    rosco_vt_from_windio['PA_CornerFreq']   = windio_dict['control']['pitch_actuator_frequency']
    rosco_vt_from_windio['PA_Damping']      = windio_dict['control']['pitch_actuator_damping']

    rosco_vt_from_windio['Y_Rate'] = windio_dict['control']['yaw_rate'] * np.deg2rad(1)
    
    return rosco_vt_from_windio

def discon_to_windio(rosco_vt):

    windio_control = {}
    windio_control['min_rotor_speed']   = rosco_vt['VS_MinOMSpd'] * radps2rpm
    windio_control['rated_rotor_speed'] = rosco_vt['PC_RefSpd'] * radps2rpm
    windio_control['rated_power']       = rosco_vt['VS_RtPwr']  # W in ROSCO, W in windIO
    
    windio_control['max_rotor_speed']   = rosco_vt['SD_MaxGenSpd'] * radps2rpm
    windio_control['max_gen_torque']    = rosco_vt['VS_MaxTq']  # Nm in ROSCO, N*m in windIO
    windio_control['max_torque_rate']   = rosco_vt['VS_MaxRat']  # Nm/s in ROSCO, N*m/s in windIO
    
    windio_control['fine_pitch'] = rosco_vt['PC_FinePit'] * np.rad2deg(1) # radians to degrees
    windio_control['optimal_tsr'] = rosco_vt['VS_TSRopt'] # dimensionless

    windio_control['min_pitch_table'] = {}
    windio_control['min_pitch_table']['wind_speed'] = rosco_vt['PS_WindSpeeds']
    windio_control['min_pitch_table']['min_pitch']  = np.array(rosco_vt['PS_BldPitchMin']) * np.rad2deg(1) # radians to degrees

    windio_control['min_pitch_limit']   = rosco_vt['PC_MinPit'] * np.rad2deg(1) # radians to degrees
    windio_control['max_pitch_limit']   = rosco_vt['PC_MaxPit'] * np.rad2deg(1) # radians to degrees
    windio_control['max_pitch_rate']    = rosco_vt['PC_MaxRat'] * np.rad2deg(1) # radians to degrees per second
    
    windio_control['lpf_frequency'] = rosco_vt['F_LPFCornerFreq']
    windio_control['lpf_damping']   = rosco_vt['F_LPFDamping']

    windio_control['region2_k'] = rosco_vt['VS_Rgn2K'] / radps2rpm**2

    windio_control['gen_torque_kp'] = rosco_vt['VS_KP'] / radps2rpm
    windio_control['gen_torque_ki'] = rosco_vt['VS_KI'] / radps2rpm

    windio_control['pitch_kp'] = {}
    windio_control['pitch_kp']['pitch_angle']   = np.array(rosco_vt['PC_GS_angles']) * np.rad2deg(1)  # radians to degrees
    windio_control['pitch_kp']['kp']            = np.array(rosco_vt['PC_GS_KP']) * np.rad2deg(1) / radps2rpm # radians to degrees

    windio_control['pitch_ki'] = {}
    windio_control['pitch_ki']['pitch_angle']   = np.array(rosco_vt['PC_GS_angles']) * np.rad2deg(1)  # radians to degrees
    windio_control['pitch_ki']['ki']            = np.array(rosco_vt['PC_GS_KI']) * np.rad2deg(1) / radps2rpm # radians to degrees

    windio_control['constant_power'] = rosco_vt['VS_ConstPower']

    windio_control['gen_actuator_frequency']    = 10000. # no generator actuator model in ROSCO, assumed to be very fast
    windio_control['gen_actuator_damping']      = 1.0 # no generator actuator model in ROSCO, assumed critically damped

    windio_control['pitch_actuator_frequency']  = rosco_vt['PA_CornerFreq']
    windio_control['pitch_actuator_damping']    = rosco_vt['PA_Damping']

    windio_control['yaw_rate'] = rosco_vt['Y_Rate'] * np.rad2deg(1)  # radians to degrees per second

    return windio_control