# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
"""
Class FileProcessing used to write out controller 
    parameter files need to run ROSCO

Methods:
-----------
run_openfast
write_DISCON
read_DISCON
write_rotor_performance
load_from_txt
DISCON_dict
list_check
"""
import datetime
import os
import ast
import numpy as np
import subprocess
import rosco.toolbox
from wisdem.inputs import load_yaml

from wisdem.inputs import load_yaml

from wisdem.inputs.validation         import simple_types

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

def write_DISCON(turbine, controller, param_file='DISCON.IN', txt_filename='Cp_Ct_Cq.txt', rosco_vt = {}):
    """
    Print the controller parameters to the DISCON.IN input file for the generic controller

    Parameters:
    -----------
    turbine: class
                Turbine class containing turbine operation information (ref speeds, etc...)
    controller: class
                Controller class containing controller operation information (gains, etc...)
    param_file: str, optional
        filename for parameter input file, should be DISCON.IN
    txt_filename: str, optional
                    filename of rotor performance file
    """

    # Get ROSCO var tree if not provided
    if not rosco_vt:
        rosco_vt = DISCON_dict(turbine, controller, txt_filename)

    # Get input descriptions from input schema
    fname_schema = os.path.join(os.path.dirname(__file__),'inputs/toolbox_schema.yaml')
    sch = load_yaml(fname_schema)
    
    # mode descriptions in main controller_params, might not be needed
    mode_descriptions = {}
    for input, props in sch['properties']['controller_params']['properties'].items():
        if 'description' in props:
            mode_descriptions[input] = props['description']

    input_descriptions = {}
    for input, props in sch['properties']['controller_params']['properties']['DISCON']['properties'].items():
        if 'description' in props:
            input_descriptions[input] = props['description']

    # Tidy inputs, if needed
    if not hasattr(rosco_vt['CC_GroupIndex'],'__len__'):
        rosco_vt['CC_GroupIndex'] = [rosco_vt['CC_GroupIndex']]     # make an array
    if not hasattr(rosco_vt['StC_GroupIndex'],'__len__'):
        rosco_vt['StC_GroupIndex'] = [rosco_vt['StC_GroupIndex']]   

    print('Writing new controller parameter file parameter file: %s.' % param_file)
    # Should be obvious what's going on here...
    file = open(param_file,'w')
    file.write('! Controller parameter input file for the %s wind turbine\n' % turbine.TurbineName)
    file.write('!    - File written using ROSCO version {} controller tuning logic on {}\n'.format(rosco.toolbox.__version__, now.strftime('%m/%d/%y')))
    file.write('\n')
    file.write('!------- SIMULATION CONTROL ------------------------------------------------------------\n')
    file.write('{0:<12d}        ! LoggingLevel		- 0: write no debug files, 1: write standard output .dbg-file, 2: LoggingLevel 1 + ROSCO LocalVars (.dbg2) 3: LoggingLevel 2 + complete avrSWAP-array (.dbg3)\n'.format(int(rosco_vt['LoggingLevel'])))
    file.write('{}                   ! DT_Out    		  - Time step to output .dbg* files, or 0 to match sampling period of OpenFAST\n'.format(rosco_vt['DT_Out']))
    file.write('{:<11d}         ! Ext_Interface		- ({})\n'.format(int(rosco_vt['Ext_Interface']), input_descriptions['Ext_Interface']))
    file.write('{:<11d}         ! Echo		        - ({})\n'.format(int(rosco_vt['Echo']), input_descriptions['Echo']))
    file.write('\n')
    file.write('!------- CONTROLLER FLAGS -------------------------------------------------\n')
    file.write('{0:<12d}        ! F_LPFType       - 1: first-order low-pass filter, 2: second-order low-pass filter, [rad/s] (currently filters generator speed and pitch control signals\n'.format(int(rosco_vt['F_LPFType'])))
    file.write('{0:<12d}        ! IPC_ControlMode - Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) (0: off, 1: 1P reductions, 2: 1P+2P reductions)\n'.format(int(rosco_vt['IPC_ControlMode'])))
    file.write('{0:<12d}        ! VS_ControlMode  - Generator torque control mode in above rated conditions (0- no torque control, 1- k*omega^2 with PI transitions, 2- WSE TSR Tracking, 3- Power-based TSR Tracking, 4- Torque-based TSR Tracking)\n'.format(int(rosco_vt['VS_ControlMode'])))
    file.write('{0:<12d}        ! VS_ConstPower   - Do constant power torque control, where above rated torque varies, 0 for constant torque)\n'.format(int(rosco_vt['VS_ConstPower'])))
    file.write('{0:<12d}        ! VS_FBP          - Fixed blade pitch configuration mode (0- variable pitch (disabled), 1- constant power overspeed, 2- WSE-lookup reference tracking, 3- torque-lookup reference tracking)\n'.format(int(rosco_vt['VS_FBP'])))
    file.write('{0:<12d}        ! PC_ControlMode  - Blade pitch control mode (0: No pitch, fix to fine pitch, 1: active PI blade pitch control)\n'.format(int(rosco_vt['PC_ControlMode'])))
    file.write('{0:<12d}        ! Y_ControlMode   - Yaw control mode (0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC)\n'.format(int(rosco_vt['Y_ControlMode'])))
    file.write('{0:<12d}        ! SS_Mode         - Setpoint Smoother mode (0: no setpoint smoothing, 1: introduce setpoint smoothing)\n'.format(int(rosco_vt['SS_Mode'])))
    file.write('{0:<12d}        ! PRC_Mode        - Power reference tracking mode (0: power control disabled, 1: lookup table from wind speed to generator speed setpoints, 2: change speed, torque, pitch to control power)\n'.format(int(rosco_vt['PRC_Mode'])))
    file.write('{0:<12d}        ! WE_Mode         - Wind speed estimator mode (0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator, 2: Extended Kalman Filter)\n'.format(int(rosco_vt['WE_Mode'])))
    file.write('{0:<12d}        ! PS_Mode         - Pitch saturation mode (0: no pitch saturation, 1: implement pitch saturation)\n'.format(int(rosco_vt['PS_Mode'])))
    file.write('{0:<12d}        ! SU_Mode         - Startup mode (0: no startup procedure, 1: startup enabled)\n'.format(int(rosco_vt['SU_Mode'])))
    file.write('{0:<12d}        ! SD_Mode         - Shutdown mode (0: no shutdown procedure, 1: shutdown enabled)\n'.format(int(rosco_vt['SD_Mode'])))
    file.write('{0:<12d}        ! Fl_Mode         - Floating specific feedback mode (0: no nacelle velocity feedback, 1: feed back translational velocity, 2: feed back rotational velocity)\n'.format(int(rosco_vt['Fl_Mode'])))
    file.write('{:<12d}        ! TD_Mode         - {}\n'.format(int(rosco_vt['TD_Mode']),mode_descriptions['TD_Mode']))
    file.write('{:<12d}        ! TRA_Mode        - {}\n'.format(int(rosco_vt['TRA_Mode']),mode_descriptions['TRA_Mode']))
    file.write('{0:<12d}        ! Flp_Mode        - Flap control mode (0: no flap control, 1: steady state flap angle, 2: Proportional flap control, 2: Cyclic (1P) flap control)\n'.format(int(rosco_vt['Flp_Mode'])))
    file.write('{0:<12d}        ! OL_Mode         - Open loop control mode (0: no open loop control, 1: open loop control vs. time, 2: rotor position control)\n'.format(int(rosco_vt['OL_Mode'])))
    file.write('{0:<12d}        ! PA_Mode         - Pitch actuator mode (0 - not used, 1 - first order filter, 2 - second order filter)\n'.format(int(rosco_vt['PA_Mode'])))
    file.write('{0:<12d}        ! PF_Mode         - Pitch fault mode (0 - not used, 1 - constant offset on one or more blades)\n'.format(int(rosco_vt['PF_Mode'])))
    file.write('{0:<12d}        ! AWC_Mode        - Active wake control (0 - not used, 1 - complex number method, 2 - Coleman transform method)\n'.format(int(rosco_vt['AWC_Mode'])))
    file.write('{0:<12d}        ! Ext_Mode        - External control mode (0 - not used, 1 - call external dynamic library)\n'.format(int(rosco_vt['Ext_Mode'])))
    file.write('{0:<12d}        ! ZMQ_Mode        - Fuse ZeroMQ interface (0: unused, 1: Yaw Control)\n'.format(int(rosco_vt['ZMQ_Mode'])))
    file.write('{:<12d}        ! CC_Mode         - {}\n'.format(int(rosco_vt['CC_Mode']),mode_descriptions['CC_Mode']))
    file.write('{:<12d}        ! StC_Mode        - {}\n'.format(int(rosco_vt['StC_Mode']),mode_descriptions['StC_Mode']))

    file.write('\n')
    file.write('!------- FILTERS ----------------------------------------------------------\n') 
    file.write('{:<13.5f}       ! F_LPFCornerFreq	  - Corner frequency (-3dB point) in the low-pass filters, [rad/s]\n'.format(rosco_vt['F_LPFCornerFreq'])) 
    file.write('{:<13.5f}       ! F_LPFDamping		  - Damping coefficient (used only when F_FilterType = 2) [-]\n'.format(rosco_vt['F_LPFDamping']))
    file.write('{:<12d}        ! F_NumNotchFilts   - {}\n'.format(int(rosco_vt["F_NumNotchFilts"]), input_descriptions["F_NumNotchFilts"]))
    file.write('{}        ! F_NotchFreqs      - {}\n'.format(write_array(rosco_vt["F_NotchFreqs"]), input_descriptions["F_NotchFreqs"]))
    file.write('{}        ! F_NotchBetaNum    - {}\n'.format(write_array(rosco_vt['F_NotchBetaNum']), input_descriptions["F_NotchBetaNum"]))
    file.write('{}        ! F_NotchBetaDen    - {}\n'.format(write_array(rosco_vt['F_NotchBetaDen']), input_descriptions["F_NotchBetaDen"]))
    file.write('{:<12d}        ! F_GenSpdNotch_N   - {}\n'.format(int(rosco_vt['F_GenSpdNotch_N']), input_descriptions["F_GenSpdNotch_N"]))
    file.write('{}        ! F_GenSpdNotch_Ind - {}\n'.format(write_array(rosco_vt['F_GenSpdNotch_Ind'],'d'), input_descriptions["F_GenSpdNotch_Ind"]))
    file.write('{:<12d}        ! F_TwrTopNotch_N   - {}\n'.format(int(rosco_vt['F_TwrTopNotch_N']), input_descriptions["F_TwrTopNotch_N"]))
    file.write('{}        ! F_TwrTopNotch_Ind - {}\n'.format(write_array(rosco_vt['F_TwrTopNotch_Ind'],'d'), input_descriptions["F_TwrTopNotch_Ind"]))
    file.write('{:<13.5f}       ! F_SSCornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother, [rad/s].\n'.format(rosco_vt['F_SSCornerFreq']))
    file.write('{:<13.5f}       ! F_WECornerFreq    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].\n'.format(rosco_vt['F_WECornerFreq']))
    file.write('{:<13.5f}       ! F_YawErr          - Low pass filter corner frequency for yaw controller [rad/s].\n'.format(rosco_vt['F_YawErr']))
    file.write('{}  ! F_FlCornerFreq    - Natural frequency and damping in the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s, -].\n'.format(write_array(rosco_vt['F_FlCornerFreq'],'<8.4f')))
    file.write('{:<13.5f}       ! F_FlHighPassFreq  - Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s].\n'.format(rosco_vt['F_FlHighPassFreq']))
    file.write('{}    ! F_FlpCornerFreq   - {}\n'.format(write_array(rosco_vt["F_FlpCornerFreq"], '<7.4f'), input_descriptions["F_FlpCornerFreq"]))
    file.write('{:<13.5f}       ! F_VSRefSpdCornerFreq		- {}\n'.format(float(rosco_vt['F_VSRefSpdCornerFreq']),input_descriptions['F_VSRefSpdCornerFreq']))
    
    file.write('\n')
    file.write('!------- BLADE PITCH CONTROL ----------------------------------------------\n')
    file.write('{:<11d}         ! PC_GS_n			- Amount of gain-scheduling table entries\n'.format(int(rosco_vt['PC_GS_n'])))
    file.write('{}              ! PC_GS_angles	    - Gain-schedule table: pitch angles [rad].\n'.format(write_array(rosco_vt['PC_GS_angles'],'<12.3f')))            
    file.write('{}              ! PC_GS_KP		- Gain-schedule table: pitch controller kp gains [s].\n'.format(write_array(rosco_vt['PC_GS_KP'],'<12.3e')))
    file.write('{}              ! PC_GS_KI		- Gain-schedule table: pitch controller ki gains [-].\n'.format(write_array(rosco_vt['PC_GS_KI'],'<12.3e')))
    file.write('{}              ! PC_GS_KD			- Gain-schedule table: pitch controller kd gains\n'.format(write_array(rosco_vt['PC_GS_KD'],'<12.3e')))
    file.write('{}              ! PC_GS_TF			- Gain-schedule table: pitch controller tf gains (derivative filter)\n'.format(write_array(rosco_vt['PC_GS_TF'],'<12.3e')))
    file.write('{:<014.5f}      ! PC_MaxPit			- Maximum physical pitch limit, [rad].\n'.format(rosco_vt['PC_MaxPit']))
    file.write('{:<014.5f}      ! PC_MinPit			- Minimum physical pitch limit, [rad].\n'.format(rosco_vt['PC_MinPit']))
    file.write('{:<014.5f}      ! PC_MaxRat			- Maximum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(rosco_vt['PC_MaxRat']))
    file.write('{:<014.5f}      ! PC_MinRat			- Minimum pitch rate (in absolute value) in pitch controller, [rad/s].\n'.format(rosco_vt['PC_MinRat']))
    file.write('{:<014.5f}      ! PC_RefSpd			- Desired (reference) HSS speed for pitch controller, [rad/s].\n'.format(rosco_vt['PC_RefSpd']))
    file.write('{:<014.5f}      ! PC_FinePit		- Record 5: Below-rated pitch angle set-point, [rad]\n'.format(rosco_vt['PC_FinePit']))
    file.write('{:<014.5f}      ! PC_Switch			- Angle above lowest minimum pitch angle for switch, [rad]\n'.format(rosco_vt['PC_Switch']))
    file.write('\n')
    file.write('!------- INDIVIDUAL PITCH CONTROL -----------------------------------------\n')
    file.write('{}! IPC_Vramp		- Start and end wind speeds for cut-in ramp function. First entry: IPC inactive, second entry: IPC fully active. [m/s]\n'.format(write_array(rosco_vt['IPC_Vramp'],'<12.3f')))
    file.write('{:<11d}         ! IPC_SatMode		- IPC Saturation method (0 - no saturation (except by PC_MinPit), 1 - saturate by PS_BldPitchMin, 2 - saturate sotfly (full IPC cycle) by PC_MinPit, 3 - saturate softly by PS_BldPitchMin)\n'.format(int(rosco_vt['IPC_SatMode']))) # Hardcode to 5 degrees
    file.write('{:<13.1f}       ! IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from IPC), [rad]\n'.format(rosco_vt['IPC_IntSat'])) 
    file.write('{}! IPC_KP			- Proportional gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(write_array(rosco_vt['IPC_KP'],'<12.3e')))
    file.write('{}! IPC_KI			- Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-]\n'.format(write_array(rosco_vt['IPC_KI'],'<12.3e')))
    file.write('{}! IPC_aziOffset		- Phase offset added to the azimuth angle for the individual pitch controller, [rad]. \n'.format(write_array(rosco_vt['IPC_aziOffset'],'<12.3f')))
    file.write('{:<13.1f}       ! IPC_CornerFreqAct - Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal (0: Disable), [rad/s]\n'.format(rosco_vt['IPC_CornerFreqAct']))
    file.write('\n')
    file.write('!------- VS TORQUE CONTROL ------------------------------------------------\n')
    file.write('{:<14.5f}      ! VS_GenEff			- Generator efficiency mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [%]\n'.format(rosco_vt['VS_GenEff']))
    file.write('{:<14.5e}      ! VS_ArSatTq		- Above rated generator torque PI control saturation, [Nm]\n'.format(rosco_vt['VS_ArSatTq']))
    file.write('{:<14.5e}      ! VS_MaxRat			- Maximum torque rate (in absolute value) in torque controller, [Nm/s].\n'.format(rosco_vt['VS_MaxRat']))
    file.write('{:<14.5e}      ! VS_MaxTq			- Maximum generator torque in Region 3 (HSS side), [Nm].\n'.format(rosco_vt['VS_MaxTq']))
    file.write('{:<14.5e}      ! VS_MinTq			- Minimum generator torque (HSS side), [Nm].\n'.format(rosco_vt['VS_MinTq']))
    file.write('{:<14.5f}      ! VS_MinOMSpd		- Minimum generator speed [rad/s]\n'.format(rosco_vt['VS_MinOMSpd']))
    file.write('{:<14.5e}      ! VS_Rgn2K		- {}\n'.format(float(rosco_vt['VS_Rgn2K']),input_descriptions['VS_Rgn2K']))
    file.write('{:<14.5e}      ! VS_RtPwr			- Wind turbine rated power [W]\n'.format(rosco_vt['VS_RtPwr']))
    file.write('{:<14.5e}      ! VS_RtTq			- Rated torque, [Nm].\n'.format(rosco_vt['VS_RtTq']))
    file.write('{:<14.5f}      ! VS_RefSpd			- Rated generator speed [rad/s]\n'.format(rosco_vt['VS_RefSpd']))
    file.write('{:<14d}      ! VS_n				- Number of generator PI torque controller gains\n'.format(int(rosco_vt['VS_n'])))
    file.write('{:<14.5e}      ! VS_KP				- Proportional gain for generator PI torque controller [-]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(rosco_vt['VS_KP']))
    file.write('{:<14.5e}      ! VS_KI				- Integral gain for generator PI torque controller [s]. (Only used in the transitional 2.5 region if VS_ControlMode =/ 2)\n'.format(rosco_vt['VS_KI']))
    file.write('{:<14.5f}      ! VS_TSRopt		    - {}\n'.format(float(rosco_vt['VS_TSRopt']),input_descriptions['VS_TSRopt']))
    file.write('\n')
    file.write('!------- FIXED PITCH REGION 3 TORQUE CONTROL ------------------------------------------------\n')
    file.write('{:<11d}         ! VS_FBP_n          - Number of gain-scheduling table entries\n'.format(int(rosco_vt['VS_FBP_n'])))
    file.write('{}              ! VS_FBP_U          - Operating schedule table: Wind speeds [m/s].\n'.format(write_array(rosco_vt["VS_FBP_U"],'<12.4f')))
    file.write('{}              ! VS_FBP_Omega      - Operating schedule table: Generator speeds [rad/s].\n'.format(write_array(rosco_vt["VS_FBP_Omega"],'<12.4e')))
    file.write('{}              ! VS_FBP_Tau        - Operating schedule table: Generator torques [N m].\n'.format(write_array(rosco_vt["VS_FBP_Tau"],'<12.4e')))
    file.write('\n')
    file.write('!------- SETPOINT SMOOTHER ---------------------------------------------\n')
    file.write('{:<13.5f}       ! SS_VSGain         - Variable speed torque controller setpoint smoother gain, [-].\n'.format(rosco_vt['SS_VSGain']))
    file.write('{:<13.5f}       ! SS_PCGain         - Collective pitch controller setpoint smoother gain, [-].\n'.format(rosco_vt['SS_PCGain']))
    file.write('\n')
    file.write('!------- POWER REFERENCE TRACKING --------------------------------------\n')
    file.write('{:<11d}         ! PRC_Comm   - {}\n'.format(int(rosco_vt['PRC_Comm']), input_descriptions["PRC_Comm"]))
    file.write('{:<13.5f}       ! PRC_R_Torque   - {}\n'.format(float(rosco_vt['PRC_R_Torque']), input_descriptions["PRC_R_Torque"]))
    file.write('{:<13.5f}       ! PRC_R_Speed   - {}\n'.format(float(rosco_vt['PRC_R_Speed']), input_descriptions["PRC_R_Speed"]))
    file.write('{:<13.5f}       ! PRC_R_Pitch   - {}\n'.format(float(rosco_vt['PRC_R_Pitch']), input_descriptions["PRC_R_Pitch"]))
    file.write('{:<11d}         ! PRC_Table_n   - {}\n'.format(int(rosco_vt['PRC_Table_n']), input_descriptions["PRC_Table_n"]))
    file.write('{}     ! PRC_R_Table   - {}\n'.format(write_array(rosco_vt["PRC_R_Table"],'<12.4f'), input_descriptions["PRC_R_Table"]))
    file.write('{}     ! PRC_Pitch_Table   - {}\n'.format(write_array(rosco_vt["PRC_Pitch_Table"],'<12.4f'), input_descriptions["PRC_Pitch_Table"]))
    file.write('{:<11d}         ! PRC_n			    -  Number of elements in PRC_WindSpeeds and PRC_GenSpeeds array\n'.format(int(rosco_vt['PRC_n'])))
    file.write('{:<13.5f}       ! PRC_LPF_Freq   - {}\n'.format(float(rosco_vt['PRC_LPF_Freq']), input_descriptions["PRC_LPF_Freq"]))
    file.write('{}     ! PRC_WindSpeeds   - {}\n'.format(write_array(rosco_vt["PRC_WindSpeeds"],'<12.4f'), input_descriptions["PRC_WindSpeeds"]))
    file.write('{}      ! PRC_GenSpeeds   - {}\n'.format(write_array(rosco_vt["PRC_GenSpeeds"],'<12.4e'), input_descriptions["PRC_GenSpeeds"]))
    file.write('\n')
    file.write('!------- WIND SPEED ESTIMATOR ---------------------------------------------\n')
    file.write('{:<13.3f}       ! WE_BladeRadius	- Blade length (distance from hub center to blade tip), [m]\n'.format(rosco_vt['WE_BladeRadius']))
    file.write('{:<11d}         ! WE_CP_n			- Amount of parameters in the Cp array\n'.format(int(rosco_vt['WE_CP_n'])))
    file.write('{:<13.1f}       ! WE_CP - Parameters that define the parameterized CP(lambda) function\n'.format(rosco_vt['WE_CP']))
    file.write('{:<13.1f}		  ! WE_Gamma			- Adaption gain of the wind speed estimator algorithm [m/rad]\n'.format(rosco_vt['WE_Gamma']))
    file.write('{:<13.1f}       ! WE_GearboxRatio	- Gearbox ratio [>=1],  [-]\n'.format(rosco_vt['WE_GearboxRatio']))
    file.write('{:<14.5f}     ! WE_Jtot			- Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]\n'.format(rosco_vt['WE_Jtot']))
    file.write('{:<13.3f}       ! WE_RhoAir			- Air density, [kg m^-3]\n'.format(rosco_vt['WE_RhoAir']))
    file.write(      '"{}"      ! PerfFileName      - File containing rotor performance tables (Cp,Ct,Cq) (absolute path or relative to this file)\n'.format(rosco_vt['PerfFileName']))
    file.write('{:<7d} {:<10d}  ! PerfTableSize     - Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios\n'.format(int(rosco_vt['PerfTableSize'][0]),int(rosco_vt['PerfTableSize'][1])))
    file.write('{:<11d}         ! WE_FOPoles_N      - Number of first-order system poles used in EKF\n'.format(int(rosco_vt['WE_FOPoles_N'])))
    file.write('{}              ! WE_FOPoles_v      - Wind speeds corresponding to first-order system poles [m/s]\n'.format(write_array(rosco_vt['WE_FOPoles_v'],'<12.3f')))
    file.write('{}              ! WE_FOPoles        - First order system poles [1/s]\n'.format(write_array(rosco_vt['WE_FOPoles'],'<12.3e')))
    file.write('\n')
    file.write('!------- YAW CONTROL ------------------------------------------------------\n')
    file.write('{:<13.5f}       ! Y_uSwitch		- Wind speed to switch between Y_ErrThresh. If zero, only the second value of Y_ErrThresh is used [m/s]\n'.format(rosco_vt['Y_uSwitch']))
    file.write('{}! Y_ErrThresh    - Yaw error threshold/deadbands. Turbine begins to yaw when it passes this. If Y_uSwitch is zero, only the second value is used. [deg].\n'.format(write_array(rosco_vt['Y_ErrThresh'],'<12.3f')))
    file.write('{:<13.5f}       ! Y_Rate			- Yaw rate [rad/s]\n'.format(rosco_vt['Y_Rate']))
    file.write('{:<13.5f}       ! Y_MErrSet		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n'.format(rosco_vt['Y_MErrSet']))
    file.write('{:<13.5f}       ! Y_IPC_IntSat		- Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]\n'.format(rosco_vt['Y_IPC_IntSat']))
    file.write('{:<13.5f}       ! Y_IPC_KP			- Yaw-by-IPC proportional controller gain Kp\n'.format(rosco_vt['Y_IPC_KP']))
    file.write('{:<13.5f}       ! Y_IPC_KI			- Yaw-by-IPC integral controller gain Ki\n'.format(rosco_vt['Y_IPC_KI']))
    file.write('\n')
    file.write('!------- TOWER CONTROL ------------------------------------------------------\n')
    file.write('{:<13.5f}       ! TRA_ExclSpeed	    - {}\n'.format(rosco_vt['TRA_ExclSpeed'], input_descriptions['TRA_ExclSpeed'] ))
    file.write('{:<13.5f}       ! TRA_ExclBand	    - {}\n'.format(rosco_vt['TRA_ExclBand'], input_descriptions['TRA_ExclBand'] ))
    file.write('{:<13.5e}       ! TRA_RateLimit	    - {}\n'.format(rosco_vt['TRA_RateLimit'], input_descriptions['TRA_RateLimit'] ))
    file.write('{:<13.5f}       ! FA_KI				- Integral gain for the fore-aft tower damper controller,  [rad*s/m]\n'.format(rosco_vt['FA_KI'] ))
    file.write('{:<13.5f}       ! FA_HPFCornerFreq	- Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]\n'.format(rosco_vt['FA_HPFCornerFreq'] ))
    file.write('{:<13.5f}       ! FA_IntSat			- Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad]\n'.format(rosco_vt['FA_IntSat'] ))
    file.write('\n')
    file.write('!------- MINIMUM PITCH SATURATION -------------------------------------------\n')
    file.write('{:<11d}         ! PS_BldPitchMin_N  - Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)\n'.format(int(rosco_vt['PS_BldPitchMin_N'])))
    file.write('{}              ! PS_WindSpeeds     - Wind speeds corresponding to minimum blade pitch angles [m/s]\n'.format(write_array(rosco_vt['PS_WindSpeeds'],'<9.4f')))
    file.write('{}              ! PS_BldPitchMin    - Minimum blade pitch angles [rad]\n'.format(write_array(rosco_vt['PS_BldPitchMin'],'<9.4f')))
    file.write('\n')
    file.write('!------- STARTUP -----------------------------------------------------------\n')
    file.write('{:<014.5f}        ! SU_StartTime            - {}\n'.format(rosco_vt['SU_StartTime'], input_descriptions['SU_StartTime']))
    file.write('{:<014.5f}        ! SU_FW_MinDuration       - {}\n'.format(rosco_vt['SU_FW_MinDuration'], input_descriptions['SU_FW_MinDuration']))
    file.write('{:<014.5f}        ! SU_RotorSpeedThresh     - {}\n'.format(rosco_vt['SU_RotorSpeedThresh'], input_descriptions['SU_RotorSpeedThresh']))
    file.write('{:<014.5f}        ! SU_RotorSpeedCornerFreq - {}\n'.format(rosco_vt['SU_RotorSpeedCornerFreq'], input_descriptions['SU_RotorSpeedCornerFreq']))
    file.write('{:<11d}           ! SU_LoadStages_N           - {}\n'.format(int(rosco_vt['SU_LoadStages_N']), input_descriptions['SU_LoadStages_N']))
    file.write('{}        ! SU_LoadStages        - {}\n'.format(write_array(rosco_vt["SU_LoadStages"]), input_descriptions["SU_LoadStages"]))
    file.write('{}        ! SU_LoadRampDuration  - {}\n'.format(write_array(rosco_vt["SU_LoadRampDuration"]), input_descriptions["SU_LoadRampDuration"]))
    file.write('{}        ! SU_LoadHoldDuration  - {}\n'.format(write_array(rosco_vt["SU_LoadHoldDuration"]), input_descriptions["SU_LoadHoldDuration"]))
    file.write('\n')
    file.write('!------- SHUTDOWN -----------------------------------------------------------\n')
    file.write('{0:<12d}        ! SD_TimeActivate        - Time to acitvate shutdown modes; no shutdown events will occur before this time. [s]\n'.format(int(rosco_vt['SD_TimeActivate'])))
    file.write('{0:<12d}        ! SD_EnablePitch         - Shutdown when collective blade pitch exceeds a threshold, [-]\n'.format(int(rosco_vt['SD_EnablePitch'])))
    file.write('{0:<12d}        ! SD_EnableYawError      - Shutdown when yaw error exceeds a threshold, [-]\n'.format(int(rosco_vt['SD_EnableYawError'])))
    file.write('{0:<12d}        ! SD_EnableGenSpeed      - Shutdown when generator speed exceeds a threshold, [-]\n'.format(int(rosco_vt['SD_EnableGenSpeed'])))
    file.write('{0:<12d}        ! SD_EnableTime          - Shutdown at a predefined time, [-]\n'.format(int(rosco_vt['SD_EnableTime'])))
    file.write('{:<014.5f}      ! SD_MaxPit              - Maximum blade pitch angle to initiate shutdown, [rad]\n'.format(rosco_vt['SD_MaxPit']))
    file.write('{:<014.5f}      ! SD_PitchCornerFreq     - Cutoff Frequency for first order low-pass filter for blade pitch angle for shutdown, [rad/s]\n'.format(rosco_vt['SD_PitchCornerFreq']))
    file.write('{:<014.5f}      ! SD_MaxYawError         - Maximum yaw error to initiate shutdown, [deg]\n'.format(rosco_vt['SD_MaxYawError']))
    file.write('{:<014.5f}      ! SD_YawErrorCornerFreq  - Cutoff Frequency for first order low-pass filter for yaw error for shutdown, [rad/s]\n'.format(rosco_vt['SD_YawErrorCornerFreq']))
    file.write('{:<014.5f}      ! SD_MaxGenSpd           - Maximum generator speed to initiate shutdown, [rad/s]\n'.format(rosco_vt['SD_MaxGenSpd']))
    file.write('{:<014.5f}      ! SD_GenSpdCornerFreq    - Cutoff Frequency for first order low-pass filter for generator speed for shutdown, [rad/s] \n'.format(rosco_vt['SD_GenSpdCornerFreq']))
    file.write('{:<014.5f}      ! SD_Time                - Shutdown time, [s]\n'.format(rosco_vt['SD_Time']))
    file.write('{:<12d}        ! SD_Method              - {}\n'.format(int(rosco_vt['SD_Method']), input_descriptions['SD_Method']))
    file.write('{:<12d}        ! SD_Stage_N             - {}\n'.format(int(rosco_vt['SD_Stage_N']), input_descriptions['SD_Stage_N']))
    file.write('{:<14}      ! SD_StageTime           - {}\n'.format(write_array(rosco_vt['SD_StageTime'],'<6.4f'), input_descriptions['SD_StageTime']))
    file.write('{:<14}      ! SD_StagePitch          - {}\n'.format(write_array(rosco_vt['SD_StagePitch'],'<6.4f'), input_descriptions['SD_StagePitch']))
    file.write('{:<14}      ! SD_MaxTorqueRate       - {}\n'.format(write_array(rosco_vt['SD_MaxTorqueRate'],'<6.4f'), input_descriptions['SD_MaxTorqueRate']))
    file.write('{:<14}      ! SD_MaxPitchRate        - {}\n'.format(write_array(rosco_vt['SD_MaxPitchRate'],'<6.4f'), input_descriptions['SD_MaxPitchRate']))
    file.write('\n')
    file.write('!------- Floating -----------------------------------------------------------\n')
    if rosco_vt['Fl_Mode'] == 2:
        floatstr = 'pitching'
    else:
        floatstr = 'velocity'
    file.write('{:<11d}         ! Fl_n              - Number of Fl_Kp gains in gain scheduling, optional with default of 1\n'.format(int(rosco_vt['Fl_n'])))
    file.write('{}        ! Fl_Kp             - Nacelle {} proportional feedback gain [s]\n'.format(write_array(rosco_vt['Fl_Kp'],'<6.4f'), floatstr))
    file.write('{}        ! Fl_U              - Wind speeds for scheduling Fl_Kp, optional if Fl_Kp is single value [m/s]\n'.format(write_array(rosco_vt['Fl_U'],'<6.4f')))
    file.write('\n')
    file.write('!------- FLAP ACTUATION -----------------------------------------------------\n')
    file.write('{:<014.5f}      ! Flp_Angle         - Initial or steady state flap angle [rad]\n'.format(rosco_vt['Flp_Angle']))
    file.write('{:<014.8e}      ! Flp_Kp            - Blade root bending moment proportional gain for flap control [s]\n'.format(rosco_vt['Flp_Kp']))
    file.write('{:<014.8e}      ! Flp_Ki            - Flap displacement integral gain for flap control [-]\n'.format(rosco_vt['Flp_Ki']))
    file.write('{:<014.5f}      ! Flp_MaxPit        - Maximum (and minimum) flap pitch angle [rad]\n'.format(rosco_vt['Flp_MaxPit']))
    file.write('\n')
    file.write('!------- Open Loop Control -----------------------------------------------------\n')
    file.write('"{}"            ! OL_Filename       - Input file with open loop timeseries (absolute path or relative to this file)\n'.format(rosco_vt['OL_Filename']))
    file.write('{:<12d}        ! OL_BP_Mode        - {}]\n'.format(int(rosco_vt['OL_BP_Mode']),input_descriptions['OL_BP_Mode']))
    file.write('{:<12f}        ! OL_BP_FiltFreq    - {}\n'.format(float(rosco_vt['OL_BP_FiltFreq']),input_descriptions['OL_BP_FiltFreq']))
    file.write('{0:<12d}        ! Ind_Breakpoint    - The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)\n'.format(int(rosco_vt['Ind_Breakpoint'])))
    file.write('{}         ! Ind_BldPitch      - The columns in OL_Filename that contains the blade pitch (1,2,3) inputs in rad [array]\n'.format(write_array(rosco_vt['Ind_BldPitch'],'<4d')))
    file.write('{0:<12d}        ! Ind_GenTq         - The column in OL_Filename that contains the generator torque in Nm\n'.format(int(rosco_vt['Ind_GenTq'])))
    file.write('{0:<12d}        ! Ind_YawRate       - The column in OL_Filename that contains the yaw rate in rad/s\n'.format(int(rosco_vt['Ind_YawRate'])))
    file.write('{:<12d}        ! Ind_Azimuth       - {}\n'.format(int(rosco_vt["Ind_Azimuth"]), input_descriptions["Ind_Azimuth"]))
    file.write('{}        ! {} - {}\n'.format(write_array(rosco_vt["RP_Gains"],'<8.4f'),"RP_Gains",input_descriptions["RP_Gains"]))
    file.write('{}        ! Ind_CableControl  - The column(s) in OL_Filename that contains the cable control inputs in m [Used with CC_Mode = 2, must be the same size as CC_Group_N]\n'.format(write_array(rosco_vt['Ind_CableControl'],'<4d')))
    file.write('{}        ! Ind_StructControl - The column(s) in OL_Filename that contains the structural control inputs [Used with StC_Mode = 2, must be the same size as StC_Group_N]\n'.format(write_array(rosco_vt['Ind_StructControl'],'<4d')))
    file.write('{:<12d}        ! Ind_R_Speed       - {}\n'.format(int(rosco_vt["Ind_R_Speed"]), input_descriptions["Ind_R_Speed"]))
    file.write('{:<12d}        ! Ind_R_Torque      - {}\n'.format(int(rosco_vt["Ind_R_Torque"]), input_descriptions["Ind_R_Torque"]))
    file.write('{:<12d}        ! Ind_R_Pitch       - {}\n'.format(int(rosco_vt["Ind_R_Pitch"]), input_descriptions["Ind_R_Pitch"]))
    file.write('\n')
    file.write('!------- Pitch Actuator Model -----------------------------------------------------\n')
    file.write('{:<014.5f}       ! PA_CornerFreq     - Pitch actuator bandwidth/cut-off frequency [rad/s]\n'.format(rosco_vt['PA_CornerFreq']))
    file.write('{:<014.5f}       ! PA_Damping        - Pitch actuator damping ratio [-, unused if PA_Mode = 1]\n'.format(rosco_vt['PA_Damping']))
    file.write('\n')
    file.write('!------- Pitch Actuator Faults -----------------------------------------------------\n')
    file.write('{}             ! PF_Offsets     - {}\n'.format(write_array(rosco_vt['PF_Offsets'],'<10.4f'), input_descriptions['PF_Offsets']))
    file.write('{}                ! PF_TimeStuck     - {}\n'.format(write_array(rosco_vt['PF_TimeStuck'],'<10.4f'), input_descriptions['PF_TimeStuck']))
    file.write('\n')
    file.write('!------- Active Wake Control -----------------------------------------------------\n')
    file.write('{0:<12d}        ! AWC_NumModes       - Number of user-defined AWC forcing modes \n'.format(int(rosco_vt['AWC_NumModes'])))
    file.write('{}        ! AWC_n              - Azimuthal mode number(s) (i.e., the number and direction of the lobes of the wake structure)\n'.format(write_array(rosco_vt['AWC_n'],'<4d')))
    file.write('{}        ! AWC_harmonic       - Harmonic(s) to apply in the AWC Inverse Coleman Transformation (only used when AWC_Mode = 2)\n'.format(write_array(rosco_vt['AWC_harmonic'],'<4d')))
    file.write('{}        ! AWC_freq           - Frequency(s) of forcing mode(s) [Hz]\n'.format(write_array(rosco_vt['AWC_freq'],'<6.4f')))
    file.write('{}        ! AWC_amp            - Pitch amplitude(s) of individual forcing mode(s) [deg]\n'.format(write_array(rosco_vt['AWC_amp'],'<6.4f')))
    file.write('{}        ! AWC_clockangle     - Initial angle(s) of forcing mode(s) [deg]\n'.format(write_array(rosco_vt['AWC_clockangle'],'<6.4f')))
    file.write('{:<014.5f}      ! AWC_phaseoffset 	 - Azimuth offset in the Coleman transformation [deg]\n'.format(float(rosco_vt['AWC_phaseoffset'])))
    file.write('{}      ! AWC_CntrGains           - KP and KI/KR gain of the active wake controller [-]\n'.format(write_array(rosco_vt['AWC_CntrGains'],'<6.4f')))
    file.write('\n')
    file.write('!------- External Controller Interface -----------------------------------------------------\n')
    file.write('"{}"            ! DLL_FileName        - Name/location of the dynamic library in the Bladed-DLL format\n'.format(rosco_vt['DLL_FileName']))
    file.write('"{}"            ! DLL_InFile          - Name of input file sent to the DLL (-)\n'.format(rosco_vt['DLL_InFile']))
    file.write('"{}"            ! DLL_ProcName        - Name of procedure in DLL to be called (-) \n'.format(rosco_vt['DLL_ProcName']))    
    file.write('\n')
    file.write('!------- ZeroMQ Interface ---------------------------------------------------------\n')
    file.write('"{}"            ! ZMQ_CommAddress     - {} \n'.format(rosco_vt['ZMQ_CommAddress'],input_descriptions['ZMQ_CommAddress']))
    file.write('{:<11f}         ! ZMQ_UpdatePeriod    - {}\n'.format(rosco_vt['ZMQ_UpdatePeriod'],input_descriptions['ZMQ_UpdatePeriod']))
    file.write('{:<11d}         ! ZMQ_ID       - {}\n'.format(int(rosco_vt['ZMQ_ID']),input_descriptions['ZMQ_ID']))
    file.write('\n')
    file.write('!------- Cable Control ---------------------------------------------------------\n')
    file.write('{:<11d}         ! CC_Group_N        - {}\n'.format(len(rosco_vt['CC_GroupIndex']), input_descriptions['CC_Group_N']))
    file.write('{:^11s}        ! CC_GroupIndex     - {}\n'.format(write_array(rosco_vt['CC_GroupIndex'],'<6d'), input_descriptions['CC_GroupIndex']))
    file.write('{:<11f}         ! CC_ActTau         - {}\n'.format(rosco_vt['CC_ActTau'], input_descriptions['CC_ActTau']  ))
    file.write('\n')
    file.write('!------- Structural Controllers ---------------------------------------------------------\n')
    file.write('{:<11d}         ! StC_Group_N       - {}\n'.format(len(rosco_vt['StC_GroupIndex']), input_descriptions['StC_Group_N']))
    file.write('{:^11s}        ! StC_GroupIndex    - {}\n'.format(write_array(rosco_vt['StC_GroupIndex'],'<6d'), input_descriptions['StC_GroupIndex']))
    
    file.close()

    # Write Open loop input
    if rosco_vt['OL_Mode'] and hasattr(controller, 'OpenLoop'):
        write_ol_control(controller)

def read_DISCON(DISCON_filename):
    '''
    Read the DISCON input file.

    Parameters:
    ----------
    DISCON_filename: string
        Name of DISCON input file to read
    
    Returns:
    --------
    DISCON_in: Dict
        Dictionary containing input parameters from DISCON_in, organized by parameter name
    '''
    
    DISCON_in = {}
    with open(DISCON_filename) as discon:
        for line in discon:
            # Skip whitespace and comment lines
            if (line[0] != '!') == (len(line.strip()) != 0):
                if (line.split()[1] != '!'):    # Array valued entries
                    array_length = line.split().index('!')
                    param = line.split()[array_length+1]
                    values = [float(x) for x in line.split()[:array_length]]
                    DISCON_in[param] = values
                else:                           # All other entries
                    param = line.split()[2]
                    value = line.split()[0]
                    # Remove printed quotations if string is in quotes
                    if (value[0] == '"') or (value[0] == "'"):
                        value = value[1:-1]
                    elif value == 'DEFAULT':
                        pass
                    else:
                        value = float(value)
                    DISCON_in[param] = value

    return DISCON_in
    

def write_rotor_performance(turbine,txt_filename='Cp_Ct_Cq.txt'):
    '''
    Write text file containing rotor performance data

    Parameters:
    ------------
        txt_filename: str, optional
                        Desired output filename to print rotor performance data. Default is Cp_Ct_Cq.txt
    '''
    print('Writing rotor performance text file: {}'.format(txt_filename))
    file = open(txt_filename,'w')
    # Headerlines
    file.write('# ----- Rotor performance tables for the {} wind turbine ----- \n'.format(turbine.TurbineName))
    file.write('# ------------ Written on {} using the ROSCO toolbox ------------ \n\n'.format(now.strftime('%b-%d-%y')))

    # Pitch angles, TSR, and wind speed
    file.write('# Pitch angle vector, {} entries - x axis (matrix columns) (deg)\n'.format(len(turbine.Cp.pitch_initial_rad)))
    for i in range(len(turbine.Cp.pitch_initial_rad)):
        file.write('{:0.4}   '.format(turbine.Cp.pitch_initial_rad[i] * rad2deg))
    file.write('\n# TSR vector, {} entries - y axis (matrix rows) (-)\n'.format(len(turbine.TSR_initial)))
    for i in range(len(turbine.TSR_initial)):
        file.write('{:0.4}    '.format(turbine.Cp.TSR_initial[i]))
    file.write('\n# Wind speed vector - z axis (m/s)\n')
    file.write('{:0.4}    '.format(turbine.v_rated))
    file.write('\n')
    
    # Cp
    file.write('\n# Power coefficient\n\n')
    for i in range(len(turbine.Cp.TSR_initial)):
        for j in range(len(turbine.Cp.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Cp_table[i,j]))
        file.write('\n')
    file.write('\n')
    
    # Ct
    file.write('\n#  Thrust coefficient\n\n')
    for i in range(len(turbine.Ct.TSR_initial)):
        for j in range(len(turbine.Ct.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Ct_table[i,j]))
        file.write('\n')
    file.write('\n')
    
    # Cq
    file.write('\n# Torque coefficient\n\n')
    for i in range(len(turbine.Cq.TSR_initial)):
        for j in range(len(turbine.Cq.pitch_initial_rad)):
            file.write('{0:.6f}   '.format(turbine.Cq_table[i,j]))
        file.write('\n')
    file.write('\n')
    file.close()


def load_from_txt(txt_filename):
    '''
    Load rotor performance data from a *.txt file. 

    Parameters:
    -----------
        txt_filename: str
                        Filename of the text containing the Cp, Ct, and Cq data. This should be in the format printed by the write_rotorperformance function
    '''
    print('Loading rotor performace data from text file:', txt_filename)

    with open(txt_filename) as pfile:
        for line in pfile:
            # Read Blade Pitch Angles (degrees)
            if 'Pitch angle' in line:
                pitch_initial = np.array([float(x) for x in pfile.readline().strip().split()])
                pitch_initial_rad = pitch_initial * deg2rad             # degrees to rad            -- should this be conditional?

            # Read Tip Speed Ratios (rad)
            if 'TSR' in line:
                TSR_initial = np.array([float(x) for x in pfile.readline().strip().split()])
            
            # Read Power Coefficients
            if 'Power' in line:
                pfile.readline()
                Cp = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Cp[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])
            
            # Read Thrust Coefficients
            if 'Thrust' in line:
                pfile.readline()
                Ct = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Ct[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

            # Read Torque Coefficients
            if 'Torque' in line:
                pfile.readline()
                Cq = np.empty((len(TSR_initial),len(pitch_initial)))
                for tsr_i in range(len(TSR_initial)):
                    Cq[tsr_i] = np.array([float(x) for x in pfile.readline().strip().split()])

        # return pitch_initial_rad TSR_initial Cp Ct Cq
        # Store necessary metrics for analysis and tuning
        # self.pitch_initial_rad = pitch_initial_rad
        # self.TSR_initial = TSR_initial
        # self.Cp_table = Cp
        # self.Ct_table = Ct 
        # self.Cq_table = Cq
        return pitch_initial_rad, TSR_initial, Cp, Ct, Cq


def DISCON_dict(turbine, controller, txt_filename=None):
    '''
    Convert the turbine and controller objects to a dictionary organized by the parameter names 
    that are defined in the DISCON.IN file.

    Parameters
    ----------
    turbine: obj
        Turbine object output from the turbine class
    controller: obj
        Controller object output from the controller class
    txt_filename: string, optional
        Name of rotor performance filename
    '''
    DISCON_dict = {}

    # Populate with available defaults
    input_schema = load_yaml(os.path.join(os.path.dirname(__file__),'inputs/toolbox_schema.yaml'))
    discon_props = input_schema['properties']['controller_params']['properties']['DISCON']['properties']
    for prop in discon_props:
        if 'default' in discon_props[prop]:
            DISCON_dict[prop] = discon_props[prop]['default']

    # ------- DEBUG -------
    DISCON_dict['LoggingLevel']	    = int(controller.LoggingLevel)
    # ------- CONTROLLER FLAGS -------
    DISCON_dict['F_LPFType']	    = int(controller.F_LPFType)
    DISCON_dict['F_NotchType']		= int(controller.F_NotchType)
    DISCON_dict['IPC_ControlMode']	= int(controller.IPC_ControlMode)
    DISCON_dict['VS_ControlMode']	= int(controller.VS_ControlMode)
    DISCON_dict['VS_ConstPower']	= int(controller.VS_ConstPower)
    DISCON_dict['VS_FBP']           = int(controller.VS_FBP)
    DISCON_dict['PC_ControlMode']   = int(controller.PC_ControlMode)
    DISCON_dict['Y_ControlMode']	= int(controller.Y_ControlMode)
    DISCON_dict['SS_Mode']          = int(controller.SS_Mode)
    DISCON_dict['PRC_Mode']         = int(controller.PRC_Mode)
    DISCON_dict['WE_Mode']          = int(controller.WE_Mode)
    DISCON_dict['PS_Mode']          = int(controller.PS_Mode > 0)
    DISCON_dict['SU_Mode']          = int(controller.SU_Mode)
    DISCON_dict['SD_Mode']          = int(controller.SD_Mode)
    DISCON_dict['Fl_Mode']          = int(controller.Fl_Mode)
    DISCON_dict['TD_Mode']          = int(controller.TD_Mode)
    DISCON_dict['TRA_Mode']         = int(controller.TRA_Mode)
    DISCON_dict['Flp_Mode']         = int(controller.Flp_Mode)
    DISCON_dict['OL_Mode']          = int(controller.OL_Mode)
    DISCON_dict['PF_Mode']          = int(controller.PF_Mode)
    DISCON_dict['PA_Mode']          = int(controller.PA_Mode)
    DISCON_dict['AWC_Mode']         = int(controller.AWC_Mode)
    DISCON_dict['Ext_Mode']         = int(controller.Ext_Mode)
    DISCON_dict['ZMQ_Mode']         = int(controller.ZMQ_Mode)
    DISCON_dict['CC_Mode']          = int(controller.CC_Mode)
    DISCON_dict['StC_Mode']          = int(controller.StC_Mode)
    # ------- FILTERS -------
    DISCON_dict['F_LPFCornerFreq']	    = turbine.bld_edgewise_freq * 1/4
    DISCON_dict['F_LPFDamping']		    = controller.F_LPFDamping
    DISCON_dict['F_NumNotchFilts']      = len(controller.f_notch_freqs)
    DISCON_dict['F_NotchFreqs']         = controller.f_notch_freqs if controller.f_notch_freqs else [0.0]
    DISCON_dict['F_NotchBetaNum']       = controller.f_notch_beta_nums if controller.f_notch_beta_nums else [0.0]
    DISCON_dict['F_NotchBetaDen']       = controller.f_notch_beta_dens if controller.f_notch_beta_dens else [0.0]
    DISCON_dict['F_GenSpdNotch_N']      = len(controller.f_notch_gen_inds)
    DISCON_dict['F_GenSpdNotch_Ind']    = controller.f_notch_gen_inds if controller.f_notch_gen_inds else [0]
    DISCON_dict['F_TwrTopNotch_N']      = len(controller.f_notch_twr_inds)
    DISCON_dict['F_TwrTopNotch_Ind']    = controller.f_notch_twr_inds if controller.f_notch_twr_inds else [0]
    DISCON_dict['F_WECornerFreq'] = controller.f_we_cornerfreq
    DISCON_dict['F_SSCornerFreq'] = controller.f_ss_cornerfreq
    DISCON_dict['F_FlHighPassFreq'] = controller.f_fl_highpassfreq
    DISCON_dict['F_FlCornerFreq'] = [controller.ptfm_freq, 1.0]
    DISCON_dict['F_FlpCornerFreq'] = [turbine.bld_flapwise_freq*3, 1.0]
    DISCON_dict['F_WECornerFreq']       = controller.f_we_cornerfreq
    DISCON_dict['F_SSCornerFreq']       = controller.f_ss_cornerfreq
    DISCON_dict['F_YawErr']             = controller.f_yawerr
    DISCON_dict['F_FlHighPassFreq']     = controller.f_fl_highpassfreq
    DISCON_dict['F_FlCornerFreq']       = [controller.ptfm_freq, 1.0]
    DISCON_dict['F_VSRefSpdCornerFreq'] = controller.f_vs_refspd_cornerfreq
    # ------- BLADE PITCH CONTROL -------
    DISCON_dict['PC_GS_n']			= len(controller.pitch_op_pc)
    DISCON_dict['PC_GS_angles']	    = controller.pitch_op_pc
    DISCON_dict['PC_GS_KP']		    = controller.pc_gain_schedule.Kp
    DISCON_dict['PC_GS_KI']		    = controller.pc_gain_schedule.Ki
    DISCON_dict['PC_GS_KD']			= [0.0 for i in range(len(controller.pc_gain_schedule.Ki))]
    DISCON_dict['PC_GS_TF']			= [0.0 for i in range(len(controller.pc_gain_schedule.Ki))]

    #	
    DISCON_dict['PC_MaxPit']		= controller.max_pitch
    DISCON_dict['PC_MinPit']		= controller.min_pitch
    DISCON_dict['PC_MaxRat']		= turbine.max_pitch_rate
    DISCON_dict['PC_MinRat']		= turbine.min_pitch_rate
    DISCON_dict['PC_RefSpd']		= turbine.rated_rotor_speed*turbine.Ng
    DISCON_dict['PC_FinePit']		= controller.min_pitch
    DISCON_dict['PC_Switch']		= 1 * deg2rad
    # ------- INDIVIDUAL PITCH CONTROL -------
    DISCON_dict['IPC_Vramp']        = controller.IPC_Vramp
    DISCON_dict['IPC_IntSat']		= 0.2618
    DISCON_dict['IPC_SatMode']		= 2
    DISCON_dict['IPC_KP']           = [controller.Kp_ipc1p, controller.Kp_ipc2p]
    DISCON_dict['IPC_KI']           = [controller.Ki_ipc1p, controller.Ki_ipc2p]
    DISCON_dict['IPC_aziOffset']	= [0.0, 0.0]
    DISCON_dict['IPC_CornerFreqAct'] = 0.0
    # ------- VS TORQUE CONTROL -------
    DISCON_dict['VS_GenEff']		= turbine.GenEff
    DISCON_dict['VS_ArSatTq']		= turbine.rated_torque
    DISCON_dict['VS_MaxRat']		= turbine.max_torque_rate
    DISCON_dict['VS_MaxTq']			= turbine.max_torque
    DISCON_dict['VS_MinTq']			= 0.0
    DISCON_dict['VS_MinOMSpd']		= controller.vs_minspd  * turbine.Ng
    DISCON_dict['VS_Rgn2K']			= controller.vs_rgn2K
    DISCON_dict['VS_RtPwr']			= turbine.rated_power
    DISCON_dict['VS_RtTq']			= turbine.rated_torque
    DISCON_dict['VS_RefSpd']		= controller.vs_refspd * (1 - controller.controller_params['ss_separation'])
    DISCON_dict['VS_n']				= 1
    DISCON_dict['VS_KP']			= controller.vs_gain_schedule.Kp[-1]
    DISCON_dict['VS_KI']			= controller.vs_gain_schedule.Ki[-1]
    DISCON_dict['VS_TSRopt']		= turbine.TSR_operational
    # ------- FIXED BLADE PITCH TORQUE CONTROL -------
    DISCON_dict['VS_FBP_n']         = len(controller.v)
    DISCON_dict['VS_FBP_U']         = controller.v
    DISCON_dict['VS_FBP_Omega']     = controller.omega_gen_op
    DISCON_dict['VS_FBP_Tau']       = controller.tau_op
    # ------- SETPOINT SMOOTHER -------
    DISCON_dict['SS_VSGain']         = controller.ss_vsgain
    DISCON_dict['SS_PCGain']         = controller.ss_pcgain
    # -------- POWER REFERENCE TRACKING ------
    DISCON_dict['PRC_n'] = 2
    DISCON_dict['PRC_WindSpeeds'] = [3,25]
    DISCON_dict['PRC_GenSpeeds'] = [rpm2RadSec * 7.56] * 2
    
    # ------- WIND SPEED ESTIMATOR -------
    DISCON_dict['WE_BladeRadius']	= turbine.rotor_radius
    DISCON_dict['WE_CP_n']			= 1
    DISCON_dict['WE_CP']            = 0
    DISCON_dict['WE_Gamma']			= 0.0
    DISCON_dict['WE_GearboxRatio']	= turbine.Ng
    DISCON_dict['WE_Jtot']			= turbine.J
    DISCON_dict['WE_RhoAir']		= turbine.rho
    DISCON_dict['PerfFileName']     = txt_filename
    DISCON_dict['PerfTableSize']    = [len(turbine.Cp.pitch_initial_rad),len(turbine.Cp.TSR_initial)]
    DISCON_dict['WE_FOPoles_N']     = len(controller.A)
    DISCON_dict['WE_FOPoles_v']     = controller.v
    DISCON_dict['WE_FOPoles']       = controller.A
    # ------- YAW CONTROL -------
    DISCON_dict['Y_uSwitch']    = 0.0
    DISCON_dict['Y_ErrThresh']  = [4.0, 8.0] # NJA: hard coding these params right now b/c we can just use the DISCON pass-through if needed
    DISCON_dict['Y_Rate']       = 0.0087 #0.5 deg/s
    DISCON_dict['Y_MErrSet']    = 0.0
    DISCON_dict['Y_IPC_IntSat'] = 0.0
    DISCON_dict['Y_IPC_KP'] = 0.0
    DISCON_dict['Y_IPC_KI'] = 0.0
    # ------- TOWER FORE-AFT DAMPING -------
    DISCON_dict['FA_KI']            = 0.0
    DISCON_dict['FA_HPFCornerFreq'] = 0.0
    DISCON_dict['FA_IntSat']		= 0.0
    # ------- MINIMUM PITCH SATURATION -------
    DISCON_dict['PS_BldPitchMin_N'] = len(controller.ps_min_bld_pitch)
    DISCON_dict['PS_WindSpeeds']    = controller.v
    DISCON_dict['PS_BldPitchMin']   = controller.ps_min_bld_pitch
    # ------- SHUTDOWN -------
    DISCON_dict['SD_EnablePitch']       = 0
    DISCON_dict['SD_EnableYawError']    = 0
    DISCON_dict['SD_EnableGenSpeed']    = 0
    DISCON_dict['SD_EnableTime']        = 0
    DISCON_dict['SD_MaxPit']            = controller.pitch_op[-1]
    DISCON_dict['SD_PitchCornerFreq']   = controller.f_sd_pitchcornerfreq
    DISCON_dict['SD_MaxYawError']       = 30
    DISCON_dict['SD_YawErrorCornerFreq']= controller.f_sd_yawerrorcornerfreq
    DISCON_dict['SD_MaxGenSpd']         = np.max(controller.omega_gen_op) * 1.2  # 20% above rated
    DISCON_dict['SD_GenSpdCornerFreq']  = controller.f_sd_genspdcornerfreq
    DISCON_dict['SD_Time']              = 9999
    DISCON_dict['SD_Method']            = 1
    DISCON_dict['SD_Stage_N']           = 1
    DISCON_dict['SD_StageTime']        = [1000] # should be be enough time for the single stage shutdown
    DISCON_dict['SD_StagePitch']       = [np.radians(90.0)] # should be be enough time for the single stage shutdown
    DISCON_dict['SD_MaxTorqueRate']     = [0.05 * turbine.max_torque_rate]
    DISCON_dict['SD_MaxPitchRate']      = [0.125 * turbine.max_pitch_rate]
    # ------- Floating -------
    DISCON_dict['Fl_n']             = len(controller.Kp_float)
    DISCON_dict['Fl_Kp']            = controller.Kp_float
    DISCON_dict['Fl_U']             = controller.U_Fl
    # ------- FLAP ACTUATION -------
    DISCON_dict['Flp_Angle']        = controller.flp_angle
    DISCON_dict['Flp_Kp']           = controller.Kp_flap[-1]
    DISCON_dict['Flp_Ki']           = controller.Ki_flap[-1]
    DISCON_dict['Flp_MaxPit']       = controller.flp_maxpit
    # ------- Open Loop Control -------
    DISCON_dict['OL_Filename']     = controller.OL_Filename
    DISCON_dict['OL_BP_Mode']      = controller.OL_BP_Mode
    DISCON_dict['Ind_Breakpoint']  = controller.Ind_Breakpoint
    DISCON_dict['Ind_BldPitch']    = controller.Ind_BldPitch
    DISCON_dict['Ind_GenTq']       = controller.Ind_GenTq
    DISCON_dict['Ind_YawRate']     = controller.Ind_YawRate
    DISCON_dict['Ind_CableControl']     = controller.Ind_CableControl
    DISCON_dict['Ind_StructControl']    = controller.Ind_StructControl
    DISCON_dict['Ind_Azimuth']      = controller.Ind_Azimuth
    DISCON_dict['Ind_R_Speed']      = controller.Ind_R_Speed
    DISCON_dict['Ind_R_Torque']     = controller.Ind_R_Torque
    DISCON_dict['Ind_R_Pitch']      = controller.Ind_R_Pitch

    # ------- Pitch Actuator -------
    DISCON_dict['PA_Mode']         = controller.PA_Mode
    DISCON_dict['PA_CornerFreq']   = controller.PA_CornerFreq
    DISCON_dict['PA_Damping']      = controller.PA_Damping
    # ------- Pitch Actuator Fault -------
    DISCON_dict['PF_Offsets']       = [0.,0.,0.]
    
    # Add pass through here
    for param, value in controller.controller_params['DISCON'].items():
        DISCON_dict[param] = value

    # Make all lists, not numpy
    DISCON_dict = simple_types(DISCON_dict)

    return DISCON_dict


def run_openfast(fast_dir, fastcall='openfast', fastfile=None, chdir=True, restart=False):
    '''
    Runs a openfast openfast simulation.
    
    NOTE: Enabling chdir can help reduce file path errors in the DISCON.IN file.

    Parameters:
    ------------
        fast_dir: string
                Name of OpenFAST directory containing input files.
        fast_file: string
                Name of OpenFAST directory containing input files.
        fastcall: string, optional
                Line used to call openfast when executing from the terminal.
        fastfile: string, optional
                Filename for *.fst input file. Function will find *.fst if not provided.
        chdir: bool, optional
                Change directory to openfast model directory before running.
    '''

    # Define OpenFAST input filename
    if not fastfile:
        for file in os.listdir(fast_dir):
            if file.endswith('.fst'):
                fastfile = file
    print('Using {} to run OpenFAST simulation'.format(fastfile))

    if chdir: # Change cwd before calling OpenFAST -- note: This is an artifact of needing to call OpenFAST from the same directory as DISCON.IN
        cwd = fast_dir
    else:
        cwd = None

    print('Running OpenFAST simulation for {} through the ROSCO toolbox...'.format(fastfile))
        # os.system('{} {}'.format(fastcall, os.path.join(fastfile)))
    if restart:
        subprocess.run([fastcall,'-restart', os.path.join(fastfile)], check=True, cwd=cwd)
    else:
        subprocess.run([fastcall, os.path.join(fastfile)], check=True, cwd=cwd)
    print('OpenFAST simulation complete.')


def list_check(x, return_bool=True):
    '''
    Check if the input is list-like or not

    Parameters:
    -----------
        x: int, float, list, or np.ndarray
            input to check
        return_bool: bool
            if true, returns True or False 

    '''
    if isinstance(x, (int, float)):
        y = x
        is_list = False
    elif isinstance(x, list):
        if len(x) == 1:
            y = x[0]
            is_list = False
        else:
            y = x
            is_list = True
    elif isinstance(x, np.ndarray):
        if x.size == 1:
            is_list = False
            y = float(x[0])
        else:
            is_list = True
            y = x
    else:
        raise AttributeError('Cannot run list_check for variable of type: {}'.format(type(x)))

    if return_bool:
        return is_list
    else:
        return y

def write_array(array,format='<.4f',line_width=12):
    if isinstance(array, str):
        array = ast.literal_eval(array)
    elif not hasattr(array,'__len__'):  #not an array
        array = [array]

    # force int if not
    if 'd' in format and type(array[0]) != int:
        array = [int(a) for a in array]

    return ''.join(['{:{}} '.format(item,format) for item in array]).ljust(line_width)
