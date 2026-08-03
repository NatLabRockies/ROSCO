import os
import numpy as np

from rosco.toolbox.ofTools.case_gen.HH_WindFile import HH_StepFile, HH_WindFile

# ROSCO 
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import utilities as ROSCO_utilities

from rosco.toolbox.inputs.validation import load_rosco_yaml

# Globals
this_dir        = os.path.dirname(os.path.abspath(__file__))
tune_case_dir   = os.path.realpath(os.path.join(this_dir,'../../../../Examples/Tune_Cases'))

def find_max_group(case_inputs):
    max_group = 0
    for ci in case_inputs:
        max_group = np.max([case_inputs[ci]['group'],max_group])
    return max_group


def set_channels():
    channels = {}
    for var in ["TipDxc1", "TipDyc1", "TipDzc1", "TipDxb1", "TipDyb1", "TipDxc2", "TipDyc2", \
         "TipDzc2", "TipDxb2", "TipDyb2", "TipDxc3", "TipDyc3", "TipDzc3", "TipDxb3", "TipDyb3", \
             "RootMxc1", "RootMyc1", "RootMzc1", "RootMxb1", "RootMyb1", "RootMxc2", "RootMyc2", \
                 "RootMzc2", "RootMxb2", "RootMyb2", "RootMxc3", "RootMyc3", "RootMzc3", "RootMxb3",\
                      "RootMyb3", "TwrBsMxt", "TwrBsMyt", "TwrBsMzt", "GenPwr", "GenTq", "RotThrust",\
                           "RtAeroCp", "RtAeroCt", "RotSpeed", "BldPitch1", "TTDspSS", "TTDspFA", \
                               "NcIMUTAxs", "NcIMUTAys", "NcIMUTAzs", "NcIMURAxs", "NcIMURAys", "NcIMURAzs", \
                                "NacYaw", "Wind1VelX", "Wind1VelY", "Wind1VelZ", "LSSTipMxa","LSSTipMya",\
                                   "LSSTipMza","LSSTipMxs","LSSTipMys","LSSTipMzs","LSShftFys","LSShftFzs", \
                                       "TipRDxr", "TipRDyr", "TipRDzr","RtVAvgxh","RtAeroFxh", "RtTSR"]:
        channels[var] = True
    return channels


def load_tuning_yaml(tuning_yaml):
    # Load yaml file, this could be an object...
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_case_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
    turbine.load_from_fast(path_params['FAST_InputFile'], \
        os.path.join(tune_case_dir,path_params['FAST_directory']), \
        rot_source='txt',\
        txt_filename=cp_filename)

    return turbine, controller, cp_filename


##############################################################################################
#
#   Wind input cases
#
##############################################################################################

def base_op_case():

    case_inputs = {}

    case_inputs[("Fst","OutFileFmt")]        = {'vals':[3], 'group':0}
    
    # DOFs
    case_inputs[("ElastoDyn","GenDOF")]      = {'vals':['True'], 'group':0}
    if False:
        case_inputs[("ElastoDyn","YawDOF")]      = {'vals':['True'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","FlapDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","EdgeDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","DrTrDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","GenDOF")]      = {'vals':['True'], 'group':0} 
        case_inputs[("ElastoDyn","TwFADOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwFADOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF1")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","TwSSDOF2")]    = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSgDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmHvDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmPDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmSwDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmRDOF")]     = {'vals':['False'], 'group':0}
        case_inputs[("ElastoDyn","PtfmYDOF")]     = {'vals':['False'], 'group':0}
    

    # Stop Generator from Turning Off
    case_inputs[('ServoDyn', 'GenTiStr')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'GenTiStp')] = {'vals': ['True'], 'group': 0}
    case_inputs[('ServoDyn', 'SpdGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'TimGenOn')] = {'vals': [0.], 'group': 0}
    case_inputs[('ServoDyn', 'GenModel')] = {'vals': [1], 'group': 0}

    case_inputs[('ServoDyn', 'VSContrl')] = {'vals': [5], 'group': 0}
    case_inputs[('ServoDyn', 'PCMode')] = {'vals': [5], 'group': 0}
    case_inputs[('ServoDyn', 'HSSBrMode')] = {'vals': [5], 'group': 0}
    case_inputs[('ServoDyn', 'YCMode')] = {'vals': [5], 'group': 0}    

    # AeroDyn
    if False:
        case_inputs[("AeroDyn", "WakeMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn", "AFAeroMod")] = {'vals': [2], 'group': 0}
        case_inputs[("AeroDyn", "TwrPotent")] = {'vals': [0], 'group': 0}
        case_inputs[("AeroDyn", "TwrShadow")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn", "TwrAero")] = {'vals': ['False'], 'group': 0}
        case_inputs[("AeroDyn", "SkewMod")] = {'vals': [1], 'group': 0}
        case_inputs[("AeroDyn", "TipLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn", "HubLoss")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn", "TanInd")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn", "AIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn", "TIDrag")] = {'vals': ['True'], 'group': 0}
        case_inputs[("AeroDyn", "IndToler")] = {'vals': [1.e-5], 'group': 0}
        case_inputs[("AeroDyn", "MaxIter")] = {'vals': [5000], 'group': 0}
        case_inputs[("AeroDyn", "UseBlCm")] = {'vals': ['True'], 'group': 0}

    return case_inputs

def power_curve(**wind_case_opts):
    # Constant wind speed, multiple wind speeds, define below

    # Runtime
    T_max   = wind_case_opts.get('TMax',400.)

    if 'U' in wind_case_opts:
        U = wind_case_opts['U']
        if type(U) != list:
            U = [U]
        elif type(U) == np.ndarray:
            U = U.tolist()
    else: # default
        # Run conditions
        U = np.arange(4,14.5,.5).tolist()
        U = np.linspace(3,25,num=16).tolist()


    case_inputs = base_op_case()
    # simulation settings
    case_inputs[("Fst","TMax")] = {'vals':[T_max], 'group':0}

    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[1], 'group':0}
    case_inputs[("InflowWind","HWindSpeed")] = {'vals':U, 'group':1}

    return case_inputs

def simp_step(**wind_case_opts):
    # Set up cases for FIW-JIP project
    # 3.x in controller tuning register

    TMax    = wind_case_opts.get('TMax',300.)
    T_step  = wind_case_opts.get('TStep',150.)
    U_start = wind_case_opts.get('U_start',[16.])
    U_end   = wind_case_opts.get('U_end',[17.])

    # Wind directory, default is run_dir
    wind_case_opts['wind_dir'] = wind_case_opts.get('wind_dir',wind_case_opts['run_dir'])

    # Step Wind Setup

    # Make Default step wind object
    hh_step = HH_StepFile()
    hh_step.TMax = TMax
    hh_step.t_step = T_step
    hh_step.wind_directory = wind_case_opts['wind_dir']

    # Run conditions
    step_wind_files = []

    for u_s,u_e in zip(U_start,U_end):
        # Make Step
        hh_step.u_start = u_s
        hh_step.u_end   = u_e
        hh_step.update()
        hh_step.write()

        step_wind_files.append(hh_step.filename)

    case_inputs = base_op_case()
    # simulation settings
    case_inputs[("Fst","TMax")] = {'vals':[TMax], 'group':0}
    # case_inputs[("Fst","DT")]        = {'vals':[1/80], 'group':0}
    
    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","FileName_Uni")] = {'vals':step_wind_files, 'group':1}

    return case_inputs


def steps(**wind_case_opts):
    # Muliple steps in same simulation at time, wind breakpoints, this function adds zero-order hold, 100 seconds to end

    if 'tt' in wind_case_opts and 'U' in wind_case_opts:
        tt = wind_case_opts['tt']
        U = wind_case_opts['U']
    else:
        raise Exception('You must define tt and U in **wind_case_opts dict to use steps() fcn')

    if 'dt' in wind_case_opts:
        dt = wind_case_opts['dt']
    else:
        dt = 0.05

    if 'U_0' in wind_case_opts:
        U_0 = wind_case_opts['U_0']
    else:
        U_0 = U[0]

    if 'TMax' in wind_case_opts:
        TMax = wind_case_opts['TMax']
    else:
        TMax = tt[-1] + 100

    if len(tt) != len(U):
        raise Exception('steps: len(tt) and len(U) must be the same')


    # Make Default step wind object
    hh_wind = HH_WindFile()
    hh_wind.TMax = TMax
    hh_wind.filename = os.path.join(wind_case_opts['run_dir'],'steps.hh')

    # Step Wind Setup
    hh_wind.time = [0]
    hh_wind.wind_speed = [U_0]
    for t, u in zip(tt,U):
        hh_wind.time.append(t-dt)
        hh_wind.wind_speed.append(hh_wind.wind_speed[-1])

        hh_wind.time.append(t)
        hh_wind.wind_speed.append(u)

    hh_wind.wind_dir = [0] * len(hh_wind.time)
    hh_wind.vert_speed = [0] * len(hh_wind.time)
    hh_wind.horiz_shear = [0] * len(hh_wind.time)
    hh_wind.vert_shear = [0] * len(hh_wind.time)
    hh_wind.linv_shear = [0] * len(hh_wind.time)
    hh_wind.gust_speed = [0] * len(hh_wind.time)
    
    if False:
        hh_wind.plot()

    hh_wind.write()
    case_inputs = base_op_case()

    case_inputs[("Fst","TMax")] = {'vals':[TMax], 'group':0}


    # wind inflow
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","FileName_Uni")] = {'vals':[hh_wind.filename], 'group':0}

    return case_inputs

def turb_bts(**wind_case_opts):
    '''
     Turbulent wind input from bts file
     Expected inputs:
        TMax            TODO: someday make all TMaxs TMax
        wind_inputs (list of string wind inputs filenames)
    '''

    if 'TMax' in wind_case_opts:
        TMax = wind_case_opts['TMax']
    else:
        TMax = 720

    if 'wind_filenames' not in wind_case_opts:
        raise Exception('Define wind_filenames when using turb_bts case generator')

    # wind inflow
    case_inputs = base_op_case()
    case_inputs[("Fst","TMax")] = {'vals':[TMax], 'group':0}
    case_inputs[("InflowWind","WindType")] = {'vals':[3], 'group':0}
    case_inputs[("InflowWind","FileName_BTS")] = {'vals':wind_case_opts['wind_filenames'], 'group':1}

    return case_inputs

def user_hh(**wind_case_opts):
    '''
     Uniform, hub-height wind file
     Expected inputs:
        TMax            (float or array) of simulation TMax
        wind_filenames (list of string wind inputs filenames)
    '''

    # Default is 720 for all cases
    TMax = wind_case_opts.get('TMax', 720)

    if 'wind_filenames' not in wind_case_opts:
        raise Exception('Define wind_filenames when using user_hh case generator')

    # Make into array
    if not hasattr(TMax,'__len__'):
        TMax = len(wind_case_opts['wind_filenames']) * [TMax]




    # wind inflow
    case_inputs = base_op_case()
    case_inputs[("Fst","TMax")] = {'vals':TMax, 'group':1}
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","FileName_Uni")] = {'vals':wind_case_opts['wind_filenames'], 'group':1}

    return case_inputs

def ramp(**wind_case_opts):
    U_start     = wind_case_opts.get('U_start',8.)
    U_end       = wind_case_opts.get('U_end',15.)
    t_start     = wind_case_opts.get('t_start',100.)
    t_end       = wind_case_opts.get('t_end',400.)
    vert_shear  = wind_case_opts.get('vert_shear',.2) 
    both_dir    = wind_case_opts.get('both_dir',False)  # ramp up and down

    # Make Default step wind object
    hh_wind = HH_WindFile()
    hh_wind.TMax = t_end
    hh_wind.filename = os.path.join(wind_case_opts['run_dir'],'ramp.hh')

    # Step Wind Setup
    if both_dir:
        hh_wind.time = [0, t_start, (t_start + t_end) / 2, t_end]
        hh_wind.wind_speed = [U_start, U_start, U_end, U_start]
    else:
        hh_wind.time = [0, t_start, t_end]
        hh_wind.wind_speed = [U_start, U_start, U_end]

    hh_wind.vert_shear = [vert_shear] * len(hh_wind.time)

    hh_wind.resample()
    hh_wind.write()

    case_inputs = base_op_case()
    case_inputs[("Fst","TMax")] = {'vals':[t_end], 'group':0}
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","FileName_Uni")] = {'vals':[hh_wind.filename], 'group':0}

    return case_inputs


def direction_change(**wind_case_opts):
    U           = wind_case_opts.get('U',[8.])
    dir_start   = wind_case_opts.get('dir_start',[0.])
    dir_end     = wind_case_opts.get('dir_end',[180.])
    t_start     = wind_case_opts.get('t_start',[100.])
    t_end       = wind_case_opts.get('t_end',[400.])
    TMax        = wind_case_opts.get('TMax',t_end)

    # Check if all inputs are lists
    if not isinstance(U, list):
        U = [U]
    if not isinstance(dir_start, list):
        dir_start = [dir_start]
    if not isinstance(dir_end, list):
        dir_end = [dir_end]
    if not isinstance(t_start, list):
        t_start = [t_start]
    if not isinstance(t_end, list):
        t_end = [t_end]
    if not isinstance(TMax, list):
        TMax = [TMax]

    # Check sizes
    if not (len(U) == len(dir_start) == len(dir_end) == len(t_start) == len(t_end) == len(TMax)) :
        raise ValueError('All input arrays must have the same length')
    
    hh_filenames = []

    # Make Default step wind object
    for i in range(len(U)):

        hh_wind = HH_WindFile()
        hh_wind.TMax = TMax[i]
        hh_wind.filename = os.path.join(wind_case_opts['run_dir'],f'direction_change_{i}.hh')

        hh_wind.time    = [0, t_start[i], t_end[i], TMax[i]]
        hh_wind.wind_speed  = [U[i], U[i], U[i], U[i]]
        hh_wind.wind_dir    = [dir_start[i], dir_start[i], dir_end[i], dir_end[i]]

        hh_wind.resample()
        hh_wind.write()
        hh_filenames.append(hh_wind.filename)

    case_inputs = base_op_case()
    case_inputs[("Fst","TMax")] = {'vals':TMax, 'group':1}
    case_inputs[("InflowWind","WindType")] = {'vals':[2], 'group':0}
    case_inputs[("InflowWind","FileName_Uni")] = {'vals':hh_filenames, 'group':1}

    return case_inputs



    
##############################################################################################
#
#   Control sweep cases
#
##############################################################################################

def sweep_rated_torque(start_group, **control_sweep_opts):

    # Sweep multiplier of original rated torque
    multipliers = np.linspace(1,1.5,5)


    # Load yaml file 
    inps = load_rosco_yaml(tuning_yaml)
    path_params         = inps['path_params']
    controller_params   = inps['controller_params']

    var_trees = []
    for m in multipliers:
        
        turbine_params      = inps['turbine_params'].copy()
        turbine_params['rated_power'] *= m

        # Instantiate turbine, controller, and file processing classes
        turbine         = ROSCO_turbine.Turbine(turbine_params)
        controller      = ROSCO_controller.Controller(controller_params)

        # Load turbine data from OpenFAST and rotor performance text file
        cp_filename = os.path.join(tune_case_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
        turbine.load_from_fast(path_params['FAST_InputFile'], \
            os.path.join(tune_case_dir,path_params['FAST_directory']), \
            rot_source='txt',\
            txt_filename=cp_filename)

        controller.tune_controller(turbine)
        discon_vt = ROSCO_utilities.DISCON_dict(turbine, controller, txt_filename=cp_filename)            
        var_trees.append(discon_vt.copy())

    # Translate array of dicts into dict of arrays
    discon_array = {}
    for var in var_trees[0]:
        discon_array[var] = []

    for vt in var_trees:
        for var in vt:
            discon_array[var].append(vt[var])
            
    
    case_inputs_control = {}
    for discon_input in discon_array:
        case_inputs_control[('DISCON_in',discon_input)] = {'vals': discon_array[discon_input], 'group': start_group}

    return case_inputs_control

def sweep_pitch_act(start_group, **control_sweep_opts):
    if 'act_bw' in control_sweep_opts:
        act_bw = control_sweep_opts['act_bw']
    else:
        raise Exception('Define act_bw to sweep or program something else.')

    case_inputs_control = {}
    case_inputs_control[('DISCON_in','PA_CornerFreq')] = {'vals': act_bw.tolist(), 'group': start_group}

    return case_inputs_control

def sweep_ipc_gains(start_group, **control_sweep_opts):
    case_inputs_control = {}

    kis = np.linspace(0,1,8).tolist()
    # kis = [0.,0.6,1.2,1.8,2.4,3.]
    KIs = [[ki * 12e-9,0.] for ki in kis]
    case_inputs_control[('DISCON_in','IPC_ControlMode')] = {'vals': [1], 'group': 0}
    # case_inputs_control[('DISCON_in','IPC_KI')] = {'vals': [[0.,0.],[1e-8,0.]], 'group': start_group}
    case_inputs_control[('DISCON_in','IPC_KI')] = {'vals': KIs, 'group': start_group}
    case_inputs_control[('DISCON_in','IPC_aziOffset')] = {'vals': [[0.0,0]], 'group': 0}
    case_inputs_control[('DISCON_in','IPC_IntSat')] = {'vals': [0.2618], 'group': 0}

    # [-0.5236,-0.43633,-0.34907,-0.2618,-0.17453,-0.087266           0    0.087266     0.17453      0.2618     0.34907     0.43633      0.5236     0.61087     0.69813      0.7854'

    return case_inputs_control

def sweep_fad_gains(start_group, **control_sweep_opts):
    case_inputs_control = {}
    g = np.array([0.,0.5,1.,1.5,2.0,2.5,3.0,3.5,4.0,5.0])
    case_inputs_control[('DISCON_in','TD_Mode')] = {'vals': [1], 'group': start_group}
    case_inputs_control[('DISCON_in','FA_KI')] = {'vals': (g*0.0175).tolist(), 'group': start_group+1}
    case_inputs_control[('DISCON_in','FA_HPFCornerFreq')] = {'vals': [0.1], 'group': start_group}
    case_inputs_control[('DISCON_in','FA_IntSat')] = {'vals': [0.2618], 'group': start_group}

    # [-0.5236,-0.43633,-0.34907,-0.2618,-0.17453,-0.087266           0    0.087266     0.17453      0.2618     0.34907     0.43633      0.5236     0.61087     0.69813      0.7854'

    return case_inputs_control

def sweep_max_torque(start_group, **control_sweep_opts):
    case_inputs_control = {}
    max_torque = np.array([1.,1.05,1.1,1.15,1.2]) * 18651.96057000 
    case_inputs_control[('DISCON_in','VS_MaxTq')] = {'vals': max_torque, 'group': start_group}

    return case_inputs_control

def sweep_ps_percent(start_group, **control_sweep_opts):
        case_inputs_control = {}
        
        # Set sweep limits here
        ps_perc = np.linspace(.7,1,num=8,endpoint=True).tolist()
        
        # load default params          
        control_param_yaml  = control_sweep_opts['tuning_yaml']
        inps                = load_rosco_yaml(control_param_yaml)
        path_params         = inps['path_params']
        turbine_params      = inps['turbine_params']
        controller_params   = inps['controller_params']

        # make default controller, turbine objects for rosco.toolbox
        turbine             = ROSCO_turbine.Turbine(turbine_params)
        turbine.load_from_fast( path_params['FAST_InputFile'],path_params['FAST_directory'])

        controller          = ROSCO_controller.Controller(controller_params)

        # tune default controller
        controller.tune_controller(turbine)

        # Loop through and make min pitch tables
        ps_ws = []
        ps_mp = []
        m_ps  = []  # flattened (omega,zeta) pairs
        for p in ps_perc:
            controller.ps_percent = p
            controller.tune_controller(turbine)
            m_ps.append(controller.ps_min_bld_pitch)
            ps_ws.append(controller.v)

        # add control gains to case_list
        # case_inputs_control[('meta','ps_perc')]          = {'vals': ps_perc, 'group': start_group}
        case_inputs_control[('DISCON_in', 'PS_BldPitchMin')] = {'vals': m_ps, 'group': start_group}
        case_inputs_control[('DISCON_in', 'PS_WindSpeeds')] = {'vals': ps_ws, 'group': start_group}

        return case_inputs_control


def test_pitch_offset(start_group, **control_sweep_opts):
    case_inputs_control = {}
    case_inputs_control[('DISCON_in','PF_Mode')] = {'vals': [1], 'group': start_group}
    case_inputs_control[('DISCON_in','PF_Offsets')] = {'vals': [[0,float(np.radians(2)),0]], 'group': start_group}
    return case_inputs_control

def sweep_yaml_input(start_group, **control_sweep_opts):
    '''
    Sweep any single tuning yaml input
    
    control_sweep_opts:
        control_param: name of parameter
        param_values: values of parameter (1D array)

    '''

    required_inputs = [('control_param','discon_param'), 'param_values']
    check_inputs(control_sweep_opts,required_inputs)

    # load default params          
    control_param_yaml  = control_sweep_opts['tuning_yaml']
    inps                = load_rosco_yaml(control_param_yaml)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']

    # make default controller, turbine objects for rosco.toolbox
    turbine             = ROSCO_turbine.Turbine(turbine_params)
    yaml_dir            = os.path.dirname(control_param_yaml)
    turbine.load_from_fast( path_params['FAST_InputFile'],os.path.join(yaml_dir,path_params['FAST_directory']))


    case_inputs = {}
    discon_lists = {}  

    for param_value in control_sweep_opts['param_values']:
        controller_params   = control_sweep_opts['controller_params'].copy()

        if 'control_param' in control_sweep_opts:
            controller_params[control_sweep_opts['control_param']] = param_value
        elif 'discon_param' in control_sweep_opts:
            controller_params.setdefault('DISCON', {})
            controller_params['DISCON'][control_sweep_opts['discon_param']] = param_value
        controller          = ROSCO_controller.Controller(controller_params)

        # tune default controller
        controller.tune_controller(turbine)

        discon_vt = ROSCO_utilities.DISCON_dict(turbine, controller, txt_filename=path_params['rotor_performance_filename'])
    
        for discon_input in discon_vt:
            if discon_input not in discon_lists:        # initialize
                discon_lists[discon_input] = []
            discon_lists[discon_input].append(discon_vt[discon_input])
        
    for discon_input, input in discon_lists.items():
        case_inputs[('DISCON_in',discon_input)] = {'vals': input, 'group': start_group+1}

    case_inputs_control = case_inputs
    return case_inputs_control


def check_inputs(control_sweep_opts,required_inputs):
    for ri in required_inputs:
        if type(ri) == str:
            if ri not in control_sweep_opts:
                raise Exception(f'{ri} is required for this control sweep')
        else:
            have_a_req_input = False
            for rk in ri:
                if rk in control_sweep_opts:
                    have_a_req_input = True
            if not have_a_req_input:
                raise Exception(f'One of {ri} is required for this control sweep')







#  def sweep_pc_mode(cont_yaml,omega=np.linspace(.05,.35,8,endpoint=True).tolist(),zeta=[1.5],group=2):
    
    
#     inps                = yaml.safe_load(open(cont_yaml))
#     path_params         = inps['path_params']
#     turbine_params      = inps['turbine_params']
#     controller_params   = inps['controller_params']

#     # make default controller, turbine objects for rosco.toolbox
#     turbine             = ROSCO_turbine.Turbine(turbine_params)
#     turbine.load_from_fast( path_params['FAST_InputFile'],path_params['FAST_directory'])

#     controller          = ROSCO_controller.Controller(controller_params)

#     # tune default controller
#     controller.tune_controller(turbine)

#     # check if inputs are lists
#     if not isinstance(omega,list):
#         omega = [omega]
#     if not isinstance(zeta,list):
#         zeta = [zeta]

#     # Loop through and make PI gains
#     pc_kp = []
#     pc_ki = []
#     m_omega = []  # flattened (omega,zeta) pairs
#     m_zeta = []  # flattened (omega,zeta) pairs
#     for o in omega:
#         for z in zeta:
#             controller.omega_pc = o
#             controller.zeta_pc  = z
#             controller.tune_controller(turbine)
#             pc_kp.append(controller.pc_gain_schedule.Kp.tolist())
#             pc_ki.append(controller.pc_gain_schedule.Ki.tolist())
#             m_omega.append(o)
#             m_zeta.append(z)

#     # add control gains to case_list
#     case_inputs = {}
#     case_inputs[('meta','omega')]          = {'vals': m_omega, 'group': group}
#     case_inputs[('meta','zeta')]          = {'vals': m_zeta, 'group': group}
#     case_inputs[('DISCON_in', 'PC_GS_KP')] = {'vals': pc_kp, 'group': group}
#     case_inputs[('DISCON_in', 'PC_GS_KI')] = {'vals': pc_ki, 'group': group}

#     return case_inputs

 ## Old sweep functions
     # # Tune Floating Feedback Gain
    # if tune == 'fl_gain':
    #     case_inputs[('DISCON_in','Fl_Kp')] = {'vals': np.linspace(0,-18,6,endpoint=True).tolist(), 'group': 2}

    # elif tune == 'fl_phase':
    #     case_inputs[('DISCON_in','Fl_Kp')] = {'vals': 8*[-25], 'group': 2}
    #     case_inputs[('DISCON_in','F_FlCornerFreq')] = {'vals': 8*[0.300], 'group': 2}
    #     case_inputs[('DISCON_in','F_FlHighPassFreq')] = {'vals':[0.001,0.005,0.010,0.020,0.030,0.042,0.060,0.100], 'group': 2}
    #     case_inputs[('meta','Fl_Phase')] = {'vals':8*[-50],'group':2}

    # elif tune == 'pc_mode':
    #     # define omega, zeta
    #     omega = np.linspace(.05,.25,8,endpoint=True).tolist()
    #     zeta  = np.linspace(1,3,3,endpoint=True).tolist()
        
    #     control_case_inputs = sweep_pc_mode(omega,zeta)
    #     case_inputs.update(control_case_inputs)



    # elif tune == 'max_tq':
    #     case_inputs[('DISCON_in','VS_MaxTq')] = {'vals': [19624046.66639, 1.5*19624046.66639], 'group': 3}

    # elif tune == 'yaw':
    #     case_inputs[('ElastoDyn','NacYaw')]     = {'vals': [-10,0,10], 'group': 3}





