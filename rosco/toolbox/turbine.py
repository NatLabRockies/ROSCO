# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import os
import numpy as np
import datetime
from scipy import interpolate
from numpy import gradient
import pickle
import matplotlib.pyplot as plt
import pandas as pd

from rosco.toolbox.utilities import load_from_txt

# Load OpenFAST readers
from openfast_io import FileTools

try:
    import weis.aeroelasticse
    use_weis = True
except:
    use_weis = False


# Some useful constants
now = datetime.datetime.now()
pi = np.pi
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class Turbine():
    """
    Class Turbine defines a turbine in terms of what is needed to 
        design the controller and to run the 'tiny' simulation.

    Primary functions (at a high level):
        - Reads an OpenFAST input deck 
        - Runs cc-blade rotor performance analysis
        - Writes a text file containing Cp, Ct, and Cq tables

    Methods:
    -------
    __str__
    load
    save
    load_from_fast
    load_from_ccblade
    load_from_txt
    generate_rotperf_fast

    Parameters:
    -----------
    turbine_params : dict
                    Dictionary containing known turbine paramaters that are not directly available from OpenFAST input files
    """

    def __init__(self, turbine_params):
        """
        Load turbine parameters from input dictionary
        """
        print('-----------------------------------------------------------------------------')
        print('Loading wind turbine data for NREL\'s ROSCO tuning and simulation processeses')
        print('-----------------------------------------------------------------------------')

        # ------ Turbine Parameters------
        self.rotor_inertia = turbine_params['rotor_inertia']         
        self.rated_rotor_speed = turbine_params['rated_rotor_speed']     
        self.v_min = turbine_params['v_min']                 
        self.v_rated = turbine_params['v_rated']               
        self.v_max = turbine_params['v_max']
        self.max_pitch_rate = turbine_params['max_pitch_rate'] 
        self.min_pitch_rate = -1 * self.max_pitch_rate
        self.max_torque_rate = turbine_params['max_torque_rate']             
        self.rated_power = turbine_params['rated_power']           
        self.bld_edgewise_freq = turbine_params['bld_edgewise_freq']     
        self.TSR_operational = turbine_params['TSR_operational']
        self.bld_flapwise_freq = turbine_params['bld_flapwise_freq']
        self.turbine_params = turbine_params


    # Allow print out of class
    def __str__(self): 
        '''
        Print some data about the turbine.
        '''
        print('-------------- Turbine Info --------------')
        print('Turbine Name: {}'.format(self.TurbineName))
        print('Rated Power: {} [W]'.format(self.rated_power))
        print('Total Inertia: {:.1f} [kg m^2]'.format(self.J))
        print('Rotor Radius: {:.1f} [m]'.format(self.rotor_radius))
        print('Rated Rotor Speed: {:.1f} [rad/s]'.format(self.rated_rotor_speed))
        print('Max Cp: {:.2f}'.format(self.Cp.max))
        print('------------------------------------------')
        return ' '

    # Save function
    def save(self,filename):
        '''
        Save turbine to pickle

        Parameters:
        ----------
        filename : str
                   Name of file to save pickle 
        # '''
        pickle.dump(self, open( filename, "wb" ) )

    # Load function
    @staticmethod
    def load(filename):
        '''
        Load turbine from pickle - outdated, but might be okay!!

        Parameters:
        ----------
        filename : str
                   Name of pickle file
        '''
        turbine = pickle.load(open(filename,'rb'))
        return turbine
    
    # Load data from fast input deck
    def load_from_fast(
            self, 
            FAST_InputFile,
            FAST_directory, 
            rot_source=None, 
            txt_filename=None,
            refine_cp_surface=False
            ):
        """
        Load the parameter files directly from a FAST input deck

        Parameters:
        -----------
            Fast_InputFile: str
                            Primary fast model input file (*.fst)
            FAST_directory: str
                            Directory for primary fast model input file
            rot_source: str, optional
                        desired source for rotor to get Cp, Ct, Cq tables. Default is to run cc-blade. 
                            options: cc-blade - run cc-blade
                                     txt - from *.txt file
            txt_filename: str, optional
                          filename for *.txt, only used if rot_source='txt'
        """
        # # Use weis if it exists
        # if use_weis:
        from openfast_io.FAST_reader import InputReader_OpenFAST # moving import here to avoid circular import
        # else:
        #     from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST

        # Load OpenFAST model using the FAST_reader
        print('Loading FAST model: %s ' % FAST_InputFile)
        self.TurbineName = FAST_InputFile.strip('.fst')
        fast = self.fast = InputReader_OpenFAST()
        fast.FAST_InputFile = FAST_InputFile
        fast.FAST_directory = FAST_directory

        fast.read_MainInput()

        ed_file = os.path.realpath(os.path.join(fast.FAST_directory, fast.fst_vt['Fst']['EDFile']))
        fast.read_ElastoDyn(ed_file)

        fast.read_AeroDyn()

        if fast.fst_vt['Fst']['CompServo']:
            fast.read_ServoDyn()
            fast.read_DISCON_in()
        else:
            fast.fst_vt['ServoDyn']['GenEff'] = 100.        # gen efficency defined in percent in ServoDyn
    

        # Use Performance tables if defined, otherwise use defaults
        if txt_filename:
            self.rotor_performance_filename = txt_filename
        else:
            self.rotor_performance_filename = 'Cp_Ct_Cq.txt'


        # Grab general turbine parameters
        self.TipRad             = fast.fst_vt['ElastoDyn']['TipRad']
        self.Rhub               = fast.fst_vt['ElastoDyn']['HubRad']
        self.hubHt              = fast.fst_vt['ElastoDyn']['TowerHt'] + fast.fst_vt['ElastoDyn']['Twr2Shft']
        self.NumBl              = fast.fst_vt['ElastoDyn']['NumBl']
        self.TowerHt            = fast.fst_vt['ElastoDyn']['TowerHt']
        self.shearExp           = 0.2  #NOTE: HARD CODED 
        
        # Fluid density
        if 'default' in str(fast.fst_vt['AeroDyn']['AirDens']):
            if fast.fst_vt['Fst']['MHK']:
                fast.fst_vt['AeroDyn']['AirDens'] = fast.fst_vt['Fst']['WtrDens']
            else:
                fast.fst_vt['AeroDyn']['AirDens'] = fast.fst_vt['Fst']['AirDens']
        self.rho                = fast.fst_vt['AeroDyn']['AirDens']

        # Kinematic viscosity
        if 'default' in str(fast.fst_vt['AeroDyn']['KinVisc']):
            fast.fst_vt['AeroDyn']['KinVisc'] = fast.fst_vt['Fst']['KinVisc']
            
        self.mu                 = fast.fst_vt['AeroDyn']['KinVisc']
        self.Ng                 = fast.fst_vt['ElastoDyn']['GBRatio']
        self.GenEff             = fast.fst_vt['ServoDyn']['GenEff']
        self.GBoxEff            = fast.fst_vt['ElastoDyn']['GBoxEff']
        self.DTTorSpr           = fast.fst_vt['ElastoDyn']['DTTorSpr']
        self.generator_inertia  = fast.fst_vt['ElastoDyn']['GenIner']
        self.tilt               = fast.fst_vt['ElastoDyn']['ShftTilt'] 
        try:
            self.precone = fast.fst_vt['ElastoDyn']['PreCone1'] # May need to change to PreCone(1) depending on OpenFAST files
        except:
            self.precone = fast.fst_vt['ElastoDyn']['PreCone(1)']
        self.yaw = 0.0
        self.J = self.rotor_inertia + self.generator_inertia * self.Ng**2
        self.rated_torque = self.rated_power/(self.GenEff/100*self.rated_rotor_speed*self.Ng)
        self.rotor_radius = self.TipRad

        # MHK cavitation properties (used to screen the operating schedule)
        self.MHK = fast.fst_vt['Fst']['MHK']
        if self.MHK:
            self.Patm = fast.fst_vt['Fst']['Patm']
            self.Pvap = fast.fst_vt['Fst']['Pvap']
            self.MSL2SWL = fast.fst_vt['Fst']['MSL2SWL']
            # Shallowest depth reached by the blade tip, at the top of its rotation.
            # hubHt is negative below MSL for an MHK turbine.
            self.tip_depth = abs(self.hubHt + self.MSL2SWL) - self.TipRad
            self.cavit_sigma_v = self.load_cavitation_number(fast)
        else:
            self.Patm = self.Pvap = self.MSL2SWL = None
            self.tip_depth = self.cavit_sigma_v = None

        # Load blade information
        self.load_blade_info()

        # Load Cp, Ct, Cq tables
        if rot_source == 'cc-blade': # Use cc-blade
            self.load_from_ccblade()
        elif rot_source == 'txt':    # Use specified text file
            self.pitch_initial_rad, self.TSR_initial, self.Cp_table, self.Ct_table, self.Cq_table = load_from_txt(
                txt_filename)
        else:   # Use text file from DISCON.in
            if os.path.exists(os.path.join(FAST_directory, fast.fst_vt['ServoDyn']['DLL_InFile'])):
                try:
                    self.pitch_initial_rad = fast.fst_vt['DISCON_in']['Cp_pitch_initial_rad']
                    self.TSR_initial = fast.fst_vt['DISCON_in']['Cp_TSR_initial']
                    self.Cp_table = fast.fst_vt['DISCON_in']['Cp_table']
                    self.Ct_table = fast.fst_vt['DISCON_in']['Ct_table']
                    self.Cq_table = fast.fst_vt['DISCON_in']['Cq_table']
                except:   # Load from cc-blade
                    print('No rotor performance data source available, running CC-Blade.')
                    self.load_from_ccblade()

        # Parse rotor performance data
        self.Cp = RotorPerformance(self.Cp_table,self.pitch_initial_rad,self.TSR_initial, refine=refine_cp_surface)
        self.Ct = RotorPerformance(self.Ct_table,self.pitch_initial_rad,self.TSR_initial, refine=refine_cp_surface)
        self.Cq = RotorPerformance(self.Cq_table,self.pitch_initial_rad,self.TSR_initial, refine=refine_cp_surface)

        # Define operational TSR
        if not self.TSR_operational:
            self.TSR_operational = self.Cp.TSR_opt
        # Compute operational Cp (may not be optimal if TSR_operational set by user)
        self.Cp_operational = self.Cp.interp_surface(self.Cp.pitch_opt, self.TSR_operational)

    def load_cavitation_number(self, fast):
        '''
        Smallest cavitation number the blade sections can tolerate, sigma_v = -Cp_min.

        Taken as the minimum of -Cp_min over the outboard airfoil polars, i.e. the most
        permissive (thinnest suction peak) condition the blade ever presents. Used to
        screen the operating schedule for tip cavitation in
        :code:`Controller.tune_controller`.

        Returns None if the AeroDyn polars carry no Cp_min column.
        '''
        if not fast.fst_vt['AeroDyn'].get('InCol_Cpmin', 0):
            return None

        # Airfoils used outboard of 70% span, where relative velocity is highest
        blade = fast.fst_vt['AeroDynBlade']
        span = np.array(blade['BlSpn'])
        afid = np.array(blade['BlAFID'], dtype=int)
        outboard = set(afid[span >= 0.7 * span[-1]])

        sigma_v = []
        for af in outboard:
            for tab in fast.fst_vt['AeroDyn']['af_data'][af - 1]:
                cpmin = np.array([c for c in tab['Cpmin'] if c is not None], dtype=float)
                if cpmin.size:
                    sigma_v.append(-cpmin.max())  # max Cp_min -> min -Cp_min

        return min(sigma_v) if sigma_v else None

    # Load rotor performance data from CCBlade 
    def load_from_ccblade(self):
        '''
        Loads rotor performance information by running cc-blade aerodynamic analysis. Designed to work with AeroDyn blade input files. 

        Parameters:
        -----------
            fast: dict
                  Dictionary containing fast model details - defined using from InputReader_OpenFAST (distributed as a part of AeroelasticSE)

        '''

        print('Loading rotor performance data from CC-Blade.')

        # Load blade information if it isn't already
        try:
            isinstance(self.cc_rotor,object)
        except(AttributeError):
            self.load_blade_info()
        
        # Generate the look-up tables, mesh the grid and flatten the arrays for cc_rotor aerodynamic analysis
        TSR_initial = np.arange(0.5, 25, 0.5)
        pitch_initial = np.arange(-5, 31, 1.)
        pitch_initial_rad = pitch_initial * deg2rad
        ws_array = np.ones_like(TSR_initial) * self.v_rated # evaluate at rated wind speed
        omega_array = (TSR_initial * ws_array / self.rotor_radius) * RadSec2rpm
        ws_mesh, pitch_mesh = np.meshgrid(ws_array, pitch_initial)
        ws_flat = ws_mesh.flatten()
        pitch_flat = pitch_mesh.flatten()
        omega_mesh, _ = np.meshgrid(omega_array, pitch_initial)
        omega_flat = omega_mesh.flatten()


        # Get values from cc-blade
        print('Running CCBlade aerodynamic analysis, this may take a minute...')
        outputs, derivs = self.cc_rotor.evaluate(ws_flat, omega_flat, pitch_flat, coefficients=True)
        CP = outputs['CP']
        CT = outputs['CT']
        CQ = outputs['CQ']
        print('CCBlade aerodynamic analysis run successfully.')

        # Reshape Cp, Ct and Cq
        Cp = np.transpose(np.reshape(CP, (len(pitch_initial), len(TSR_initial))))
        Ct = np.transpose(np.reshape(CT, (len(pitch_initial), len(TSR_initial))))
        Cq = np.transpose(np.reshape(CQ, (len(pitch_initial), len(TSR_initial))))

        # Store necessary metrics for analysis
        self.pitch_initial_rad = pitch_initial_rad
        self.TSR_initial = TSR_initial
        self.Cp_table = Cp
        self.Ct_table = Ct 
        self.Cq_table = Cq

    def generate_rotperf_fast(self, openfast_path, FAST_runDirectory=None, run_BeamDyn=False,
                              debug_level=1, run_type='multi'):
        '''
        Use openfast to generate Cp surface data. Will be slow, especially if using BeamDyn,
        but may be necessary if cc-blade is not sufficient.

        Parameters:
        -----------
        openfast_path: str
            path to openfast
        FAST_runDirectory: str
            directory to run openfast simulations in
        run_BeamDyn: bool
            Flag to run beamdyn - does not exist yet
        debug_level: float
            0 - no outputs, 1 - simple outputs, 2 - all outputs
        run_type: str
            'serial' - run in serial, 'multi' - run using python multiprocessing tools, 
            'mpi' - run using mpi tools
        '''
        if use_weis:
            from weis.aeroelasticse import runFAST_pywrapper, CaseGen_General
        else:
            from rosco.toolbox.ofTools.case_gen import runFAST_pywrapper, CaseGen_General
        # Load pCrunch tools
        from pCrunch import Processing

        # setup values for surface
        v0 = self.v_rated + 2
        TSR_initial = np.arange(3, 15,1)
        pitch_initial = np.arange(-1,25,1)
        rotspeed_initial = TSR_initial*v0/self.rotor_radius * RadSec2rpm # rpms

        # Specify Case Inputs
        case_inputs = {}

        # ------- Setup OpenFAST inputs --------
        case_inputs[('Fst','TMax')] = {'vals': [330], 'group': 0}
        case_inputs[('Fst','Compinflow')] = {'vals': [1], 'group': 0}
        case_inputs[('Fst','CompAero')] = {'vals': [2], 'group': 0}
        case_inputs[('Fst','CompServo')] = {'vals': [1], 'group': 0}
        case_inputs[('Fst','CompHydro')] = {'vals': [0], 'group': 0}
        if run_BeamDyn:
            case_inputs[('Fst','CompElast')] = {'vals': [2], 'group': 0}
        else:
            case_inputs[('Fst','CompElast')] = {'vals': [1], 'group': 0}
        case_inputs[('Fst', 'OutFileFmt')] = {'vals': [2], 'group': 0}

        # AeroDyn
        case_inputs[('AeroDyn', 'WakeMod')] = {'vals': [1], 'group': 0}
        case_inputs[('AeroDyn', 'AfAeroMod')] = {'vals': [1], 'group': 0}
        case_inputs[('AeroDyn', 'TwrPotent')] = {'vals': [0], 'group': 0}
        
        # ElastoDyn
        case_inputs[('ElastoDyn', 'FlapDOF1')] = {'vals': ['True'], 'group': 0}
        case_inputs[('ElastoDyn', 'FlapDOF2')] = {'vals': ['True'], 'group': 0}
        case_inputs[('ElastoDyn', 'EdgeDOF')] = {'vals': ['True'], 'group': 0}
        case_inputs[('ElastoDyn', 'TeetDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'DrTrDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'GenDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'YawDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'TwFADOF1')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'TwFADOF2')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'TwSSDOF1')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'TwSSDOF2')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmSgDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmSwDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmHvDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmRDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmPDOF')] = {'vals': ['False'], 'group': 0}
        case_inputs[('ElastoDyn', 'PtfmYDOF')] = {'vals': ['False'], 'group': 0}

        # BeamDyn
        # NEEDED

        # InflowWind
        case_inputs[('InflowWind', 'WindType')] = {'vals': [1], 'group': 0}
        case_inputs[('InflowWind', 'HWindSpeed')] = {'vals': [v0], 'group': 0}
        case_inputs[('InflowWind', 'PLexp')] = {'vals': [0], 'group': 0}

        # ServoDyn
        case_inputs[('ServoDyn', 'PCMode')] = {'vals': [0], 'group': 0}
        case_inputs[('ServoDyn', 'VSContrl')] = {'vals': [0], 'group': 0} 
        case_inputs[('ServoDyn', 'HSSBrMode')] = {'vals': [0], 'group': 0}
        case_inputs[('ServoDyn', 'YCMode')] = {'vals': [0], 'group': 0}

        # ------- Setup sweep values inputs --------
        case_inputs[('ElastoDyn', 'BlPitch1')] = {'vals': list(pitch_initial), 'group': 1}
        case_inputs[('ElastoDyn', 'BlPitch2')] = {'vals': list(pitch_initial), 'group': 1}
        case_inputs[('ElastoDyn', 'BlPitch3')] = {'vals': list(pitch_initial), 'group': 1}
        case_inputs[('ElastoDyn', 'RotSpeed')] = {'vals': list(rotspeed_initial), 'group': 2}


        # FAST details
        fastBatch = runFAST_pywrapper.runFAST_pywrapper_batch()
        fastBatch.FAST_exe = openfast_path  # Path to executable
        fastBatch.FAST_InputFile = self.fast.FAST_InputFile
        fastBatch.FAST_directory = self.fast.FAST_directory
        if not FAST_runDirectory:
            FAST_runDirectory = os.path.join(os.getcwd(), 'RotPerf_OpenFAST')
        fastBatch.FAST_runDirectory = FAST_runDirectory
        fastBatch.debug_level = debug_level

        # Generate cases
        case_name_base = self.TurbineName + '_rotperf'
        case_list, case_name_list = CaseGen_General.CaseGen_General(
            case_inputs, dir_matrix=fastBatch.FAST_runDirectory, namebase=case_name_base)
        fastBatch.case_list = case_list
        fastBatch.case_name_list = case_name_list

        # Make sure proper outputs exist
        var_out = [
            # ElastoDyn (this is probably overkill on the outputs)
            "BldPitch1", "BldPitch2", "BldPitch3", "Azimuth", "RotSpeed", "GenSpeed", "NacYaw",
            "OoPDefl1", "IPDefl1", "TwstDefl1", "OoPDefl2", "IPDefl2", "TwstDefl2", "OoPDefl3",
            "IPDefl3", "TwstDefl3", "RootFxc1",
            "RootFyc1", "RootFzc1", "RootMxc1", "RootMyc1", "RootMzc1", "RootFxc2", "RootFyc2",
            "RootFzc2", "RootMxc2", "RootMyc2", "RootMzc2", "RootFxc3", "RootFyc3", "RootFzc3",
            "RootMxc3", "RootMyc3", "RootMzc3", "Spn1MLxb1", "Spn1MLyb1", "Spn1MLzb1", "Spn1MLxb2",
            "Spn1MLyb2", "Spn1MLzb2", "Spn1MLxb3", "Spn1MLyb3", "Spn1MLzb3", "RotThrust", "LSSGagFya",
            "LSSGagFza", "RotTorq", "LSSGagMya", "LSSGagMza", 
            # ServoDyn
            "GenPwr", "GenTq",
            # AeroDyn
            "RtArea", "RtVAvgxh", "B1N3Clrnc", "B2N3Clrnc", "B3N3Clrnc",
            "RtFldCp", 'RtFldCq', 'RtFldCt', 'RtTSR', # NECESSARY
            # InflowWind
            "Wind1VelX", 
        ]
        channels = {}
        for var in var_out:
            channels[var] = True
        fastBatch.channels = channels

        # Run OpenFAST
        if run_type.lower() == 'multi':
            fastBatch.run_multi()
        elif run_type.lower()=='mpi':
            fastBatch.run_mpi()
        elif run_type.lower()=='serial':
            fastBatch.run_serial()

        # ========== Post Processing ==========
        # Save statistics
        fp = Processing.FAST_Processing()

        # Find all outfiles
        fname_case_matrix = os.path.join(FAST_runDirectory, 'case_matrix.yaml')
        case_matrix = FileTools.load_yaml(fname_case_matrix, package=1)
        cm = pd.DataFrame(case_matrix)
        # Parse case matrix and find outfiles names
        outfiles = []
        case_names = cm['Case_Name']
        outfiles = []
        for name in case_names:
            outfiles.append(os.path.join(FAST_runDirectory, name + '.outb'))



        # Set some processing parameters
        fp.OpenFAST_outfile_list = outfiles
        fp.namebase = case_name_base
        fp.t0 = 270 
        fp.parallel_analysis = True
        fp.results_dir = os.path.join(FAST_runDirectory,'stats')
        fp.verbose = True
        # Save for debug!
        fp.save_LoadRanking = False
        fp.save_SummaryStats = False

        print('Processing openfast data on {} cores.'.format(fp.parallel_cores))

        # Load and save statistics and load rankings
        stats, load_rankings = fp.batch_processing()

        # Get means of last 30 seconds of 300 second simulation
        CP = stats[0]['RtAeroCp']['mean']
        CT = stats[0]['RtAeroCt']['mean']
        CQ = stats[0]['RtAeroCq']['mean']

        # Reshape Cp, Ct and Cq
        Cp = np.transpose(np.reshape(CP, (len(pitch_initial), len(TSR_initial))))
        Ct = np.transpose(np.reshape(CT, (len(pitch_initial), len(TSR_initial))))
        Cq = np.transpose(np.reshape(CQ, (len(pitch_initial), len(TSR_initial))))

        # Store necessary metrics for analysis
        self.pitch_initial_rad = pitch_initial * deg2rad
        self.TSR_initial = TSR_initial
        self.Cp_table = Cp
        self.Ct_table = Ct
        self.Cq_table = Cq

    
    def load_blade_info(self):
        '''
        Loads wind turbine blade data from an OpenFAST model. 

        Should be used if blade information is needed (i.e. for flap controller tuning), 
        but a rotor performance file is defined and and cc-blade does not need to be run. 
        
        Parameters:
        -----------
            self - note: needs to contain fast input file info provided by load_from_fast.
        '''
        # if use_weis:
        from openfast_io.FAST_reader import InputReader_OpenFAST
        # else:
        #     from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
        from wisdem.ccblade.ccblade import CCAirfoil, CCBlade

        # Create CC-Blade Rotor
        r0 = np.array(self.fast.fst_vt['AeroDynBlade']['BlSpn']) 
        chord = np.array(self.fast.fst_vt['AeroDynBlade']['BlChord'])
        theta = np.array(self.fast.fst_vt['AeroDynBlade']['BlTwist'])
        # -- Adjust for AeroDyn
        r = r0 + self.Rhub
        af_idx = np.array(self.fast.fst_vt['AeroDynBlade']['BlAFID']).astype(int) - 1 #Reset to 0 index

        # Read OpenFAST Airfoil data, assumes AeroDyn > v15.03 and associated polars > v1.01
        af_dict = {}
        for i, section in enumerate(self.fast.fst_vt['AeroDyn']['af_data']):
            if section[0]['NumTabs'] > 1:  # sections with multiple airfoil tables
                reynolds_ref = self.turbine_params['reynolds_ref']
                if reynolds_ref:  # default is 0
                    # find closest table to reynolds_ref, interpolate someday, approximate is probably okay
                    Res_in_table = np.array([sec['Re'] for sec in section])
                    ref_tab = np.abs(Res_in_table - reynolds_ref).argmin()
                else:
                    print("ROSCO Warning: No Reynolds number provided and airfoils have multiple tables, using center table")
                    ref_tab = int(np.floor(section[0]['NumTabs']/2)) # get information from "center" table
                Alpha = section[ref_tab]['Alpha']
                Re = np.array([section[ref_tab]['Re']])
                Cl = section[ref_tab]['Cl']
                Cd = section[ref_tab]['Cd']
                Cm = section[ref_tab]['Cm']
                af_dict[i] = CCAirfoil(Alpha, Re, Cl, Cd, Cm)
            else:        
                Alpha = section[0]['Alpha']                   # sections without multiple airfoil tables
                Re = np.array([section[0]['Re']])
                Cl = section[0]['Cl']
                Cd = section[0]['Cd']
                Cm = section[0]['Cm']
                af_dict[i] = CCAirfoil(Alpha, Re, Cl, Cd, Cm)

        # define airfoils for CCBlade
        af = [0]*len(r)
        for i in range(len(r)):
            af[i] = af_dict[af_idx[i]]

        # Now save the CC-Blade rotor
        nSector = 8  # azimuthal discretizations
        self.cc_rotor = CCBlade(r, chord, theta, af, self.Rhub, self.rotor_radius, self.NumBl, rho=self.rho, mu=self.mu,
                        precone=-self.precone, tilt=-self.tilt, yaw=self.yaw, shearExp=self.shearExp, hubHt=self.hubHt, nSector=nSector)

        # Save some additional blade data 
        self.af_data = self.fast.fst_vt['AeroDyn']['af_data']
        self.span = r 
        self.chord = chord
        self.twist = theta
        
class RotorPerformance():
    '''
    Class RotorPerformance used to find details from rotor performance 
        tables generated by CC-blade or similer BEM-solvers. 

    Methods:
    --------
    interp_surface
    interp_gradient
    plot_performance

    Parameters:
    -----------
    performance_table : array_like (-)
                        An [n x m] array containing a table of rotor performance data (Cp, Ct, Cq).
    pitch_initial_rad : array_like (rad)
                        An [m x 1] or [1 x m] array containing blade pitch angles corresponding to performance_table. 
    TSR_initial : array_like (rad)
                    An [n x 1] or [1 x n] array containing tip-speed ratios corresponding to  performance_table 
    '''
    def __init__(self, performance_table, pitch_initial_rad, TSR_initial, refine=False):

        # Store performance data tables
        self.performance_table = performance_table          # Table containing rotor performance data, i.e. Cp, Ct, Cq
        self.pitch_initial_rad = pitch_initial_rad          # Pitch angles corresponding to x-axis of performance_table (rad)
        self.TSR_initial = TSR_initial                      # Tip-Speed-Ratios corresponding to y-axis of performance_table (rad)

        # Calculate Gradients
        self.gradient_TSR, self.gradient_pitch = gradient(performance_table)             # gradient_TSR along y-axis, gradient_pitch along x-axis (rows, columns)

        # "Optimal" below rated TSR and blade pitch (for Cp) - note this may be limited by resolution of Cp-surface
        self.max = np.amax(performance_table)
        self.max_ind = np.where(performance_table == np.amax(performance_table))

        # Throw a warning if there is more than one max pitch angle
        if len(self.max_ind[1]) > 1:
            print('rosco.toolbox Warning: repeated maximum values in a performance table and the last one @ pitch = {} rad. was taken...'.format(self.pitch_opt[-1]))

        # Refine optimal point using smooth interpolation -- gives more precise estimates than coarse grid
        # Select region around maximizing TSR and blade pitch
        TSR_max_ind = self.max_ind[0][-1]
        pitch_max_ind = self.max_ind[1][-1]

        self.TSR_opt = float(TSR_initial[TSR_max_ind]) # If multiple maximizers, pick lowest TSR
        self.pitch_opt = float(pitch_initial_rad[pitch_max_ind]) # If multiple maximizers, pick highest pitch

        if refine:
            ind_search_num = 3  # Number of indices to expand search in each direction
            TSR_search_ind = [np.max([0, TSR_max_ind - ind_search_num]), np.min([TSR_max_ind + ind_search_num, len(self.TSR_initial)-1])]
            pitch_search_ind = [np.max([0, pitch_max_ind - ind_search_num]), np.min([pitch_max_ind + ind_search_num, len(self.pitch_initial_rad)-1])]
            TSR_search_range = range(TSR_search_ind[0], TSR_search_ind[1]+1)
            pitch_search_range = range(pitch_search_ind[0], pitch_search_ind[1]+1)


            # Generate finer mesh in the proximity of maximizer
            fine_mesh_scale = 20
            TSR_fine = np.linspace(self.TSR_initial[TSR_search_ind[0]], self.TSR_initial[TSR_search_ind[1]], np.diff(TSR_search_ind)[0] * fine_mesh_scale)
            pitch_fine = np.linspace(self.pitch_initial_rad[pitch_search_ind[0]], self.pitch_initial_rad[pitch_search_ind[1]], np.diff(pitch_search_ind)[0] * fine_mesh_scale)
            pitch_fine_mesh, TSR_fine_mesh = np.meshgrid(pitch_fine, TSR_fine) # Construct mesh grids of fine data to interpolate performance surface
            performance_fine = interpolate.interpn(
                [TSR_initial[TSR_search_range], pitch_initial_rad[pitch_search_range]],
                self.performance_table[TSR_search_range, :][:, pitch_search_range],
                np.array([TSR_fine_mesh, pitch_fine_mesh]).T,
                bounds_error='False', method='cubic').T # Cubic spline interpolation over finer mesh data


            # Save optimal performance, TSR, and pitch
            performance_opt = np.max(performance_fine) # Maximal Cx on fine grid
            performance_max_ind = np.where(performance_fine == performance_opt) # Find maximizer
            self.TSR_opt = float(TSR_fine[performance_max_ind[0][0]]) # If multiple maximizers, pick lowest TSR
            self.pitch_opt = float(pitch_fine[performance_max_ind[1][-1]]) # If multiple maximizers, pick highest pitch

            if False:  # for debugging refinement
                print(f'rosco.toolbox: Original maximum performance estimate: {self.max} @ pitch = {self.pitch_initial_rad[pitch_max_ind]} rad, TSR = {self.TSR_initial[TSR_max_ind]} rad')
                print(f'rosco.toolbox: Refined maximum performance estimate: {performance_opt} @ pitch = {self.pitch_opt} rad, TSR = {self.TSR_opt} rad')
                import matplotlib.pyplot as plt
                performance_table[performance_table<0] = 0
                fig = plt.figure()
                plt.contourf(pitch_initial_rad*rad2deg, TSR_initial, performance_table, 20)
                plt.xlim([-3,3])
                plt.ylim([6,9])
                plt.colorbar()
            
                fig = plt.figure()
                plt.contourf(pitch_fine_mesh*rad2deg, TSR_fine_mesh, performance_fine, 20)
                plt.colorbar()

                print('here')

    def interp_surface(self,pitch,TSR):
        '''
        2d interpolation to find point on rotor performance surface
        
        Parameters:
        -----------
        pitch : float (rad)
                Pitch angle to look up
        TSR : float (rad)
              Tip-speed ratio to look up
        '''
        
        # Form the interpolant functions which can look up any arbitrary location on rotor performance surface
        interp_fun = interpolate.RectBivariateSpline(
            self.pitch_initial_rad, self.TSR_initial, self.performance_table.T)
        return np.squeeze(interp_fun(pitch,TSR).T)

    def interp_gradient(self,pitch,TSR):
        '''
        2d interpolation to find gradient at a specified point on rotor performance surface
        
        Parameters:
        -----------
        pitch : float (rad)
                Pitch angle to look up
        TSR : float (rad)
              Tip-speed ratio to look up

        Returns:
        --------
        interp_gradient : array_like
                          [1 x 2] array coresponding to gradient in pitch and TSR directions, respectively
        '''
        # Form the interpolant functions to find gradient at any arbitrary location on rotor performance surface
        dCP_beta_interp = interpolate.RectBivariateSpline(self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch.T)
        dCP_TSR_interp = interpolate.RectBivariateSpline(self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR.T)

        # grad.shape output as (2,) numpy array, equivalent to (pitch-direction,TSR-direction)
        grad = np.array([dCP_beta_interp(pitch,TSR).T, dCP_TSR_interp(pitch,TSR).T])
        return np.ndarray.flatten(grad)
    
    def plot_performance(self):
        '''
        Plot rotor performance data surface. 
        
        Parameters:
        -----------
        self
        '''

        # Find maximum point
        max_ind = np.unravel_index(np.argmax(self.performance_table, axis=None), self.performance_table.shape)
        max_beta_id = self.pitch_initial_rad[max_ind[1]]
        max_tsr_id = self.TSR_initial[max_ind[0]]

        cbarticks = np.linspace(0.0,self.performance_table.max(),20)

        P = plt.contourf(self.pitch_initial_rad * rad2deg, self.TSR_initial, self.performance_table, cbarticks)
                        # levels=20,vmin=0)
        plt.colorbar(format='%1.3f')
        # plt.title('Power Coefficient', fontsize=14, fontweight='bold')
        plt.xlabel('Pitch Angle [deg]', fontsize=14, fontweight='bold')
        plt.ylabel('TSR [-]', fontsize=14, fontweight='bold')
        plt.scatter(max_beta_id * rad2deg, max_tsr_id, color='red')
        plt.annotate('max = {:<1.3f}'.format(np.max(self.performance_table)),
                    (max_beta_id+0.2, max_tsr_id+0.2), color='red')
        plt.xticks(fontsize=12)
        plt.yticks(fontsize=12)
