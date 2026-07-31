# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import numpy as np
import os
import datetime
from scipy import interpolate, integrate
from rosco.toolbox.utilities import list_check
from scipy import optimize

# Some useful constants
now = datetime.datetime.now()
pi = np.pi
rad2deg = np.rad2deg(1)
deg2rad = np.deg2rad(1)
rpm2RadSec = 2.0*(np.pi)/60.0
RadSec2rpm = 60/(2.0 * np.pi)

class Controller():
    """
    Class Controller used to calculate controller tunings parameters


    Methods:
    -------
    tune_controller

    Parameters:
    -----------
    controller_params: dict
                       Dictionary containing controller paramaters that need to be defined
    """

    def __init__(self, controller_params):
        ''' 
        Load controller tuning parameters from input dictionary
        '''

        print('-----------------------------------------------------------------------------')
        print('   Tuning a reference wind turbine controller using NREL\'s ROSCO toolbox    ')
        print('-----------------------------------------------------------------------------')

        # Controller Flags
        self.LoggingLevel       = controller_params['LoggingLevel']
        self.F_LPFType          = controller_params['F_LPFType']
        self.F_NotchType        = controller_params['F_NotchType']
        self.IPC_ControlMode    = controller_params['IPC_ControlMode']
        self.VS_ControlMode     = controller_params['VS_ControlMode']
        self.VS_ConstPower      = controller_params['VS_ConstPower']
        self.VS_FBP             = controller_params['VS_FBP']
        self.PC_ControlMode     = controller_params['PC_ControlMode']
        self.Y_ControlMode      = controller_params['Y_ControlMode']
        self.SS_Mode            = controller_params['SS_Mode']
        self.WE_Mode            = controller_params['WE_Mode']
        self.PS_Mode            = controller_params['PS_Mode']
        self.SU_Mode            = controller_params['SU_Mode']
        self.SD_Mode            = controller_params['SD_Mode']
        self.Fl_Mode            = controller_params['Fl_Mode']
        self.TD_Mode            = controller_params['TD_Mode']
        self.TRA_Mode           = controller_params['TRA_Mode']
        self.Flp_Mode           = controller_params['Flp_Mode']
        self.PA_Mode            = controller_params['PA_Mode']
        self.PF_Mode            = controller_params['PF_Mode']
        self.PRC_Mode           = controller_params['PRC_Mode']
        self.AWC_Mode           = controller_params['AWC_Mode']
        self.Ext_Mode           = controller_params['Ext_Mode']
        self.ZMQ_Mode           = controller_params['ZMQ_Mode']
        self.CC_Mode            = controller_params['CC_Mode']
        self.StC_Mode           = controller_params['StC_Mode']

        # Necessary parameters
        self.U_pc = list_check(controller_params['U_pc'], return_bool=False)
        self.zeta_pc = list_check(controller_params['zeta_pc'], return_bool=False)
        self.omega_pc = list_check(controller_params['omega_pc'], return_bool=False)
        self.zeta_vs = controller_params['zeta_vs']
        self.omega_vs = controller_params['omega_vs']
        self.interp_type = controller_params['interp_type']

        # Optional parameters with defaults
        self.max_pitch          = controller_params['max_pitch']
        self.vs_minspd          = controller_params['vs_minspd']
        self.ss_vsgain          = controller_params['ss_vsgain']
        self.ss_pcgain          = controller_params['ss_pcgain']
        self.ps_percent         = controller_params['ps_percent']
        self.WS_GS_n            = controller_params['WS_GS_n']
        self.PC_GS_n            = controller_params['PC_GS_n']
        self.flp_maxpit         = controller_params['flp_maxpit']
        self.Kp_ipc1p           = controller_params['IPC_Kp1p']
        self.Ki_ipc1p           = controller_params['IPC_Ki1p']
        self.Kp_ipc2p           = controller_params['IPC_Kp2p']
        self.Ki_ipc2p           = controller_params['IPC_Kp2p']
        self.IPC_Vramp          = controller_params['IPC_Vramp']
        self.ZMQ_UpdatePeriod   = controller_params['ZMQ_UpdatePeriod']

        # FBP config defaults to constant power, underspeed
        self.fbp_power_mode = controller_params['VS_FBP_power_mode']
        self.fbp_speed_mode = controller_params['VS_FBP_speed_mode']
        self.fbp_U = controller_params['VS_FBP_U'] # DBS: Should we set this default based on rated speed?
        self.fbp_P = controller_params['VS_FBP_P']

        # Optional parameters without defaults
        if 'min_pitch' in controller_params:
            # Set below using Cp surface if no user input
            self.min_pitch = controller_params['min_pitch']

        if self.VS_FBP > 0:
            
            # Fail if generator torque enabled in Region 3 but pitch control not disabled (may enable these modes to operate together in the future)
            if self.PC_ControlMode != 0:
                raise Exception(
                    'rosco.toolbox:controller: PC_ControlMode must be 0 if VS_FBP > 0')

        if self.Flp_Mode > 0:
            if 'flp_kp_norm' in controller_params and 'flp_tau' in controller_params:
                self.flp_kp_norm = controller_params['flp_kp_norm']
                self.flp_tau     = controller_params['flp_tau']
            else:
                raise Exception(
                    'rosco.toolbox:controller: flp_kp_norm and flp_tau must be set if Flp_Mode > 0')

        if self.Fl_Mode > 0:
            if 'twr_freq' in controller_params and 'ptfm_freq' in controller_params:
                self.twr_freq   = controller_params['twr_freq']
                self.ptfm_freq  = controller_params['ptfm_freq']
            else:
                raise Exception('rosco.toolbox:controller: twr_freq and ptfm_freq must be set if Fl_Mode > 0')

            # Kp_float direct setting
            if 'Kp_float' in controller_params:
                self.Kp_float = controller_params['Kp_float']
            else:
                self.Kp_float = np.array([0])

            self.tune_Fl = controller_params['tune_Fl']


        else:
            self.twr_freq   = 0
            self.ptfm_freq  = 0

        # Use critical damping if LPFType = 2
        if controller_params['F_LPFType'] == 2:
            self.F_LPFDamping = 0.7
        else:
            self.F_LPFDamping = 0.0

        # Filter parameters
        self.f_we_cornerfreq        = controller_params['filter_params']['f_we_cornerfreq']
        self.f_fl_highpassfreq      = controller_params['filter_params']['f_fl_highpassfreq']
        self.f_ss_cornerfreq        = controller_params['filter_params']['f_ss_cornerfreq']
        self.f_yawerr               = controller_params['filter_params']['f_yawerr']
        self.f_vs_refspd_cornerfreq = controller_params['filter_params']['f_vs_refspd_cornerfreq']
        self.f_sd_pitchcornerfreq        = controller_params['filter_params']['f_sd_pitchcornerfreq']
        self.f_sd_yawerrorcornerfreq        = controller_params['filter_params']['f_sd_yawerrorcornerfreq']
        self.f_sd_genspdcornerfreq        = controller_params['filter_params']['f_sd_genspdcornerfreq']

        # Open loop parameters: set up and error catching
        self.OL_Mode            = controller_params['OL_Mode']
        self.OL_Filename        = controller_params['open_loop']['filename']
        self.Ind_Breakpoint  = self.Ind_GenTq = self.Ind_YawRate = self.Ind_Azimuth = 0
        self.Ind_R_Speed = self.Ind_R_Torque = self.Ind_R_Pitch = 0
        self.Ind_BldPitch        = [0,0,0]
        self.Ind_CableControl    = [0]
        self.Ind_StructControl   = [0]
        self.OL_BP_Mode             = 0
        
        if self.OL_Mode:
            ol_params               = controller_params['open_loop']

            # Apply DISCON inputs if they exist
            if 'OL_Filename' in controller_params['DISCON']:
                ol_params['filename'] = controller_params['DISCON']['OL_Filename']

            available_ol_params = [
                'OL_BP_Mode',
                'Ind_Breakpoint',
                'Ind_BldPitch',
                'Ind_GenTq',
                'Ind_YawRate',
                'Ind_Azimuth',
                'Ind_R_Speed',
                'Ind_R_Torque',
                'Ind_R_Pitch',
                'Ind_CableControl',
                'Ind_StructControl'
                ]
            for param in available_ol_params:
                if param in controller_params['DISCON']:
                    ol_params[param] = controller_params['DISCON'][param]


            self.OL_BP_Mode         = ol_params['OL_BP_Mode']
            self.Ind_Breakpoint  = ol_params['Ind_Breakpoint']
            self.Ind_BldPitch    = ol_params['Ind_BldPitch']
            self.Ind_GenTq       = ol_params['Ind_GenTq']
            self.Ind_YawRate     = ol_params['Ind_YawRate']
            self.Ind_Azimuth     = ol_params['Ind_Azimuth']
            self.Ind_R_Speed     = ol_params['Ind_R_Speed']
            self.Ind_R_Torque    = ol_params['Ind_R_Torque']
            self.Ind_R_Pitch     = ol_params['Ind_R_Pitch']
            self.Ind_CableControl     = ol_params['Ind_CableControl']
            self.Ind_StructControl    = ol_params['Ind_StructControl']

            # Check that file exists because we won't write it
            if not os.path.exists(controller_params['open_loop']['filename']):
                raise Exception(f'Open-loop control set up, but the open loop file {self.OL_Filename} does not exist')
            

        # Pitch actuator parameters
        self.PA_Mode = controller_params['PA_Mode']
        self.PA_CornerFreq = controller_params['PA_CornerFreq']
        self.PA_Damping = controller_params['PA_Damping']

        # Save controller_params for later (direct passthrough)
        self.controller_params = controller_params

        # Error checking: number of breakpoints
        if self.WS_GS_n <= self.PC_GS_n:
            raise Exception('Number of WS breakpoints is not greater than pitch control breakpoints')

        # Error checking: pitch controller inputs
        if list_check(self.U_pc) and \
            (list_check(self.omega_pc) or list_check(self.zeta_pc)) and \
                not len(self.U_pc) == len(self.omega_pc) == len(self.zeta_pc):
            raise Exception(
                'U_pc, omega_pc, and zeta_pc are all list-like and are not of equal length')


    def tune_controller(self, turbine):
        """
        Given a turbine model, tune a controller based on the NREL generic controller tuning process

        Parameters:
        -----------
        turbine : class
                  Turbine class containing necessary turbine information to accurately tune the controller. 
        """
        # -------------Load Parameters ------------- #
        # Re-define Turbine Parameters for shorthand
        J = turbine.J                           # Total rotor inertial (kg-m^2) 
        rho = turbine.rho                       # Air density (kg/m^3)
        R = turbine.rotor_radius                    # Rotor radius (m)
        Ar = np.pi*R**2                         # Rotor area (m^2)
        Ng = turbine.Ng                         # Gearbox ratio (-)
        rated_rotor_speed = turbine.rated_rotor_speed               # Rated rotor speed (rad/s)

        # ------------- Saturation Limits --------------- #
        turbine.max_torque = turbine.rated_torque * self.controller_params['max_torque_factor']
        if not hasattr(self, 'min_pitch'): # Set min pitch to optimal if no user input
            self.min_pitch = turbine.Cp.pitch_opt
        turbine.min_pitch = self.min_pitch

        # -------------Define Operation Points ------------- #
        TSR_rated = rated_rotor_speed*R/turbine.v_rated  # TSR at rated

        # separate wind speeds by operation regions
        # add one to above rated because we don't use rated in the pitch control gain scheduling
        v_below_rated = np.linspace(turbine.v_min,turbine.v_rated, num=self.WS_GS_n-self.PC_GS_n)[:-1]             # below rated
        v_above_rated = np.linspace(turbine.v_rated,turbine.v_max, num=self.PC_GS_n+1)             # above rated
        v = np.concatenate((v_below_rated, v_above_rated))

        # Construct power schedule differently based on pitch control configuration
        if self.VS_FBP > 0: # If using torque control in Region 3

            # Check if constant power control disabled (may be implemented to work concurrently in the future)
            if self.VS_ConstPower != 0:
                raise Exception("VS_ConstPower must be 0 when VS_FBP > 0")

            # Begin with user-defined power curve from input yaml (default constant rated power)
            f_P_user_defined = interpolate.interp1d(self.fbp_U, self.fbp_P, fill_value=(self.fbp_P[0], self.fbp_P[-1]), bounds_error=False)
            P_user_defined = f_P_user_defined(v)
            if self.fbp_power_mode == 0:
                P_user_defined *= turbine.rated_power
            # Maximum potential power from MPPT (extending Region 2 power curve to cut-out)
            Cp_operational = turbine.Cp.interp_surface(self.min_pitch, turbine.TSR_operational) # Compute TSR at controller pitch saturation and operational TSR (may not be optimal)
            P_max = 0.5 * turbine.rho * np.pi*turbine.rotor_radius**2 * Cp_operational * v**3 \
                * turbine.GBoxEff/100 * turbine.GenEff/100 # Includes generator efficiency reduction from available inflow power
            # Take minimum between user input and max possible
            P_op = np.min([P_user_defined, P_max], axis=0)
            # Operation along Cp surface (with fixed pitch)
            Cp_op = (P_op / P_max) * Cp_operational
            Cp_op_br = Cp_op[:len(v_below_rated)]
            Cp_op_ar = Cp_op[len(v_below_rated):]

            # Identify TSR matching the Cp values (similar to variable pitch angle interpolation below)
            Cp_FBP = np.ndarray.flatten(turbine.Cp.interp_surface(self.min_pitch, turbine.TSR_initial))     # all Cp values for fine blade pitch
            Cp_maxidx = Cp_FBP.argmax()
            # When we depart from Cp_max, our TSR has to fall to either above or below TSR_opt, leading to overspeed and underspeed configurations
            if self.fbp_speed_mode: # Overspeed
                # Interpolate inverse Cp surface slice with TSR >= TSR_opt
                Cp_op = np.clip(Cp_op, np.min(Cp_FBP[Cp_maxidx:]), np.max(Cp_FBP[Cp_maxidx:]))            # saturate Cp values to be on Cp surface                                                             # Find maximum Cp value for this TSR
                f_cp_TSR = interpolate.interp1d(Cp_FBP[Cp_maxidx:], turbine.TSR_initial[Cp_maxidx:])             # interpolate function for Cp(tsr) values
            else: # Underspeed
                # Interpolate inverse Cp surface slice with TSR <= TSR_opt
                Cp_op = np.clip(Cp_op, np.min(Cp_FBP[:Cp_maxidx+1]), np.max(Cp_FBP[:Cp_maxidx+1]))            # saturate Cp values to be on Cp surface                                                             # Find maximum Cp value for this TSR
                f_cp_TSR = interpolate.interp1d(Cp_FBP[:Cp_maxidx+1], turbine.TSR_initial[:Cp_maxidx+1])             # interpolate function for Cp(tsr) values
            TSR_op = f_cp_TSR(Cp_op)
            # Defer to operational TSR for below rated, even if other optimum (should keep Cp the same, but may lead to discontinuities in operating schedule if min_pitch is not optimal)
            TSR_op[v < turbine.v_rated] = turbine.TSR_operational
            TSR_below_rated = TSR_op[:len(v_below_rated)] # Should be constant at TSR_operational
            TSR_above_rated = TSR_op[len(v_below_rated):]

        # elif self.PC_ControlMode > 0: # If using pitch control in Region 3
        else: # Default here even if pitch control disabled to maintain backwards compatibility

            # TSR setpoints
            TSR_below_rated = [min(turbine.TSR_operational, rated_rotor_speed*R/v) for v in v_below_rated] # below rated, saturated to not exceed rated rotor speed
            TSR_above_rated = rated_rotor_speed*R/v_above_rated                     # above rated
            TSR_op = np.concatenate((TSR_below_rated, TSR_above_rated))             # operational TSRs

            # Find expected operational Cp values
            Cp_above_rated = turbine.Cp.interp_surface(self.min_pitch,TSR_above_rated[0])     # Cp during rated operation (not optimal)
            Cp_op_ar = Cp_above_rated * (TSR_above_rated/TSR_rated)**3           # above rated
            Cp_op_br = [turbine.Cp.interp_surface(self.min_pitch, TSR) for TSR in TSR_below_rated] # Below rated, reinterpolated for accurate operational Cp based on controller pitch
            Cp_op = np.concatenate((Cp_op_br, Cp_op_ar))                         # operational CPs to linearize around


        # initialize variables
        pitch_initial_rad = turbine.pitch_initial_rad
        TSR_initial = turbine.TSR_initial
        pitch_op    = np.empty(len(TSR_op))
        dCp_beta    = np.empty(len(TSR_op))
        dCp_TSR     = np.empty(len(TSR_op))
        dCt_beta    = np.empty(len(TSR_op))
        dCt_TSR     = np.empty(len(TSR_op))
        Ct_op       = np.empty(len(TSR_op))

        # ------------- Find Linearized State "Matrices" ------------- #
        # At each operating point
        for i in range(len(TSR_op)):

            if self.VS_FBP > 0: # Fixed blade pitch control in Region 3

                # Constant pitch, either user input or optimal pitch from Cp surface
                pitch_op[i] = self.min_pitch

            else: # Variable pitch control in Region 3 (default)

                # Find pitch angle as a function of expected operating CP for each TSR operating point
                Cp_TSR = np.ndarray.flatten(turbine.Cp.interp_surface(turbine.pitch_initial_rad, TSR_op[i]))     # all Cp values for a given tsr
                Cp_maxidx = Cp_TSR.argmax()    
                Cp_op[i] = np.clip(Cp_op[i], np.min(Cp_TSR[Cp_maxidx:]), np.max(Cp_TSR[Cp_maxidx:]))            # saturate Cp values to be on Cp surface                                                             # Find maximum Cp value for this TSR
                f_cp_pitch = interpolate.interp1d(Cp_TSR[Cp_maxidx:],pitch_initial_rad[Cp_maxidx:])             # interpolate function for Cp(tsr) values

                # expected operational blade pitch values. Saturates by min_pitch if it exists
                if v[i] <= turbine.v_rated: # Below rated, keep lowest pitch
                    pitch_op[i] = min(self.min_pitch, f_cp_pitch(Cp_op[i]))
                else:                       # above rated, keep highest pitch
                    pitch_op[i] = max(self.min_pitch, f_cp_pitch(Cp_op[i]))

            # Calculate Cp Surface gradients
            dCp_beta[i], dCp_TSR[i] = turbine.Cp.interp_gradient(pitch_op[i],TSR_op[i]) 
            dCt_beta[i], dCt_TSR[i] = turbine.Ct.interp_gradient(pitch_op[i],TSR_op[i]) 

            # Thrust
            Ct_TSR      = np.ndarray.flatten(turbine.Ct.interp_surface(turbine.pitch_initial_rad, TSR_op[i]))     # all Cp values for a given tsr
            f_ct        = interpolate.interp1d(pitch_initial_rad,Ct_TSR)
            Ct_op[i]    = f_ct(pitch_op[i])
            Ct_op[i]    = np.clip(Ct_op[i], np.min(Ct_TSR), np.max(Ct_TSR))        # saturate Ct values to be on Ct surface

        # Compute generator speed and torque operating schedule
        P_op = 0.5 * turbine.rho * np.pi*turbine.rotor_radius**2 * Cp_op * v**3 * turbine.GBoxEff/100 * turbine.GenEff/100
        if self.VS_FBP == 0: # Saturate between min speed and rated if variable pitch in Region 3
            omega_op = np.maximum(np.minimum(turbine.rated_rotor_speed, TSR_op*v/R), self.vs_minspd)
        else: # Only saturate min pitch if torque control in Region 3
            omega_op = np.maximum(TSR_op*v/R, self.vs_minspd)
        omega_gen_op = omega_op * Ng

        tau_op = P_op / omega_gen_op / (turbine.GenEff/100) # Includes increase to counteract generator efficiency loss, but not gearbox efficiency loss
        # Check if maximum torque leaves enough leeway to control the system
        if np.max(tau_op) > turbine.max_torque: # turbine.max_torque * 1.2 # DBS: Should we include additional margin? 
            print('WARNING: Torque operating schedule is above maximum generator torque and may not be realizable within saturation limits.')
            # DBS: Future - add input constraints to satisfy maximum torque, speed, and thrust (peak shaving) in addition to power

        # Check if options allow a nonmonotonic torque schedule
        if self.VS_FBP == 3:
            # The simulation will crash if we have a nonmonotonic schedule, so fail to generate the config and alert the user
            if np.any(np.diff(tau_op) <= 0):
                raise Exception("VS controller reference torque interpolation is selected (VS_FBP_ref_mode == 1), but computed generator torque schedule is not monotonically increasing. Reconfigure power curve, ensure VS_FBP_speed_mode == 0, or switch VS_FBP to 2.")


        # Full Cx surface gradients
        dCp_dbeta   = dCp_beta/np.diff(pitch_initial_rad)[0]
        dCp_dTSR    = dCp_TSR/np.diff(TSR_initial)[0]
        dCt_dbeta   = dCt_beta/np.diff(pitch_initial_rad)[0]
        dCt_dTSR    = dCt_TSR/np.diff(TSR_initial)[0]

        # Linearized system derivatives, equations from https://wes.copernicus.org/articles/7/53/2022/wes-7-53-2022.pdf
        dtau_dbeta      = Ng/2*rho*Ar*R*(1/TSR_op)*dCp_dbeta*v**2  # (26)
        dtau_dlambda    = Ng/2*rho*Ar*R*v**2*(1/(TSR_op**2))*(dCp_dTSR*TSR_op - Cp_op)   # (7)
        dlambda_domega  = R/v/Ng
        dtau_domega     = dtau_dlambda*dlambda_domega
        dlambda_dv      = -(TSR_op/v)

        Pi_beta         = 1/2 * rho * Ar * v**2 * dCt_dbeta
        Pi_omega        = 1/2 * rho * Ar * R * v * dCt_dTSR
        Pi_wind         = 1/2 * rho * Ar * v**2 * dCt_dTSR * dlambda_dv + rho * Ar * v * Ct_op

        # Second order system coefficients
        if not self.VS_ConstPower:       # Constant torque above rated
            A = dtau_domega/J
        else:                            # Constant power above rated
            A = dtau_domega/J 
            A[-len(v_above_rated)+1:] += Ng**2/J * turbine.rated_power/(Ng**2*rated_rotor_speed**2)
        B_tau = -Ng**2/J              # Torque input  
        B_beta = dtau_dbeta/J         # Blade pitch input 

        # Wind Disturbance Input
        dtau_dv = (0.5 * rho * Ar * 1/rated_rotor_speed) * (dCp_dTSR*dlambda_dv*v**3 + Cp_op*3*v**2) 
        B_wind = dtau_dv/J # wind speed input - currently unused 


        # separate and define below and above rated parameters
        A_vs = A[0:len(v_below_rated)]          # below rated
        A_pc = A[-len(v_above_rated)+1:]     # above rated
        B_tau = B_tau * np.ones(len(v))

        # Resample omega_ and zeta_pc at above rated wind speeds
        if not list_check(self.omega_pc) and not list_check(self.zeta_pc):
            self.omega_pc_U = self.omega_pc * np.ones(len(v_above_rated[1:]))
            self.zeta_pc_U = self.zeta_pc * np.ones(len(v_above_rated[1:]))
        else:
            if self.interp_type == 'sigma':  # sigma interpolation
                self.omega_pc_U = multi_sigma(v_above_rated[1:], self.U_pc, self.omega_pc)
                self.zeta_pc_U = multi_sigma(v_above_rated[1:], self.U_pc, self.zeta_pc)
            else:   # standard scipy interpolation types
                interp_omega = interpolate.interp1d(self.U_pc, self.omega_pc, kind=self.interp_type, bounds_error=False, fill_value='extrapolate')
                interp_zeta = interpolate.interp1d(self.U_pc, self.zeta_pc, kind=self.interp_type, bounds_error=False, fill_value='extrapolate')
                self.omega_pc_U = interp_omega(v_above_rated[1:])
                self.zeta_pc_U = interp_zeta(v_above_rated[1:])

        # -- Find gain schedule --
        self.pc_gain_schedule = ControllerTypes()
        self.pc_gain_schedule.second_order_PI(self.zeta_pc_U, self.omega_pc_U,A_pc,B_beta[-len(v_above_rated)+1:],linearize=True,v=v_above_rated[1:])        
        self.vs_gain_schedule = ControllerTypes()
        if self.VS_FBP == 0:
            # Using variable pitch control in Region 3, so generate torque gain schedule only for Region 2
            self.vs_gain_schedule.second_order_PI(self.zeta_vs, self.omega_vs,A_vs,B_tau[0:len(v_below_rated)],linearize=False,v=v_below_rated)
        else:
            # Using fixed pitch torque control in Region 3, so generate torque gain schedule for Regions 2 and 3
            self.vs_gain_schedule.second_order_PI(self.zeta_vs, self.omega_vs,A,B_tau,linearize=False,v=v)

        # -- Find K for Komega_g^2 --
        # Careful handling of different efficiencies
        # P_lss = 1/2 * Cp * rho * pi * R^5 / (TSR^3 * Ng^3)
        # P_hss = GBoxEff * P_lss = tau_gen * omega_gen = K * omega_gen^3
        # P_gen = GenEff * P_hss
        # Generator efficiency is not included in K here, but gearbox efficiency is
        # Note that this differs from the
        self.vs_rgn2K = (pi*rho*R**5.0 * turbine.Cp.max * turbine.GBoxEff/100) / (2.0 * turbine.Cp.TSR_opt**3 * Ng**3) * self.controller_params['rgn2k_factor']
        self.vs_refspd = min(turbine.TSR_operational * turbine.v_rated/R, turbine.rated_rotor_speed) * Ng

        # -- Define some setpoints --
        # minimum rotor speed saturation limits
        if self.vs_minspd:
            self.vs_minspd = np.maximum(self.vs_minspd, (turbine.TSR_operational * turbine.v_min / turbine.rotor_radius))
        else: 
            self.vs_minspd = (turbine.TSR_operational * turbine.v_min / turbine.rotor_radius)
        self.pc_minspd = self.vs_minspd

        # Set IPC ramp inputs if not already defined
        if max(self.IPC_Vramp) == 0.0:
            self.IPC_Vramp = [turbine.v_rated*0.8, turbine.v_rated]

        # Store some variables
        self.v              = v                                  # Wind speed (m/s)
        self.v_above_rated  = v_above_rated
        self.v_below_rated  = v_below_rated
        # Mod by A. Wright
        self.v_for_gs       = v[-len(v_above_rated)+1:]
		# end
        self.pitch_op       = pitch_op
        self.pitch_op_pc    = pitch_op[-len(v_above_rated)+1:]
        self.TSR_op         = TSR_op
        self.A              = A 
        self.B_beta         = B_beta
        self.B_tau          = B_tau
        self.B_wind         = B_wind
        self.omega_op       = omega_op
        self.omega_gen_op   = omega_gen_op
        self.tau_op         = tau_op
        self.power_op       = P_op
        self.Pi_omega       = Pi_omega
        self.Pi_beta        = Pi_beta
        self.Pi_wind        = Pi_wind

        # - Might want these to debug -
        self.Cp_op = Cp_op

        # --- Minimum pitch saturation ---
        self.ps_min_bld_pitch = np.ones(len(self.pitch_op)) * self.min_pitch
        self.ps = ControllerBlocks()

        if self.PS_Mode == 1:  # Peak Shaving
            self.ps.peak_shaving(self, turbine)
        elif self.PS_Mode == 2: # Cp-maximizing minimum pitch saturation
            self.ps.min_pitch_saturation(self,turbine)
        elif self.PS_Mode == 3: # Peak shaving and Cp-maximizing minimum pitch saturation
            self.ps.peak_shaving(self, turbine)
            self.ps.min_pitch_saturation(self,turbine)

        # --- Power control ---
        
        PRC_Table_n = self.controller_params['DISCON']['PRC_Table_n']
        pitch_range = np.linspace(self.min_pitch,self.max_pitch, num=100)  # radians
        
        # Cp at optimal TSR
        Cp_TSR_opt = turbine.Cp.interp_surface(pitch_range,turbine.Cp.TSR_opt)
        
        # Solve for pitch that decreases Cp by R_range using linear interpolation
        R_range = np.linspace(0,1,num=PRC_Table_n)
        Cp_R = R_range * turbine.Cp.max
        pitch_R = np.interp(Cp_R,np.flip(Cp_TSR_opt),np.flip(pitch_range))  # need to flip because interp wants xp to be increasing
        
        # Save values back to DISCON dict
        self.controller_params['DISCON']['PRC_R_Table'] = R_range
        self.controller_params['DISCON']['PRC_Pitch_Table'] = pitch_R

        # --- Floating feedback term ---

        if self.Fl_Mode >= 1: # Floating feedback

            # Wind speed gain scheduling
            self.U_Fl = self.controller_params['U_Fl']
            if self.U_Fl:  # default is [], only have one Fl_Kp
                if type(self.U_Fl) == str:
                    if self.U_Fl == 'all':  
                        # Mod by A. Wright: get the array of Kp_float values at the values of v-above rated (see self.v_for_gs calculated around line 344).
                        self.U_Fl = self.v_for_gs
                    else:
                        raise Exception("Invalid entry in controller_params for U_Fl, please see schema")
            else:
                self.U_Fl = np.array([turbine.v_rated * (1.05)])


            # If we haven't set Kp_float as a control parameter, we tune it automatically here
            if self.tune_Fl:
                Kp_float = (dtau_dv/dtau_dbeta) * Ng 
                if self.Fl_Mode == 2:
                    Kp_float *= turbine.TowerHt      
                f_kp     = interpolate.interp1d(v,Kp_float)
                self.Kp_float = f_kp(self.U_Fl)   # get Kp at v_rated + 0.5 m/s

            # Make arrays if not
            if not np.shape(self.Kp_float):
                self.Kp_float = np.array([self.Kp_float])
            if not np.shape(self.U_Fl):
                self.U_Fl = np.array([self.U_Fl])
            
            # Check size of Kp_float and U_Fl
            if len(self.Kp_float) != len(self.U_Fl):
                raise Exception('The sizes of Kp_float and U_Fl are not equal, please check your controller_params')



            # Turn on the notch filter if floating and not already on
            if not self.F_NotchType:
                self.F_NotchType = 2      
                      
            # And check for .yaml input inconsistencies
            if self.twr_freq == 0.0 or self.ptfm_freq == 0.0:
                print('WARNING: twr_freq and ptfm_freq should be defined for floating turbine control!!')
            
        else:
            self.Kp_float = np.array([0.0])
            self.U_Fl = np.array([0.0])
        
        # Flap actuation 
        if self.Flp_Mode >= 1:
            self.flp_angle = 0.0
            try:
                self.tune_flap_controller(turbine)
            except AttributeError:
                print('ERROR: If Flp_Mode > 0, you need to have blade information loaded in the turbine object.')
                raise
            except UnboundLocalError:
                print('ERROR: You are attempting to tune a flap controller for a blade without flaps!')
                raise
        else:
            self.flp_angle = 0.0
            self.Ki_flap = np.array([0.0])
            self.Kp_flap = np.array([0.0])

        # --- Set up filters ---
        self.f_lpf_cornerfreq = turbine.bld_edgewise_freq / 4

        # Notch filters
        self.f_notch_freqs = []
        self.f_notch_beta_nums = []
        self.f_notch_beta_dens = []
        self.f_notch_gen_inds = []
        self.f_notch_twr_inds = []

        if self.F_NotchType:
            if self.Flp_Mode:
                self.f_notch_freqs.append(turbine.bld_flapwise_freq)
                self.f_notch_beta_nums.append(0.0)
                self.f_notch_beta_dens.append(0.50)
            else:
                self.f_notch_freqs.append(self.twr_freq)
                self.f_notch_beta_nums.append(0.0)
                self.f_notch_beta_dens.append(0.25)

            if self.F_NotchType == 1 or self.F_NotchType == 3:
                self.f_notch_gen_inds.append(1)
            elif self.F_NotchType == 2:
                self.f_notch_twr_inds.append(1)


        # --- Direct input passthrough ---
        if 'f_lpf_cornerfreq' in self.controller_params['filter_params']:
            self.f_lpf_cornerfreq = self.controller_params['filter_params']['f_lpf_cornerfreq']

    
    def tune_flap_controller(self,turbine):
        '''
        Tune controller for distributed aerodynamic control

        Parameters:
        -----------
        turbine : class
                  Turbine class containing necessary turbine information to accurately tune the controller. 
        '''
        # Find blade aerodynamic coefficients
        v_rel = []
        phi_vec = []
        for i, _ in enumerate(self.v):
            turbine.cc_rotor.induction_inflow=True
            # Axial and tangential inductions
            try: 
                a, ap, _, _, _ = turbine.cc_rotor.distributedAeroLoads(
                                                self.v[i], self.omega_op[i], self.pitch_op[i], 0.0)
            except ValueError:
                loads, _ = turbine.cc_rotor.distributedAeroLoads(
                                                self.v[i], self.omega_op[i], self.pitch_op[i], 0.0)
                a = loads['a']      # Axial induction factor
                ap = loads['ap']    # Tangential induction factor
                 
            # Relative windspeed along blade span
            v_rel.append([np.sqrt(self.v[i]**2*(1-a)**2 + self.omega_op[i]**2*turbine.span**2*(1-ap)**2)])
            # Inflow wind direction
            phi_vec.append(self.pitch_op[i] + turbine.twist*deg2rad)

        # Lift and drag coefficients
        num_af = len(turbine.af_data) # number of airfoils
        Cl0 = np.zeros(num_af)
        Cd0 = np.zeros(num_af)
        Clp = np.zeros(num_af)
        Cdp = np.zeros(num_af)
        Clm = np.zeros(num_af)
        Cdm = np.zeros(num_af)
        
        for i,section in enumerate(turbine.af_data):
            # assume airfoil section as AOA of zero for slope calculations
            a0_ind = section[0]['Alpha'].index(np.min(np.abs(section[0]['Alpha'])))
            # Coefficients 
            #  - If the flap exists in this blade section, define Cx-plus,-minus,-neutral(0)
            #  - IF teh flap does not exist in this blade section, Cx matrix is all the same value
            if section[0]['NumTabs'] == 3:  # sections with 3 flaps
                Clm[i,] = section[0]['Cl'][a0_ind]
                Cdm[i,] = section[0]['Cd'][a0_ind]
                Cl0[i,] = section[1]['Cl'][a0_ind]
                Cd0[i,] = section[1]['Cd'][a0_ind]
                Clp[i,] = section[2]['Cl'][a0_ind]
                Cdp[i,] = section[2]['Cd'][a0_ind]
                Ctrl_flp = float(section[2]['UserProp'])
            else:                           # sections without 3 flaps
                Cl0[i,] = Clp[i,] = Clm[i,] = section[0]['Cl'][a0_ind]
                Cd0[i,] = Cdp[i,] = Cdm[i,] = section[0]['Cd'][a0_ind]
                Ctrl = float(section[0]['UserProp'])

        # Find lift and drag coefficient slopes w.r.t. flap angle
        Kcl = (Clp - Cl0)/( (Ctrl_flp-Ctrl)*deg2rad )
        Kcd = (Cdp - Cd0)/( (Ctrl_flp-Ctrl)*deg2rad )

        # Find integrated constants
        self.kappa = np.zeros(len(v_rel))  # "flap efficacy term"
        C1 = np.zeros(len(v_rel))
        C2 = np.zeros(len(v_rel))
        for i, (v_sec,phi) in enumerate(zip(v_rel, phi_vec)):
            C1[i] = integrate.trapezoid(0.5 * turbine.rho * turbine.chord * v_sec[0]**2 * turbine.span * Kcl * np.cos(phi))
            C2[i] = integrate.trapezoid(0.5 * turbine.rho * turbine.chord * v_sec[0]**2 * turbine.span * Kcd * np.sin(phi))
            self.kappa[i]=C1[i]+C2[i]

        # PI Gains
        if (self.flp_kp_norm == 0 or self.flp_tau == 0) or (not self.flp_kp_norm or not self.flp_tau):
            raise ValueError('flp_kp_norm and flp_tau must be nonzero for Flp_Mode >= 1')
        self.Kp_flap = self.flp_kp_norm / self.kappa
        self.Ki_flap = self.flp_kp_norm / self.kappa / self.flp_tau

class ControllerBlocks():
    '''
    Class ControllerBlocks defines tuning parameters for additional controller features or "blocks"

    Methods:
    --------
    peak_shaving

    '''
    def __init__(self):
        pass
    
    def peak_shaving(self,controller, turbine):
        ''' 
        Define minimum blade pitch angle for peak shaving routine based on a maximum allowable thrust 

        Parameters:
        -----------
        controller: class
                    Controller class containing controller operational information
        turbine: class
                 Turbine class containing necessary wind turbine information for controller tuning
        '''

        # Re-define Turbine Parameters for shorthand
        rho = turbine.rho             # Air density (kg/m^3)
        R = turbine.rotor_radius      # Rotor radius (m)
        A = np.pi*R**2                # Rotor area (m^2)

        # Initialize some arrays
        Ct_op = np.empty(len(controller.TSR_op),dtype='float64')
        Ct_max = np.empty(len(controller.TSR_op),dtype='float64')

        # Find unshaved rotor thrust coefficients at each TSR
        for i in range(len(controller.TSR_op)):
            Ct_op[i] = turbine.Ct.interp_surface(controller.pitch_op[i],controller.TSR_op[i])

        # Thrust vs. wind speed    
        T = 0.5 * rho * A * controller.v**2 * Ct_op

        # Define minimum max thrust and initialize pitch_min
        Tmax = controller.ps_percent * np.max(T)
        pitch_min = np.ones(len(controller.pitch_op)) * controller.min_pitch

        # Modify pitch_min if max thrust exceeds limits
        for i in range(len(controller.TSR_op)):
            # Find Ct values for operational TSR
            Ct_tsr = turbine.Ct.interp_surface(turbine.pitch_initial_rad,controller.TSR_op[i])
            # Define max Ct values
            Ct_max[i] = Tmax/(0.5 * rho * A * controller.v[i]**2)
            if T[i] > Tmax:
                Ct_op[i] = Ct_max[i]
            else:
                # TSR_below_rated = np.minimum(np.max(TSR_above_rated), TSR_below_rated)
                Ct_max[i] = np.minimum( np.max(Ct_tsr), Ct_max[i])

            # Define minimum pitch angle
            # - find min(\beta) so that Ct <= Ct_max and \beta > \beta_fine at each operational TSR
            f_pitch_min = interpolate.interp1d(Ct_tsr, turbine.pitch_initial_rad, kind='linear', bounds_error=False, fill_value=(turbine.pitch_initial_rad[0],turbine.pitch_initial_rad[-1]))
            pitch_min[i] = max(controller.min_pitch, f_pitch_min(Ct_max[i]))

        # Save to controller object
        controller.ps_min_bld_pitch = pitch_min

        # save some outputs for analysis or future work
        self.Tshaved = 0.5 * rho * A * controller.v**2 * Ct_op
        self.pitch_min = pitch_min
        self.v = controller.v
        self.Ct_max = Ct_max
        self.Ct_op = Ct_op
        self.T = T

    def min_pitch_saturation(self, controller, turbine):
        '''
        Minimum pitch saturation in low wind speeds to maximize power capture

        Parameters:
        -----------
        controller: class
                    Controller class containing controller operational information
        turbine: class
                 Turbine class containing necessary wind turbine information for controller tuning
        '''
        # Find TSR associated with minimum rotor speed
        TSR_at_minspeed = (controller.pc_minspd) * turbine.rotor_radius / controller.v_below_rated
        
        # For each below rated wind speed operating point
        for i in range(len(TSR_at_minspeed)):
            if TSR_at_minspeed[i] > controller.TSR_op[i]:
                controller.TSR_op[i] = TSR_at_minspeed[i]
        
                # Initialize some arrays
                Cp_op = np.empty(len(turbine.pitch_initial_rad),dtype='float64')
                min_pitch = np.empty(len(TSR_at_minspeed),dtype='float64')
        
                # ------- Find Cp-maximizing minimum pitch schedule ---------
                # Cp coefficients at below-rated tip speed ratios
                Cp_op = turbine.Cp.interp_surface(turbine.pitch_initial_rad,TSR_at_minspeed[i])

                # Setup and run small optimization problem to find blade pitch angle that maximizes Cp at a given TSR
                # - Finds \beta to satisfy max( Cp(\beta,TSR_op) )
                f_pitch_min = interpolate.interp1d(turbine.pitch_initial_rad, -Cp_op, kind='quadratic', bounds_error=False, fill_value=(turbine.pitch_initial_rad[0],turbine.pitch_initial_rad[-1]))
                res = optimize.minimize(f_pitch_min, 0.0)
                min_pitch[i] = res.x[0]
                
                # modify existing minimum pitch schedule
                controller.ps_min_bld_pitch[i] = np.maximum(controller.ps_min_bld_pitch[i], min_pitch[i])

                # Save Cp_op
                controller.Cp_op[i] = -res.fun
            else:
                return


class ControllerTypes():
    '''
    Class ControllerTypes used to define any types of controllers that can be tuned. 
        Generally, calculates gains based on some pre-defined tuning parameters. 

    Methods:
    --------
    second_order_PI
    '''
    def __init__(self):
        pass

    def second_order_PI(self,zeta,om_n,A,B,linearize=False,v=None):
        '''
        Define proportional integral gain schedule for a closed
            loop system with a standard second-order form.

        Parameters:
        -----------
        zeta : list of floats (-)
               Desired damping ratio with breakpoints at v
        om_n : list of floats (rad/s)
               Desired natural frequency with breakpoints at v
        A : array_like (1/s)
            Plant poles (state transition matrix)
        B : array_like (varies)
            Plant numerators (input matrix)
        linearize : bool, optional
                    If 'True', find a gain scheduled based on a linearized plant.
        v : array_like (m/s)
            Wind speeds for linearized plant model, if desired. 
        '''
        # Linearize system coefficients w.r.t. wind speed if desired
        if linearize:
            pA = np.polyfit(v,A,1)
            pB = np.polyfit(v,B,1)
            A = pA[0]*v + pA[1]
            B = pB[0]*v + pB[1]

        # Calculate gain schedule
        self.Kp = 1/B * (2*zeta*om_n + A)
        self.Ki = om_n**2/B           

class OpenLoopControl(object):
    '''
    Open loop controls for
        - blade_pitch
        - generator_torque
        - nacelle_yaw and nacelle_yaw_rate

    Please see Examples/example_14.py for an example on how to use this class.

    '''

    def __init__(self, **kwargs):
        self.dt         = 0.05 # sec
        self.t_max      = 200 # sec
        self.breakpoint = 'time' # or wind_speed
        self.du         = 0.5 # m/s
        self.u_min      = 5  # m/s
        self.u_max      = 30  # m/s

        self.allowed_controls = [
            'blade_pitch',
            'generator_torque',
            'nacelle_yaw',
            'nacelle_yaw_rate',
            'cable_control',
            'struct_control',
            'R_speed',
            'R_torque',
            'R_pitch',
            ]
        self.allowed_breakpoints = ['time','wind_speed']

        # Optional population class attributes from key word arguments
        for (k, w) in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        self.ol_series = {}
        if self.breakpoint == 'time':
            self.ol_series[self.breakpoint] = np.arange(0,self.t_max+self.dt,self.dt)
        elif self.breakpoint == 'wind_speed':
            self.ol_series[self.breakpoint] = np.arange(self.u_min,self.u_max+self.du,self.du)
        else:
            raise Exception(f'Breakpoint of {self.breakpoint} is not allowed.  Available options are {self.allowed_breakpoints}.')

        
    def const_timeseries(self,control,value):
        if control == 'nacelle_yaw':
            raise Exception('Open loop control of nacelle_yaw will not work.  Only the yaw rate is controlled from OpenFAST.  Please change NacYaw in ElastoDyn and YawNeut in ServoDyn.')
        
        self.ol_series[control] = value * np.ones(len(self.ol_series[self.breakpoint]))
        

    def interp_series(self,control,breakpoints,values,method='sigma'):

        # Error checking
        if not list_check(breakpoints) or len(breakpoints) == 1:
            raise Exception('Open loop breakpoints are not a list with length greater than 1')

        if not list_check(values) or len(values) == 1:
            raise Exception('Open loop values are not a list with length greater than 1')

        if len(breakpoints) != len(values):
            raise Exception('Open loop breakpoints and values do not have the same length')

        # Check if control in allowed controls, cable_control_* is in cable_control
        if not any([ac in control for ac in self.allowed_controls]):
            raise Exception(f'Open loop control of {control} is not allowed')
        
        # Check that breakpoints are valid
        if self.breakpoint == 'wind_speed':
            if max(breakpoints) > self.u_max:
                raise Exception(f'The maximum desired wind speed breakpoint ({max(breakpoints)}) is greater than u_max ({self.u_max})')
        elif self.breakpoint == 'time':
            if max(breakpoints) > self.t_max: 
                raise Exception(f'The maximum desired time breakpoint ({max(breakpoints)}) is greater than t_max ({self.t_max})')

        # Finally interpolate
        if method == 'sigma':
            self.ol_series[control] = multi_sigma(self.ol_series[self.breakpoint],breakpoints,values)
        
        elif method == 'linear':
            self.ol_series[control] = np.interp(self.ol_series[self.breakpoint],breakpoints,values,left=values[0],right=values[-1])

        elif method == 'cubic':
            interp_fcn = interpolate.Akima1DInterpolator(breakpoints,values,method='akima',extrapolate=True) #,kind='cubic',fill_value=values[-1],bounds_error=False)
            self.ol_series[control] = interp_fcn(self.ol_series[self.breakpoint])

        else:
            raise Exception(f'Open loop interpolation method {method} not supported')

        if control == 'nacelle_yaw':
            self.compute_yaw_rate()


    def sine_timeseries(self,control,amplitude,period,offset=0):
        
        if period <= 0:
            raise Exception('Open loop sine input period is <= 0')
        
        if self.breakpoint != 'time':
            raise Exception(f'Only time breakpoints are allowed for sine timeseries')

        if control not in self.allowed_controls:
            raise Exception(f'Open loop control of {control} is not allowed')
        else:
            self.ol_series[control] = amplitude * np.sin(2 * np.pi *  self.ol_series['time'] / period) + offset

        if control == 'nacelle_yaw':
            self.compute_yaw_rate()

    def compute_yaw_rate(self):
        self.ol_series['nacelle_yaw_rate'] = np.concatenate(([0],np.diff(self.ol_series['nacelle_yaw'])))/self.dt

    def plot_series(self):
        '''
        Debugging script for showing open loop timeseries
        '''
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(len(self.ol_series)-1,1)
        i_ax = -1
        for ol_input in self.ol_series:
            if ol_input not in ['time','wind_speed']:
                i_ax += 1
                if len(self.ol_series)-1 == 1:
                    ax.plot(self.ol_series[self.breakpoint],self.ol_series[ol_input])
                    ax.set_ylabel(ol_input)
                else:
                    ax[i_ax].plot(self.ol_series[self.breakpoint],self.ol_series[ol_input])
                    ax[i_ax].set_ylabel(ol_input)
        return fig, ax

    def write_input(self,ol_filename):
        ''' 
        Write open loop control input
        Return open_loop dict for control params
        '''

        ol_series = self.ol_series

        # Init indices
        Ind_Breakpoint = 1
        Ind_Azimuth = Ind_GenTq = Ind_YawRate = Ind_R_Speed = Ind_R_Torque = Ind_R_Pitch = 0
        Ind_BldPitch = 3*[0]
        Ind_CableControl = []
        Ind_StructControl = []

        ol_index_counter = 0   # start input index at 2

        # Write breakpoint first, initialize OL matrix
        if 'time' in ol_series:
            ol_input_matrix = ol_series['time']
        elif 'wind_speed' in ol_series:
            ol_input_matrix = ol_series['wind_speed']
        else:
            raise Exception('WARNING: no time or wind-speed index for open loop control.  These are the only indices currently supported')
        
        if 'time' in ol_series and 'wind_speed' in ol_series:
            raise Exception('WARNING: both time and wind_speed are in the OL input setup.  Please check.')

        for channel in ol_series:

            # increment index counter first for 1-indexing in input file
            ol_index_counter += 1

            # skip writing for certain channels, if already in ol_input_matrix
            skip_write = False
            
            # Set open loop index based on name
            if channel == 'time':
                Ind_Breakpoint = ol_index_counter
                skip_write = True
            elif channel == 'wind_speed':
                Ind_Breakpoint = ol_index_counter
                skip_write = True   
            elif channel == 'blade_pitch':  # collective blade pitch
                Ind_BldPitch = 3 * [ol_index_counter]
            elif channel == 'generator_torque':
                Ind_GenTq = ol_index_counter
            elif channel == 'nacelle_yaw_rate':
                Ind_YawRate = ol_index_counter
            elif channel == 'nacelle_yaw':
                ol_index_counter -= 1  # don't increment counter
                skip_write = True
            elif channel == 'blade_pitch1':
                Ind_BldPitch[0] = ol_index_counter
            elif channel == 'blade_pitch2':
                Ind_BldPitch[1] = ol_index_counter
            elif channel == 'blade_pitch3':
                Ind_BldPitch[2] = ol_index_counter
            elif channel == 'azimuth':
                Ind_Azimuth = ol_index_counter
            elif channel == 'R_speed':
                Ind_R_Speed = ol_index_counter
            elif channel == 'R_torque':
                Ind_R_Torque = ol_index_counter
            elif channel == 'R_pitch':
                Ind_R_Pitch = ol_index_counter
            elif 'cable_control' in channel or 'struct_control' in channel:
                skip_write = True
                ol_index_counter -= 1  # don't increment counter



            # append open loop input array for non-ptfm channels
            if not skip_write:
                ol_input_matrix = np.c_[ol_input_matrix,ol_series[channel]]

        ol_index_counter += 1  # Increment counter so it's 1 more than time, just like above in each iteration

        # Cable control
        is_cable_chan = np.array(['cable_control' in ol_chan for ol_chan in ol_series.keys()])
        if any(is_cable_chan):
            # if any channels are cable_control_*
            n_cable_chan = np.sum(is_cable_chan)
            cable_chan_names = np.array(list(ol_series.keys()))[is_cable_chan]

            # Let's assume they are 1-indexed and all there, otherwise a key error will be thrown
            for cable_chan in cable_chan_names:
                ol_input_matrix = np.c_[ol_input_matrix,ol_series[cable_chan]]
                Ind_CableControl.append(ol_index_counter)
                ol_index_counter += 1

        # Struct control
        is_struct_chan = ['struct_control' in ol_chan for ol_chan in ol_series.keys()]
        if any(is_struct_chan):
            # if any channels are struct_control_*
            n_struct_chan = np.sum(np.array(is_struct_chan))

            # Let's assume they are 1-indexed and all there, otherwise a key error will be thrown
            for i_chan in range(1,n_struct_chan+1):
                ol_input_matrix = np.c_[ol_input_matrix,ol_series[f'struct_control_{i_chan}']]
                Ind_StructControl.append(ol_index_counter)
                ol_index_counter += 1


        # Open file
        if not os.path.exists(os.path.dirname(os.path.abspath(ol_filename))):
            os.makedirs(os.path.dirname(os.path.abspath(ol_filename)))
        
        with open(ol_filename,'w') as f:
            # Write header
            headers = [''] * ol_index_counter
            units = [''] * ol_index_counter

            if self.breakpoint == 'time':
                header_line = '!\tTime' # TODO: not sure if we need header_line and headers, check struct control
                unit_line   = '!\t(sec.)'

                headers[0] = 'Time'
                units[0] = 'sec.'
            elif self.breakpoint == 'wind_speed':
                header_line = '!\tWindSpeed' # TODO: not sure if we need header_line and headers, check struct control
                unit_line   = '!\t(m/s)'

                headers[0] = 'WindSpeed'
                units[0] = 'm/s'
            

            if Ind_GenTq:
                headers[Ind_GenTq-1] = 'GenTq'
                units[Ind_GenTq-1] = '(Nm)'

            if Ind_YawRate:
                headers[Ind_YawRate-1] = 'YawRate'
                units[Ind_YawRate-1] = '(rad/s)'

            if Ind_Azimuth:
                headers[Ind_Azimuth-1] = 'Azimuth'
                units[Ind_Azimuth-1] = '(rad)'

            if any(Ind_BldPitch):
                if all_same(Ind_BldPitch):
                    headers[Ind_BldPitch[0]-1] = 'BldPitch123'
                    units[Ind_BldPitch[0]-1] = '(rad)'
                else:
                    headers[Ind_BldPitch[0]-1] = 'BldPitch1'
                    units[Ind_BldPitch[0]-1] = '(rad)'
                    headers[Ind_BldPitch[1]-1] = 'BldPitch2'
                    units[Ind_BldPitch[1]-1] = '(rad)'
                    headers[Ind_BldPitch[2]-1] = 'BldPitch3'
                    units[Ind_BldPitch[2]-1] = '(rad)'

            if Ind_CableControl:
                for i_chan in range(1,n_cable_chan+1):
                    header_line += f'\t\tCable{i_chan}'
                    unit_line   += '\t\t(m)'
            else:
                Ind_CableControl = [0]

            if Ind_StructControl:
                for i_chan in range(1,n_struct_chan+1):
                    header_line += f'\t\tStruct{i_chan}'
                    unit_line   += '\t\t(m)'
            else:
                Ind_StructControl = [0]

            if Ind_R_Speed:
                headers[Ind_R_Speed-1] = 'R_speed'
                units[Ind_R_Speed-1] = '(-)'

            if Ind_R_Torque:
                headers[Ind_R_Torque-1] = 'R_Torque'
                units[Ind_R_Torque-1] = '(-)'

            if Ind_R_Pitch:
                headers[Ind_R_Pitch-1] = 'R_Pitch'
                units[Ind_R_Pitch-1] = '(-)'

            # Join headers and units
            header_line = '!' + '\t\t'.join(headers) + '\n'
            unit_line = '!' + '\t\t'.join(units) + '\n'

            f.write(header_line)
            f.write(unit_line)

            # Write lines
            for ol_line in ol_input_matrix:
                line = ''.join(['{:<10.8f}\t'.format(val) for val in ol_line]) + '\n'
                f.write(line)

        # Output open_loop dict for control params
        open_loop = {}
        open_loop['filename']           = ol_filename
        open_loop['Ind_Breakpoint']  = Ind_Breakpoint
        open_loop['Ind_BldPitch']    = Ind_BldPitch
        open_loop['Ind_GenTq']       = Ind_GenTq
        open_loop['Ind_YawRate']     = Ind_YawRate
        open_loop['Ind_Azimuth']     = Ind_Azimuth
        open_loop['Ind_CableControl']     = Ind_CableControl
        open_loop['Ind_StructControl']    = Ind_StructControl
        open_loop['Ind_R_Speed']     = Ind_R_Speed
        open_loop['Ind_R_Torque']    = Ind_R_Torque
        open_loop['Ind_R_Pitch']     = Ind_R_Pitch
        if self.breakpoint == 'time':
            open_loop['OL_BP_Mode'] = 0
        elif self.breakpoint == 'wind_speed':
            open_loop['OL_BP_Mode'] = 1



        return open_loop


# ----------- Helper Functions -----------
def sigma(tt,t0,t1,y0=0,y1=1):
    ''' 
    generates timeseries for a smooth transition from y0 to y1 from x0 to x1

    Parameters:
    -----------
    tt: List-like
        time indices
    t0: float
        start time
    t1: float
        end time
    y0: float
        start output
    y1: 
        end output

    Returns:
    --------
    yy: List-like
        output timeseries corresponding to tt
    '''

    a3 = 2/(t0-t1)**3
    a2 = -3*(t0+t1)/(t0-t1)**3
    a1 = 6*t1*t0/(t0-t1)**3
    a0 = (t0-3*t1)*t0**2/(t0-t1)**3

    a = np.array([a3,a2,a1,a0])  

    T = np.vander(tt,N=4)       # vandermonde matrix

    ss = T @ a.T                # base sigma

    yy = (y1-y0) * ss + y0      # scale and offset

    return yy


def multi_sigma(xx,x_bp,y_bp):
    '''
    Make a sigma interpolation with multiple breakpoints

    Parameters:
    -----------
    xx: list of floats (-)
            new sample points
    x_bp: list of floats (-)
            breakpoints
    y_bp : list of floats (-)
            function value at breakpoints

    Returns:
    --------
    yy: List-like
        output timeseries corresponding to tt
    '''
    # initialize
    yy = np.zeros_like(xx)

    # interpolate sigma functions between all breakpoints
    for i_sigma in range(0,len(x_bp)-1):
        ind_i       = (xx >= x_bp[i_sigma]) & (xx < x_bp[i_sigma+1])
        xx_i        = xx[ind_i]
        yy_i        = sigma(xx_i,x_bp[i_sigma],x_bp[i_sigma+1],y0=y_bp[i_sigma],y1=y_bp[i_sigma+1])
        yy[ind_i]   = yy_i

    # add first and last values to beginning and end
    yy[xx<x_bp[0]]      = y_bp[0]
    yy[xx>=x_bp[-1]]     = y_bp[-1]

    if False:  # debug plot
        import matplotlib.pyplot as plt
        plt.plot(xx,yy)
        plt.show()

    return yy

def all_same(items):
    return all(x == items[0] for x in items)
