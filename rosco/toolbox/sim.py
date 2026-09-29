# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# speROSCO_cific language governing permissions and limitations under the License.

import numpy as np
from rosco.toolbox import turbine as ROSCO_turbine
import matplotlib.pyplot as plt
import sys

# Some useful constants
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0*(np.pi)/60.0


class Sim():
    """
    Simple controller simulation interface for a wind turbine.
     - Currently runs a 1DOF simple rotor model based on an OpenFAST model

    Note: Due to the complex nature of the wind speed estimators implemented in ROSCO, 
    using them for simulations is known to cause problems for the simple simulator. 
    We suggesting using WE_Mode = 0 for the simulator


    Methods:
    --------
    sim_ws_series

    Parameters:
    -----------
    turbine: class
             Turbine class containing wind turbine information from OpenFAST model
    controller_int: class
                    Controller interface class to run compiled controller binary
    """

    def __init__(self, turbine, controller_int):
        """
        Setup the simulator
        """
        self.turbine = turbine
        self.controller_int = controller_int

    def sim_ws_series(self, t_array, ws_array, rotor_rpm_init=10, init_pitch=0.0,
                      wd_array=None, yaw_init=0.0,
                      make_plots=True):
        '''
        Simulate simplified turbine model using a complied controller (.dll or similar).
            - currently a 1DOF rotor model

        Parameters:
        -----------
            t_array: list-like
                     Array of time steps, (s)
            ws_array: list-like
                      Array of wind speeds, (m/s)
            wd_array: list-like
                      Array of wind directions, (rad)
            yaw_init: float
                      Initial "north", (or constant) yaw angle, (rad)
            rotor_rpm_init: float, optional
                            initial rotor speed, (rpm)
            init_pitch: float, optional
                        initial blade pitch angle, (deg)
            make_plots: bool, optional
                        True: generate plots, False: don't. 
        '''

        # Store turbine data for convenience
        dt = t_array[1] - t_array[0]
        R = self.turbine.rotor_radius
        GBRatio = self.turbine.Ng

        # Declare output arrays
        bld_pitch = np.ones_like(t_array) * init_pitch * deg2rad
        rot_speed = np.ones_like(t_array) * rotor_rpm_init * \
            rpm2RadSec  # represent rot speed in rad / s
        gen_speed = np.ones_like(t_array) * rotor_rpm_init * GBRatio * \
            rpm2RadSec  # represent gen speed in rad/s
        aero_torque = np.ones_like(t_array) * 1000.0
        gen_torque = np.ones_like(t_array)
        gen_power = np.ones_like(t_array) * 0.0
        nac_yaw = np.ones_like(t_array) * yaw_init
        nac_yawerr = np.ones_like(t_array) * 0.0
        nac_yawrate = np.ones_like(t_array) * 0.0

        # check for wind direction array
        if isinstance(wd_array, (list, np.ndarray)):
            if len(ws_array) != len(wd_array):
                raise ValueError('ws_array and wd_array must be the same length')
        else:
            wd_array = np.ones_like(ws_array) * 0.0

        # Loop through time
        for i, t in enumerate(t_array):
            if i == 0:
                continue  # Skip the first run
            ws = ws_array[i]
            wd = wd_array[i]

            # Load current Cq data
            tsr = rot_speed[i-1] * self.turbine.rotor_radius / ws
            cq = self.turbine.Cq.interp_surface(bld_pitch[i-1], tsr)
            cp = self.turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
            # Update the turbine state
            #       -- 1DOF model: rotor speed and generator speed (scaled by Ng)
            aero_torque[i] = 0.5 * self.turbine.rho * (np.pi * R**3) * (cp/tsr) * ws**2
            rot_speed[i] = rot_speed[i-1] + (dt/self.turbine.J)*(aero_torque[i] - self.turbine.Ng * gen_torque[i-1] / (self.turbine.GBoxEff/100))
            gen_speed[i] = rot_speed[i] * self.turbine.Ng
            #       -- Simple nacelle model
            nac_yawerr[i] = wd - nac_yaw[i-1]

            # populate turbine state dictionary
            turbine_state = {}
            if i < len(t_array)-1:
                turbine_state['iStatus'] = 1
            else:
                turbine_state['iStatus'] = -1
            turbine_state['t'] = t
            turbine_state['dt'] = dt
            turbine_state['ws'] = ws
            turbine_state['bld_pitch'] = bld_pitch[i-1]
            turbine_state['gen_torque'] = gen_torque[i-1]
            turbine_state['gen_speed'] = gen_speed[i]
            turbine_state['gen_eff'] = self.turbine.GenEff/100
            turbine_state['rot_speed'] = rot_speed[i]
            turbine_state['Yaw_fromNorth'] = nac_yaw[i]
            turbine_state['Y_MeasErr'] = nac_yawerr[i-1]

            # Define outputs
            gen_torque[i], bld_pitch[i], nac_yawrate[i] = self.controller_int.call_controller(turbine_state)

            # Calculate the power
            gen_power[i] = gen_speed[i] * gen_torque[i] * self.turbine.GenEff / 100

            # Calculate the nacelle position
            nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i] * dt

        self.controller_int.kill_discon()

        # Save these values
        self.bld_pitch = bld_pitch
        self.rot_speed = rot_speed
        self.gen_speed = gen_speed
        self.aero_torque = aero_torque
        self.gen_torque = gen_torque
        self.gen_power = gen_power
        self.t_array = t_array
        self.ws_array = ws_array
        self.wd_array = wd_array
        self.nac_yaw = nac_yaw

        if make_plots:
            # if sum(nac_yaw) > 0:
            if True:
                fig, axarr = plt.subplots(5, 1, sharex=True, figsize=(6, 10))

                ax = axarr[0]
                ax.plot(self.t_array, self.ws_array)
                ax.set_ylabel('Wind Speed (m/s)')
                ax.grid()
                ax = axarr[1]
                ax.plot(self.t_array, self.wd_array * 180/np.pi, label='WindDirection')
                ax.plot(self.t_array, self.nac_yaw * 180/np.pi, label='NacelleAngle')
                ax.legend(loc='best')
                ax.set_ylabel('Angle(deg)')
                ax.set_xlabel('Time (s)')
                ax.grid()
                ax = axarr[2]
                ax.plot(self.t_array, self.rot_speed)
                ax.set_ylabel('Rot Speed (rad/s)')
                ax.grid()
                ax = axarr[3]
                ax.plot(self.t_array, self.gen_power/1000)
                ax.set_ylabel('Gen Power (kW)')
                ax.grid()
                ax = axarr[4]
                ax.plot(self.t_array, self.bld_pitch*rad2deg)
                ax.set_ylabel('Bld Pitch (deg)')
                ax.set_xlabel('Time (s)')
                ax.grid()

            else:
                fig, axarr = plt.subplots(4, 1, sharex=True, figsize=(6, 10))

                ax = axarr[0]
                ax.plot(self.t_array, self.ws_array)
                ax.set_ylabel('Wind Speed (m/s)')
                ax.grid()
                ax = axarr[1]
                ax.plot(self.t_array, self.rot_speed)
                ax.set_ylabel('Rot Speed (rad/s)')
                ax.grid()
                ax = axarr[2]
                ax.plot(self.t_array, self.gen_torque)
                ax.set_ylabel('Gen Torque (N)')
                ax.grid()
                ax = axarr[3]
                ax.plot(self.t_array, self.bld_pitch*rad2deg)
                ax.set_ylabel('Bld Pitch (deg)')
                ax.set_xlabel('Time (s)')
                ax.grid()

    def sim_ws_wd_series(self, t_array, ws_array, wd_array,
                         rotor_rpm_init=10,
                         init_pitch=0.0,
                         init_yaw=None,
                         make_plots=True):
        '''
        Simulate simplified turbine model using a complied controller (.dll or similar).
            - currently a 1DOF rotor model

        Parameters:
        -----------
            t_array: float
                 Array of time steps, (s)
            ws_array: float
                 Array of wind speeds, (s)
            wd_array: float
                 Array of wind directions, (deg)
            rotor_rpm_init: float, optional
                 Initial rotor speed, (rpm)
            init_pitch: float, optional
                 Initial blade pitch angle, (deg)
            init_yaw: float, optional
                 Initial yaw angle, if None then start with no misalignment,
                 i.e., the yaw angle is set to the initial wind direction (deg)
            make_plots: bool, optional
                 True: generate plots, False: don't. 
        '''

        # Store turbine data for convenience
        dt = t_array[1] - t_array[0]
        R = self.turbine.rotor_radius
        GBRatio = self.turbine.Ng

        # Declare output arrays
        bld_pitch = np.ones_like(t_array) * init_pitch
        rot_speed = np.ones_like(t_array) * rotor_rpm_init * \
            rpm2RadSec  # represent rot speed in rad / s
        gen_speed = np.ones_like(t_array) * rotor_rpm_init * GBRatio * \
            rpm2RadSec  # represent gen speed in rad/s
        aero_torque = np.ones_like(t_array) * 1000.0
        gen_torque = np.ones_like(t_array)  # * trq_cont(turbine_dict, gen_speed[0])
        gen_power = np.ones_like(t_array) * 0.0
        nac_yawerr = np.ones_like(t_array) * 0.0
        if init_yaw is None:
            init_yaw = wd_array[0]
        else:
            nac_yawerr[0] = init_yaw - wd_array[0]
        nac_yaw = np.ones_like(t_array) * init_yaw
        nac_yawrate = np.ones_like(t_array) * 0.0

        # Loop through time
        for i, t in enumerate(t_array):
            if i == 0:
                continue  # Skip the first run
            ws = ws_array[i]
            wd = wd_array[i]
            nac_yawerr[i] = (wd - nac_yaw[i-1])*deg2rad

            # Load current Cq data
            tsr = rot_speed[i-1] * self.turbine.rotor_radius / ws
            cq = self.turbine.Cq.interp_surface([bld_pitch[i-1]], tsr)

            # Update the turbine state
            #       -- 1DOF model: rotor speed and generator speed (scaled by Ng)
            aero_torque[i] = 0.5 * self.turbine.rho * (np.pi * R**2) * cq * R * ws**2
            rot_speed[i] = rot_speed[i-1] + (dt/self.turbine.J)*(aero_torque[i]
                                                                 * self.turbine.GenEff/100 - self.turbine.Ng * gen_torque[i-1])
            gen_speed[i] = rot_speed[i] * self.turbine.Ng

            # populate turbine state dictionary
            turbine_state = {}
            # populate turbine state dictionary
            turbine_state = {}
            if i < len(t_array)-1:
                turbine_state['iStatus'] = 1
            else:
                turbine_state['iStatus'] = -1
            turbine_state['t'] = t
            turbine_state['dt'] = dt
            turbine_state['ws'] = ws
            turbine_state['bld_pitch'] = bld_pitch[i-1]
            turbine_state['gen_torque'] = gen_torque[i-1]
            turbine_state['gen_speed'] = gen_speed[i]
            turbine_state['gen_eff'] = self.turbine.GenEff/100
            turbine_state['rot_speed'] = rot_speed[i]
            turbine_state['Yaw_fromNorth'] = nac_yaw[i]
            turbine_state['Y_MeasErr'] = nac_yawerr[i-1]
            
            # Call the controller

            gen_torque[i], bld_pitch[i], nac_yawrate[i] = self.controller_int.call_controller(turbine_state)

            # Calculate the power
            gen_power[i] = gen_speed[i] * gen_torque[i]
            gen_power[i] = gen_speed[i] * gen_torque[i] * self.turbine.GenEff / 100

            # Update the nacelle position
            nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i]*rad2deg*dt

        # Save these values
        self.bld_pitch = bld_pitch
        self.rot_speed = rot_speed
        self.gen_speed = gen_speed
        self.aero_torque = aero_torque
        self.gen_torque = gen_torque
        self.gen_power = gen_power
        self.t_array = t_array
        self.ws_array = ws_array
        self.wd_array = wd_array
        self.nac_yaw = nac_yaw
        self.nac_yawrate = nac_yawrate

        if make_plots:
            fig, axarr = plt.subplots(nrows=6, sharex=True, figsize=(8, 14))

            ax = axarr[0]
            ax.plot(self.t_array, self.ws_array)
            ax.set_ylabel('Wind Speed (m/s)')

            ax = axarr[1]
            ax.plot(self.t_array, self.wd_array, label='wind direction')
            ax.plot(self.t_array, self.nac_yaw, label='yaw position')
            ax.set_ylabel('Wind Direction (deg)')
            ax.legend(loc='best')

            ax = axarr[2]
            ax.plot(self.t_array, self.nac_yaw*rad2deg)
            ax.set_ylabel('Nacelle yaw error (deg)')

            ax = axarr[3]
            ax.plot(self.t_array, self.rot_speed)
            ax.set_ylabel('Rot Speed (rad/s)')

            ax = axarr[4]
            ax.plot(self.t_array, self.gen_torque)
            ax.set_ylabel('Gen Torque (N)')

            ax = axarr[5]
            ax.plot(self.t_array, self.bld_pitch*rad2deg)
            ax.set_ylabel('Bld Pitch (deg)')

            ax.set_xlabel('Time (s)')
            for ax in axarr:
                ax.grid()
