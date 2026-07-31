# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import ctypes
from ctypes import (
    byref,
    cdll,
    POINTER,
    c_float,
    c_char_p,
    #c_double,
    create_string_buffer,
    c_int32,
    c_void_p,
    wintypes,
)
import numpy as np
import platform
import os
import zmq
import logging
from rosco.toolbox.ofTools.util.FileTools import load_yaml



logger = logging.getLogger(__name__)
# Choose logging level below
logger.setLevel(logging.INFO)       # For a basic level of logs 
# logger.setLevel(logging.DEBUG)    # For more detailed logs helpful for debugging



# Some useful constants
deg2rad = np.deg2rad(1)
rad2deg = np.rad2deg(1)
rpm2RadSec = 2.0 * (np.pi) / 60.0


class ControllerInterface:
    """
    Define interface to a given controller using the avrSWAP array

    Methods:
    --------
    call_discon
    call_controller
    show_control_values

    Parameters:
    -----------
    lib_name : str
                name of compiled dynamic library containing controller, (.dll,.so,.dylib)

    """

    def __init__(self, lib_name, param_filename="DISCON.IN", **kwargs):
        """
        Setup the interface
        """
        self.lib_name = lib_name
        self.param_name = param_filename

        # Set default parameters
        # PARAMETERS
        self.DT = 0.1
        self.num_blade = 3
        self.char_buffer = 500
        self.avr_size = 500
        self.sim_name = "simDEBUG"
        self.pitch = 0 # option to provid initial blade pitch value

        # Set kwargs, like DT
        for k, w in kwargs.items():
            try:
                setattr(self, k, w)
            except:
                pass

        try:
            self.init_discon()
        except ValueError:
            pass

    def init_discon(self):
        # Initialize
        
        self.torque = 0
        # -- discon
        self.discon = cdll.LoadLibrary(self.lib_name)
        self.avrSWAP = np.zeros(self.avr_size)

        # Define some avrSWAP parameters, NOTE: avrSWAP indices are offset by -1 from Fortran
        self.avrSWAP[2] = self.DT
        self.avrSWAP[60] = self.num_blade
        self.avrSWAP[19] = 1.0  # HARD CODE initial gen speed = 1 rad/s
        self.avrSWAP[20] = 1.0  # HARD CODE initial rot speed = 1 rad/s
        self.avrSWAP[82] = 0  # HARD CODE initial nacIMU = 0
        self.avrSWAP[26] = 10  # HARD CODE initial wind speed = 10 m/s

        # Blade pitch initial conditions
        self.avrSWAP[3] = self.pitch * np.deg2rad(1)
        self.avrSWAP[32] = self.pitch * np.deg2rad(1)
        self.avrSWAP[33] = self.pitch * np.deg2rad(1)

        self.avrSWAP[27] = 1  # IPC

        # Torque initial condition
        self.avrSWAP[22] = 0

        # Code this as first call
        self.avrSWAP[0] = 0

        # Initialize DISCON and related
        self.aviFAIL = c_int32()  # 1
        self.accINFILE = self.param_name.encode("utf-8")
        self.avcOUTNAME = (self.sim_name).encode("utf-8")
        self.avcMSG = create_string_buffer(1000)
        self.discon.DISCON.argtypes = [
            POINTER(c_float),
            POINTER(c_int32),
            c_char_p,
            c_char_p,
            c_char_p,
        ]  # (all defined by ctypes)

        # Put some values in
        self.avrSWAP[48] = self.char_buffer
        self.avrSWAP[49] = len(self.param_name)
        self.avrSWAP[50] = len(self.avcOUTNAME)
        self.avrSWAP[51] = self.char_buffer

        # Run DISCON
        self.call_discon()

        # Code as not first run now that DISCON has been initialized
        self.avrSWAP[0] = 1

        if self.aviFAIL.value < 0:
            raise ValueError("ROSCO dynamic library has returned an error")

    def call_discon(self):
        """
        Call libdiscon.dll (or .so,.dylib,...)
        """
        # Convert AVR swap to the c pointer
        c_float_p = POINTER(c_float)
        data = self.avrSWAP.astype(np.float32)
        p_data = data.ctypes.data_as(c_float_p)

        # Run DISCON
        self.discon.DISCON(
            p_data, byref(self.aviFAIL), self.accINFILE, self.avcOUTNAME, self.avcMSG
        )

        # Push back to avr swap
        self.avrSWAP = data

    def call_controller(self, turbine_state, end=False):
        """
        Runs the controller. Passes current turbine state to the controller, and returns control inputs back

        Parameters:
        -----------
        turbine_state: dict
            t: float
                time, (s)
            dt: float
                timestep, (s)
            pitch: float
                blade pitch, (rad)
            genspeed: float
                    generator speed, (rad/s)
            geneff: float
                    generator efficiency, (rad/s)
            rotspeed: float
                    rotor speed, (rad/s)
            ws: float
                wind speed, (m/s)
            yaw: float, optional
                nacelle yaw position (from north) (rad)
            yawerr: float, optional
                yaw misalignment, defined as the wind direction minus the yaw
                position (rad)
        """

        # Add states to avr
        self.avrSWAP[0] = turbine_state["iStatus"]
        self.avrSWAP[1] = turbine_state["t"]
        self.avrSWAP[2] = turbine_state["dt"]
        self.avrSWAP[3] = turbine_state["bld_pitch"]
        self.avrSWAP[32] = turbine_state["bld_pitch"]
        self.avrSWAP[33] = turbine_state["bld_pitch"]
        self.avrSWAP[14] = (
            turbine_state["gen_speed"]
            * turbine_state["gen_torque"]
            * turbine_state["gen_eff"]
        )
        self.avrSWAP[22] = turbine_state["gen_torque"]
        self.avrSWAP[19] = turbine_state["gen_speed"]
        self.avrSWAP[20] = turbine_state["rot_speed"]
        self.avrSWAP[23] = turbine_state["Y_MeasErr"] # radians, yaw error
        self.avrSWAP[26] = turbine_state["ws"]
        self.avrSWAP[36] = turbine_state["Yaw_fromNorth"] # radians, nacelle heading from north
        try:
            self.avrSWAP[82] = turbine_state["NacIMU_FA_RAcc"]
        except KeyError:
            self.avrSWAP[82] = 0

        # pass translational acceleration
        try:
            self.avrSWAP[52] = turbine_state['FA_Acc_TT']
        except KeyError:
            self.avrSWAP[52] = 0

        # call controller
        self.call_discon()

        # return controller states
        self.pitch = self.avrSWAP[41]
        self.torque = self.avrSWAP[46]
        self.nac_yawrate = self.avrSWAP[47]

        return (self.torque, self.pitch, self.nac_yawrate)

    def show_control_values(self):
        """
        Show control values - should be obvious
        """
        print("Pitch", self.pitch)
        print("Torque", self.torque)

    def kill_discon(self):
        """
        Unload the dylib from memory: https://github.com/bwoodsend/cslug/blob/master/cslug/_stdlib.py
        """

        print("Shutting down {}".format(self.lib_name))
        handle = self.discon._handle

        # Start copy here
        OS = platform.system()

        def null_free_dll(*spam):  # pragma: no cover
            pass

        # Try to find a good runtime library which is always available and contains
        # the standard library C functions such as malloc() or printf().
        # XXX: Keep chosen library names in sync with the table in `cslug/stdlib.py`.

        extra_libs = []

        if OS == "Windows":  # pragma: Windows
            try:
                ctypes.windll.kernel32.FreeLibrary.argtypes = [wintypes.HMODULE]
                _dlclose = ctypes.windll.kernel32.FreeLibrary
                dlclose = lambda handle: 0 if _dlclose(handle) else 1
            except:
                kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
                kernel32.FreeLibrary.argtypes = [ctypes.wintypes.HMODULE]
                dlclose = lambda handle: 0 if kernel32.FreeLibrary(handle) else 1
            # There's some controversy as to whether this DLL is guaranteed to exist.
            # It always has so far but isn't documented. However, MinGW assumes that it
            # is so, should this DLL be removed, then we have much bigger problems than
            # just this line. There is also vcruntime140.dll which isn't a standard part
            # of the OS but is always shipped with Python so we can guarantee its
            # presence. But vcruntime140 contains only a tiny strict-subset of msvcrt.
            stdlib = ctypes.CDLL("msvcrt")

        elif OS == "Darwin":  # pragma: Darwin
            try:
                try:
                    # macOS 11 (Big Sur). Possibly also later macOS 10s.
                    stdlib = ctypes.CDLL("libc.dylib")
                except OSError:  # pragma: no cover
                    stdlib = ctypes.CDLL("libSystem")
            except OSError:  # pragma: no cover
                # Older macOSs. Not only is the name inconsistent but it's
                # not even in PATH.
                _stdlib = "/usr/lib/system/libsystem_c.dylib"
                if os.path.exists(_stdlib):
                    stdlib = ctypes.CDLL(_stdlib)
                else:
                    stdlib = None
            if stdlib is not None:  # pragma: no branch
                dlclose = stdlib.dlclose
            else:  # pragma: no cover
                # I hope this never happens.
                dlclose = null_free_dll

        elif OS == "Linux":  # pragma: Linux
            try:
                stdlib = ctypes.CDLL("")
            except OSError:  # pragma: no cover
                # Either Alpine Linux or Android.
                # Unfortunately, there doesn't seem to be any practical way
                # to tell them apart.
                stdlib = ctypes.CDLL("libc.so")

                # Android, like FreeBSD puts its math functions
                # in a dedicated `libm.so`.
                # The only way to know that this is not Alpine is to check if the math
                # functions are already available in `libc.so`.
                if not hasattr(stdlib, "sin"):
                    extra_libs.append(ctypes.CDLL("libm.so"))
            dlclose = stdlib.dlclose

        # End copy here
        dlclose.argtypes = [c_void_p]
        dlclose(handle)

        del self.discon


class wfc_zmq_server:
    """Server side implementation of wind farm control interface for the ROSCO using ZeroMQ

    This class enables users to receive measurements from ROSCO and then send back control
    setpoints (generator torque, nacelle heading and/or blade pitch angles) using ZeroMQ
    messaging library.

    Attirbutes
    ----------
    network_address : str
        Address of the server usually in the format "tcp://*:5555"
    timeout : float
        Time till server time out
    verbose : bool
        Prints details of messages being passed using the server
    logfile : string
        Path of the logfile; if logfile is not provided, logging is disabled

    methods
    -------
    runserver()
        Run the server to recieve and send data to ROSCO controllers
    wfc_controller(id, current_time)
        User defined method that contains the controller algorithm.
    """

    # Read the interface file to obtain the structure of measurements and setpoints
    interface_file = os.path.realpath(
        os.path.join(
            os.path.dirname(__file__), "../controller/rosco_registry/wfc_interface.yaml"
        )
    )
    wfc_interface = load_yaml(interface_file)

    def __init__(self, network_address="tcp://*:5555", timeout=600.0, verbose=False,logfile=None):
        """Instanciate the server"""
        self.network_address = network_address
        self.timeout = timeout
        self.verbose = verbose
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.connections = wfc_zmq_connections(self.wfc_interface)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(self.network_address)
        if self.verbose:
            print(
                f"Successfully established connection a ZeroMQ server at {network_address}"
            )
        
        if logfile is not None:
            print(logfile)
            logger_filehandler = logging.FileHandler(logfile, "w+")
            logger_filehandler.setFormatter(logging.Formatter("%(asctime)s: %(message)s"))
            logger.addHandler(logger_filehandler)
        else:
            logging.disable()
            if self.verbose:
                print('Logging disabled')

        logger.info(
            f"Successfully established connection a ZeroMQ server at {network_address}"
        )


    def runserver(self):
        """Run the server to get measurements and send setpoints to ROSCO"""
        connect_zmq = True
        while connect_zmq:
            # Wait for and obtain measurements from ROSCO
            measurements = self._get_measurements()

            # Obtain the identifier from the measurments
            id = int(measurements["ZMQ_ID"])

            # Add turbie id to the list of connected turbines
            self.connections._add_unique(id)

            # Update the measurements of the turbine
            self.connections._update_measurements(id, measurements)
            try:
                # Try to get the setpoints of the turbine
                logger.debug(f"Trying to get setpoints for id = {id}")
                self._get_setpoints(id, measurements)
            except NotImplementedError as e:
                # Disconnect from the server and raise an error
                # if the user has not defined a wind farm controller
                self._disconnect()
                logger.critical(f'Disconnected due to wfc_controller not being defined by the user')
                raise e
            else:
                # If setpoints are successfully read then
                # send the setpoint to the ROSCO client
                self._send_setpoints(id)

                # Check if there are no clients connected to the server
                # and if so, disconnect the server
                logger.debug('Checking for disconnect')
                connect_zmq = self._check_for_disconnect()

    def _get_setpoints(self, id, measurements):
        """Get current setpoint from the wind farm controller

        Gets the setpoint for the current turbine at the current time step
        """
        current_time = self.connections.measurements[id]["Time"]
        logger.debug(
            f"Asking wfc_controller for setpoints at time = {current_time} for id = {id}"
        )
        setpoints = self.wfc_controller.update_setpoints(id, current_time, measurements)
        logger.info(f"Received setpoints {setpoints} from wfc_controller for time = {current_time} and id = {id}")

        for i, s in enumerate(self.wfc_interface["setpoints"]):
            self.connections.setpoints[id][s] = setpoints.get(s, self.wfc_interface['setpoints_default'][i])
            logger.debug(f'Set setpoint {s} in the connections list to {setpoints.get(s,0)} for id = {id}')

    def wfc_controller(self, id, current_time, measurements):
        """User defined wind farm controller

        Users needs to overwrite this method by their wind farm controller.
        The user defined method should take as argument the turbine id, the
        current time and current measurements and return the setpoints
        for the particular turbine for the current time. It should ouput the
        setpoints as a dictionary whose keys should be as defined in
        wfc_zmq_server.wfc_interface. If user does not overwrite this method,
        an exception is raised and the simulation stops.

        Examples
        --------
        >>> # Define the wind farm controller
        >>> def wfc_controller(id, current_time):
        >>>     if current_time <= 10.0:
        >>>         YawOffset = 0.0
        >>>     else:
        >>>         if id == 1:
        >>>             YawOffset = -10.0
        >>>         else:
        >>>             YawOffset = 10
        >>>     setpoints = {}
        >>>     setpoints["ZMQ_YawOffset"] = YawOffset
        >>>     return setpoints
        >>>
        >>> # Overwrite the wfc_controller method of the server
        >>> server.wfc_controller = wfc_controller
        """
        logger.critical("User defined wind farm controller not found")
        raise NotImplementedError("Wind farm controller needs to be defined.")

    def _get_measurements(self):
        """Receive measurements from ROSCO .dll"""
        if self.verbose:
            print("[%s] Waiting to receive measurements from ROSCO...")

        # message_in = self.socket.recv_string()
        # Initialize a poller for timeouts
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)
        timeout_ms = int(self.timeout * 1000)
        # poller.poll(timeout_ms)
        events = poller.poll(timeout_ms)
        if self.socket in dict(events):
            # if poller.poll(timeout_ms):
            # Receive measurements over network protocol
            logger.debug(
                f"Checked for timeout and waiting for measurements from a ROSCO client"
            )
            message_in = self.socket.recv_string()
            logger.debug(f"Received raw message: {message_in} ")
        else:
            # raise IOError("[%s] Connection to '%s' timed out."
            #   % (self.identifier, self.network_address))
            logger.info(f"Connection timed out")
            raise IOError("Connection timed out")

        # Convert to individual strings and then to floats
        meas_float = message_in
        meas_float = meas_float.replace("\x00", "").split(",")
        meas_float = [float(m) for m in meas_float]

        # Convert to a measurement dict
        meas_dict = {}
        for i_meas, meas in enumerate(self.wfc_interface["measurements"]):
            meas_dict[meas] = meas_float[i_meas]
        logger.info(f"Received message (formatted): {meas_dict}")
        if self.verbose:
            print("[%s] Measurements received:", meas_dict)

        return meas_dict

    def _send_setpoints(self, id):
        """Send setpoints to ROSCO .dll ffor individual turbine control"""

        # Create a string message with setpoints to send to ROSCO
        message_out = ", ".join(
            [f"{s:016.5f}" for s in self.connections.setpoints[id].values()]
        ).encode("utf-8")

        #  Send reply back to client
        
        logger.debug(f"Raw setpoints to be sent to id = {id} is {message_out}")
        if self.verbose:
            print("[%s] Sending setpoint string to ROSCO: %s." % (id, message_out))

        # Send control setpoints over network protocol
        self.socket.send(message_out)
        logger.info(f"Sent setpoints {self.connections.setpoints[id]} to id = {id}")

        if self.verbose:
            print("[%s] Setpoints sent successfully." % id)

    def _check_for_disconnect(self):
        """Disconnect if no clients are connected to the server"""
        num_connected = sum(self.connections.connected.values())
        logger.debug(f'Still connected to {num_connected} clients')
        if num_connected > 0:
            connect_zmq = True
            if self.verbose:
                print("Still connected to ", num_connected, " ROSCO clients")
        else:
            connect_zmq = False
            logger.info('Shutting down server as all the clients have dropped off')
            self._disconnect()
        return connect_zmq

    def _disconnect(self):
        """Disconnect from zmq server"""
        logger.info('Socket terminated')
        self.socket.close()
        context = zmq.Context()
        context.term()


class wfc_zmq_connections:
    """
    This class is used to track the current ROSCO client connections,
    their current measurements and setpoints.
    """

    # Dictionary of ROSCO clients connected to the server
    connected = {}

    def __init__(self, wfc_interface):
        self.wfc_interface = wfc_interface
        self.setpoints = {}
        self.measurements = {}

    def _add_unique(self, id):
        """Add to the dictionary of connected client

        Add the current turbine to the dictionary of connected clients,
        if it has not been added before. Next, initilize the measurements
        and setpoints for the turbine to 0.
        """
        if id not in wfc_zmq_connections.connected.keys():
            wfc_zmq_connections.connected.update({id: True})
            logger.info(f"Connected to a new ROSCO with id = {id}")

            self.setpoints.update(
                {id: {s: 0.0 for s in self.wfc_interface["setpoints"]}}
            )  # init setpoints with zeros
            self.measurements.update(
                {id: {s: 0.0 for s in self.wfc_interface["measurements"]}}
            )  # init measurements with zeros

    def _update_measurements(self, id, measurements):
        """Update the measurements and remove turbine from connected clients"""
        self.measurements.update({id: measurements})
        logger.debug(f"Updated measurements for ROSCO with id = {id} ")
        if measurements["iStatus"] == -1:
            wfc_zmq_connections.connected[id] = False
            logger.info(f"Received disconnect signal from ROSCO with id = {id}")
