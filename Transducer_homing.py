'''
Unit convention: mm/kg/s/deg
'''
import socket
import time
import os
import numpy as np
import logging
import json
# import server
from pynput import keyboard
from logging.handlers import RotatingFileHandler
from logging import Formatter
from UR3e import *
from Pathfinders import *

(X_RANGE,               # ROM in the X-axis (0)
 Y_RANGE,               # "   "   "  Y-axis (0)
 Z_RANGE,               # "   "   "  Z-axis (positive)
 RX_RANGE,              # "   "   "  Rx-axis (positive)
 RY_RANGE,              # "   "   "  Ry-axis (positive)
 RZ_RANGE) = 0,0,2.5,5,5,0
T_RES,R_RES = 0.25,0.5     # Resolution of high-def scans, in mm and deg respectively
D_PATHFINDER = FourSquares
K_PATHFINDER = EllipsoidFullScan
G_PATHFINDER = Greedy_discrete_degree
T_PATHFINDER = Bespoke_to_OCE
HEADLESS_TEST = False
IK_TEST:bool = False # Set to True to save joint positions during a scan
DATA_CHANNELS = 3


# LOGGING SETUP IS COMPLETE, don't touch again
def logger_setup(log_level=logging.INFO) -> logging.Logger:
    '''Sets up the logging system.'''
    date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")

    # Configure the root logger to a particular folder, format, and level.
    # Lower the level when things are working better or worse.
    root_logger = logging.getLogger()
    path = os.path.dirname(__file__)
    
    handler = RotatingFileHandler(filename=path+f"\\logging\\runtime_test.log",
        backupCount=8,encoding="utf-8")
    handler.doRollover()

    formatter = Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)
    root_logger.setLevel(log_level)
    logger = logging.getLogger(__name__)
    logger.info("Debug log for the robot starting on " + date_time_str)
    return logger

logger = logger_setup(logging.INFO)
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

def main():
    '''Initialize a transducer_homing object and run that mf'''
    robot = Transducer_homing()
    robot.initialize()
    robot.connect_to_matlab()

    robot.start()

class Transducer_homing:
    '''The UR3e robot keeps the position and angle of the TCP in 
    meters/radians, this class mostly stores them in mm/deg'''
    def __init__(self):
        self.robot: UR3e = None
        self.matlab_socket: socket.socket = None
        self.latest_loop:int = -1

        if IK_TEST:
            self.joint_history: list[tuple] = []
            self.starting_joints: list[float] = None

        self.key_listener = keyboard.Listener(on_press=self.on_press,
            on_release=self.on_release)
        self.key_listener.start()
        self.keys_pressed:set[str] = set()


# TODO: Delete all use of speed-presets et cetera,
# they are not necessary for the current robot configuration. 
        self.speed_presets = [(0.005, 0.025,0.025,0.1),
                            (0.0125,0.05,0.05,0.2),
                            (0.025,0.1,0.1,0.4),
                            (0.05,0.2,0.2,0.6),
                            (0.1,0.4,0.3,0.8),
                            (0.2,0.8,0.4,1),
                            (0.4,1.0,0.5,1.2),
                            (0.8,2.0,1.0,2.0)]

        self.speed_preset: int = 3
        
        self.refresh_rate:float = 112.
        # self.lag = 0.2 #10. / self.refresh_rate #0.1

        self.range_of_motion = {'X': 0,'Y': 0,'Z':8,'Rx':30,'Ry':30,'Rz':0}

    def connect_to_matlab(self, server_ip:str='localhost',
                        port:int=508) -> tuple[bool, str]:
        '''Connects to the socket server that passes information
        between this module and the signal processing module.'''
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            logger.warning(f'Connection error to robot: {e}')
            return (False,e.strerror)
        except ConnectionRefusedError as e:
            logger.warning("Connection was fuckin refused :/ fuck me")
            return (False,str(e))
        
        logger.info('Connected to MATLAB')
        return (True,'')

    def _disconnect_from_matlab(self):
        self.matlab_socket.send(b'end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        os.system("title " + 'Transducer homing')

        if HEADLESS_TEST:
            self.robot = Fake_UR3e()
        else:
            self.robot = UR3e()
            if not self.robot.connect(ip_robot)[0]:
                return False
            logger.info("robot successfully connected to all ports!")
            
            self.robot.initialize()

            v,vR,a,aR = self.speed_presets[self.speed_preset]
            
            self.robot.set_parameters(acc=a,velocity=v,acc_rot=aR,vel_rot=vR)
            #TODO: Delete following if no problems
            # self.robot.set_max_displacement(self.max_disp)
            self.robot.set_parameters(base='base')
        
        logger.info("Robot initialized :)")

    def start(self):
        _RUN,_PATHFINDER_ACTIVE,freedrive = True,False,False

        if HEADLESS_TEST:
            self.listener = self.fake_MATLAB_listener()
        else:
            # self.listener = self.fake_MATLAB_listener()
            self.listener = self.MATLAB_listener()

        self.last_ten_refresh_rate:np.array = np.zeros((40,1))

        self.starting_pos = [np.copy(self.robot.pos), np.copy(self.robot.angle)]
        nextpoint:np.ndarray | int = None

        # i_rr = 0.

        self.t:float = time.time()
        (new_mag, latest_mag) = next(self.listener) # MAYBE latest_mag is a tuple???
        # latest_mag = 1
        
        self.i=-1
        logger.info('About to start the main loop')
        t0 = time.time()

        time.sleep(0.025)
        while _RUN:
            self.robot.update()
            t1=time.time()
            if _PATHFINDER_ACTIVE and not freedrive:
                # Wait for the robot to arrive at current target (log how long it took)
                v = self.robot.wait_for_at_tar()
                logger.info(f"Waiting for the at_tar return took {v} loops")

            (new_mag, latest_mag) = next(self.listener)

            if new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%40] = time.time() - self.t
                self.t = time.time()
                
                pos_angle = self.robot.current_relative_target

                if _PATHFINDER_ACTIVE:
                    self.pathfinder.newMag(np.append(pos_angle,latest_mag))

            # For clarity, the response to key prseses has been moved to another method
            _RUN,_PATHFINDER_ACTIVE,nextpoint,freedrive = self.key_press_actions(_RUN,_PATHFINDER_ACTIVE,nextpoint,freedrive)

            # If there is a running pathfinder:
            if _PATHFINDER_ACTIVE and not freedrive:

                # Find the next point
                nextpoint = self.pathfinder.next()

                if type(nextpoint) is str and nextpoint == 'reset_origin':
                    self.robot.set_initial_pos()
                    time.sleep(0.01)
                    nextpoint = self.pathfinder.next()
                if type(nextpoint) is int and nextpoint == 1: # nextpoint = 1 means end of path reached
                    _PATHFINDER_ACTIVE = False

                    # Save points
                    self.pathfinder.save_points()

                    # # Return robot to starting position (comment out when you don't wanna do this)
                    # self.robot.movel_to_target(np.zeros(6))
                    # self.robot.movel_to_target(self.pathfinder.max_point[:6])

                    if IK_TEST:
                        self._save_IK_data()
                else:
                    logger.debug(f"triggering movel to {nextpoint}")
                    self.robot.movel_to_target(nextpoint)

            # Update the GUI if enough time has elapsed.
            if (t1 - t0) > (1/30):
                self.main_menu_GUI(_PATHFINDER_ACTIVE, nextpoint, freedrive, latest_mag)
                t0 = t1
            # Post things to the logs.
            self.main_loop_logs()
        #-------------------------Bottom of loop-------------------------------#
        # self.robot.movel_to_target(np.zeros(6))

        # loop is exited
        self.robot.disconnect()
        logger.info('Robot disconnected.')
        if self.matlab_socket is not None:
            self._disconnect_from_matlab()
            logger.info('Disconnected from server.')
        if _PATHFINDER_ACTIVE:
            self.pathfinder.save_points()

    def _save_IK_data(self, notes="No notes yet"):
        data={"Start joints": self.starting_joints,
            "joints_at_points": self.joint_history,
            "TCP_offset": np.copy(self.robot.tcp_offset).tolist(),
            "Notes": "No notes yet"}
        path = (os.path.dirname(__file__) + "\\IK_Scans" + 
            f"\\Test_{time.strftime('%m_%d__%H_%M')}.json")
        with open(path, 'w+') as outfile:
            json.dump(data, outfile, indent=3)

    def key_press_actions(
            self,
            RUN:bool,
            PATHFINDER_ACTIVE:bool,
            nextpoint:np.ndarray,
            freedrive:bool) -> tuple[bool,Pathfinder,tuple,bool]:
        '''
        Each run through the main loop, listen for actions triggered
        by key input.
        q - Quit the program and exit (saving first)
        [ - Increment the robot speed preset (deprecated)
        ] - Decrement the "     "     "      (")
        d - Start the baseline, demonstration pathfinder (visits each
            corner of the search space once)
        k - Start the full-scan pathfinder (traverses the entire search
            space at a given resoltion (slow))
        g - Start the greedy_discrete_degree pathfinder
        x - Cancel the running pathfinder (robot should go home)
        a - change the range of the full-scan pathfinder 
            (not implemented)
        f - toggle Freedrive
        '''
        if 'q' in self.keys_pressed: # Quit
            RUN = False
            self.keys_pressed.remove('q')
        # elif '[' in self.keys_pressed: # Increment
        #     if self.speed_preset < len(self.v_list) - 1:
        #         self.speed_preset += 1
        #     self.keys_pressed.remove('[')
        # elif ']' in self.keys_pressed: # Decrement
        #     if self.speed_preset > 0:
        #         self.speed_preset -= 1
        #     self.keys_pressed.remove(']')
        if PATHFINDER_ACTIVE:
            if 'x' in self.keys_pressed: # stop the running pathfinder
                PATHFINDER_ACTIVE = False
                self.pathfinder.save_points()
                logger.debug(f"triggering movel to {((0,0,0),(0,0,0))}")
                self.robot.movel_to_target(np.zeros(6))
                self.keys_pressed.remove('x')
        else:
            if 'd' in self.keys_pressed: # Start basic pathfinder
                PATHFINDER_ACTIVE = True
                self.pathfinder = D_PATHFINDER(Z_RANGE,RX_RANGE,RY_RANGE,X_RANGE,Y_RANGE,RZ_RANGE)
                self.robot.set_initial_pos()
                self.starting_joints = np.copy(self.robot.initial_joints).tolist()
                self.keys_pressed.remove('d')
            if "k" in self.keys_pressed: # Start fullscan pathfinder
                PATHFINDER_ACTIVE = True
                self.pathfinder = K_PATHFINDER((T_RES,R_RES),Z_RANGE,RX_RANGE,
                                    RY_RANGE,X_RANGE,Y_RANGE,RZ_RANGE,
                                    semi_axes=np.ones(6)*0.98,data_channels=DATA_CHANNELS)
                nextpoint = self.pathfinder.next()
                self.keys_pressed.remove('k')
            if "g" in self.keys_pressed: # Start maxfinding pathfinder
                PATHFINDER_ACTIVE = True
                # self.pathfinder = Greedy_discrete_degree(20,15,15,bias=0,steps=2,inc=1.0)
                # self.pathfinder = GradientAscent(20,15,15,bias=0,steps=2,inc=0.75,traverse=1.0)
                self.pathfinder = G_PATHFINDER(Z_RANGE,RX_RANGE,
                                    RY_RANGE,X_RANGE,Y_RANGE,RZ_RANGE,
                                    bias=0,inc=0.2,data_channels=3)
                nextpoint = self.pathfinder.next()
                self.keys_pressed.remove('g')
            if "t" in self.keys_pressed: # Start maxfinding pathfinder
                PATHFINDER_ACTIVE = True
                # self.pathfinder = Greedy_discrete_degree(20,15,15,bias=-1,steps=2,inc=1.0)
                self.pathfinder = T_PATHFINDER(Z_RANGE,RX_RANGE,
                                    RY_RANGE,X_RANGE,Y_RANGE,RZ_RANGE)
                # self.pathfinder = GradientAscent(20,15,15,bias=0,steps=2,inc=0.75,traverse=1.0)
                nextpoint = self.pathfinder.next()
                self.keys_pressed.remove('t')
            if 'a' in self.keys_pressed: # Change operating range
                self.change_range_gui()
                self.keys_pressed.remove('a')
            if 'f' in self.keys_pressed:
                self.robot.toggle_freedrive()
                freedrive = not freedrive
                self.keys_pressed.remove('f')

        return RUN,PATHFINDER_ACTIVE,nextpoint,freedrive

    def main_loop_logs(self):
        '''Log information that gets posted every single loop, 
        moved here to reduce crowding on the main screen.'''
        logger.debug(f"In-loop keys pressed: {self.keys_pressed}")
        logger.debug(f"Q in pressed keys: {'q' in self.keys_pressed}")
        logger.debug("----------------------------------------------")
        logger.debug("Position info about the robot:")
        logger.debug(f"\tInitial pos/angle: ({self.robot.initial_pos.T},".join(
            f"{self.robot.initial_angle.T})"))
        logger.debug(f"\tCurrent pos/angle: ({self.robot.pos.T}, {self.robot.angle.T}")
        logger.debug(f"\tCurrent joint positions (radians): {self.robot.joints.T}")
        logger.debug("----------------------------------------------")

    def _get_delta_pos(self) -> tuple[float,float]:
        '''Converts the get_delta_pos method built into the UR3e method into
        the local units used in the pathfinders, mm/kg/s/deg'''
        d_pos,d_angle = self.robot.get_delta_pos()
        
        d_pos = d_pos*1000
        d_angle = np.rad2deg(d_angle)
        
        for i in range(3):
            if (d_angle[i] > 180):
                d_angle[i] -= 360
            elif (d_angle[i] < -180):
                d_angle[i] += 360

        return (d_pos,d_angle)

    def MATLAB_listener(self) -> tuple[bool, float]:
        '''Creates a generator for listening to the server for new data from
        MATLAB.

        Returns a tuple containing a boolean and a float, the boolean
        indicating whether the magnitude is new and the float representing the data.'''
        a_,b_ = 5,500
        def loss_function(slope, intercept):
            return 1000 - a_*(np.abs((slope*200) + intercept - 110)) - b_*np.abs(slope)

        mag,self.latest_loop = -1,-1
        i=0
        while True:
            self.matlab_socket.send(b'motion')
            msg = self.matlab_socket.recv(1024)
            i += 1

            mag = []
            for i in range(DATA_CHANNELS-1):
                mag.append((float(msg[4+(10*i):13+(10*i)]) - 50000) / 100000)

            # mag = (float(msg[4:13]) - 50000) / 100000

            mag.append(loss_function(mag[0],mag[1]))

            mag = tuple(mag)
            loop = int(msg[1:3])

            if loop == self.latest_loop:
                time.sleep(0.005)
            else:
                self.latest_loop = loop
                yield (True, mag)

    
    def fake_MATLAB_listener(self) -> tuple[bool, float]:
        '''This method fakes the input from the MATLAB listener, can be used 
        to debug the motion of the robotic arm on occasions when we aren't using
        the transducer itself yet.'''
        while True:
            mag = round(time.time()) % 59
            yield (True, mag)

    def change_range_gui(self):
        '''It is not a priority right now but I would ultimately like there to
        be a GUI option for adjusting the default range of motion for the
        pathfinder. By default the range is +/- 8 mm along the z axis and +/- 45
        degrees along the Rx and Ry axes.'''
        keys = list(self.range_of_motion.keys())

        def key_press_actions_2(self, keep_going:bool,
            row:int) -> tuple[bool,int]:
            ''' Each run through the main loop, listen for actions triggered
            by key input.
            q - Quit the program and exit (saving first)'''
            if 'q' in self.keys_pressed: # Quit
                keep_going = False
                self.keys_pressed.remove('q')
            elif 'right' in self.keys_pressed: # Increment
                self.range_of_motion[keys[row]] += 1
                self.keys_pressed.remove('right')
            elif 'left' in self.keys_pressed: # Decrement
                self.range_of_motion[keys[row]] -= 1
                self.keys_pressed.remove('left')
            elif 'up' in self.keys_pressed: # Decrement
                if row > 0:
                    row -= 1
                self.keys_pressed.remove('up')
            elif 'down' in self.keys_pressed: # Decrement
                if row < len(keys):
                    row += 1
                self.keys_pressed.remove('down')
                
            return keep_going,row

        keep_going = True
        row = 0

        while keep_going:
            prin = ["Modify the range of motion that will be passed to the" 
                " Fullscan or demo pathfinders."]
            
            for i in range(len(keys)):
                if i == row:
                    prin.append(f"{keys[i]}: {self.range_of_motion[keys[i]]} <---")
                else:
                    prin.append(f"{keys[i]}: {self.range_of_motion[keys[i]]}")
    
            prin.append('----------------------')
            print('\n'.join(prin), end=(len(prin) + 1)*'\033[F')
            keep_going,row = key_press_actions_2(self,keep_going,row)     

    def main_menu_GUI(self, PATHFINDER_ACTIVE, current_target, freedrive, latest_mag):
        pos = self.robot.pos
        angle = self.robot.angle
        joints = self.robot.joints
        d_pos,d_ang = self._get_delta_pos()


        prin = ["TCP position in relation to its initial position:",
        f"\t({d_pos.T}(mm),{d_ang.T}(deg))",
        "=========================",
        f"TCP position in base: ({pos.T * 1000}, {np.rad2deg(angle.T)}",
        f"Current joint position in degrees: ({np.rad2deg(joints.T)})",
        f'Recent refresh rate: {np.mean(self.last_ten_refresh_rate)}',
        "------------------------",
        f"Latest mag:{latest_mag}",
        f"Freedrive active: {freedrive}",
        "Press (f) to toggle (f)reedrive mode :)",
        '']

        # for i in range(DATA_CHANNELS):
        #     prin.append(f"\t")
        #     bars = int((latest_mag[i]/2.5) * 80)
        #     spaces = 80 - bars
        #     prin.append((bars*'|') + (spaces*' ') + '|')
        
        
        if not PATHFINDER_ACTIVE:
            [prin.append(i) for i in 
            ['Press (d)emo to demonstrate the basic pathrouting module',
            'Press (k) to trigger a full scan with hard-coded resolution.',
            'Press (g) to trigger a amplitude max-finding pathrouter.',
            'Press (t) to trigger an alternate amplitude max-finding pathrouter.',
            'Press (q) to quit']]
        else:
            if current_target is not None and type(current_target) is not int:
                prin.append('')
                t_pos_ang = current_target
                prin.append(f"Next target: ({[format(a,'1.2f') for a in t_pos_ang]})")
            [prin.append(i) for i in ['Press (x) to cancel the running pathfinder',
            "Press (m) to movel to the next target point.", "",
            'Progress of the current running pathfinder:']]
            for i in self.pathfinder.progress_report():
                prin.append('\t'+ i)

        prin.append('')
        prin.append('')
        prin.append('\n' + time.ctime() + '\n' + 16*'-')
        prin.append('')
        prin.append('')
        prin.append('')
        prin.append('')
        prin.append('')

        print('\n'.join(prin), end=(len(prin) + 1)*'\033[F')

    def on_press(self, key):
        logger.debug(f"Pressed the key {key}")
        logger.info(f"Pressed the character {key.char}")
        self.keys_pressed.add(key.char)
    
    def on_release(self, key):
        pass
        # self.keys_pressed.remove(key.char)

if __name__=="__main__":
  main()
else:
  print("run from import")