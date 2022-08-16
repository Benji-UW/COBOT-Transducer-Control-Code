'''
Unit convention: mm/kg/s/deg
'''
import socket
import time
import os
import numpy as np
import logging
import json
import server
from pynput import keyboard
from logging.handlers import RotatingFileHandler
from logging import Formatter
from threading import Thread
from UR3e import *
from Pathfinders import *

def logger_setup():
    date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")

    # Configure the root logger to a particular folder, format, and level. Lower the level when things
    # are working better or worse.
    root_logger = logging.getLogger()
    path = os.path.dirname(__file__)
    
    handler = RotatingFileHandler(filename=path+f"\\logging\\runtime_test.log",\
        backupCount=8,encoding="utf-8")
    handler.doRollover()

    formatter = Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)
    root_logger.setLevel(logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info("Debug log for the robot starting on " + date_time_str)
    return logger

logger = logger_setup()

def main():
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    Thread(target=server.start_server).start()

    '''Initialize a transducer_homing object and run that mf'''
    robot = Transducer_homing()
    robot.initialize()
    robot.connect_to_matlab()

    robot.start()

class Transducer_homing:
    '''The UR3e robot keeps the position and angle of the TCP in meters/radians, 
    this class mostly stores them in mm/deg'''
    def __init__(self):
        self.robot = None
        self.matlab_socket = None
        self.latest_loop = -1

        self.joint_history = []
        self.starting_joints = None

        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()
        self.keys_pressed = set()

        self.headless_test = False

        self.speed_presets = [(0.005, 0.025,0.025,0.1),\
                            (0.0125,0.05,0.05,0.2),\
                            (0.025,0.1,0.1,0.4),\
                            (0.05,0.2,0.2,0.6),\
                            (0.1,0.4,0.3,0.8),
                            (0.2,0.8,0.4,1),
                            (0.4,1.0,0.5,1.2),
                            (0.8,2.0,1.0,2.0)]

        self.speed_preset = 3
        
        self.refresh_rate = 112.
        self.lag = 0.2 #10. / self.refresh_rate #0.1
        self.max_disp = 0.5

        self.range_of_motion = {'X': 0,'Y': 0,'Z':8,'Rx':30,'Ry':30,'Rz':0}

    def connect_to_matlab(self, server_ip='localhost', port=508):
        '''Connects to the central socket server that coordinates information between
        this module and the MATLAB signal processing info.'''
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            logger.warning(f'Connection error to robot: {e}')
            return (False,e)
        except ConnectionRefusedError as e:
            logger.warning("Connection was fuckin refused :/ fuck me")
            return (False,e)
        
        logger.info('Connected to MATLAB')
        return (True,'')

    def _disconnect_from_matlab(self):
        self.matlab_socket.send(b'end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        os.system("title " + 'Transducer homing')

        if self.headless_test:
            self.robot = Fake_UR3e()
        else:
            self.robot = UR3e()
            if not self.robot.connect(ip_robot)[0]:
                return False
            logger.info("robot successfully connected to all ports!")
            
            self.robot.initialize()

            v,vR,a,aR = self.speed_presets[self.speed_preset]
            
            self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
            self.robot.set_max_displacement(self.max_disp)
            self.robot.set_parameters(base='base')
        
        logger.info("Robot initialized :)")

    def start(self):
        run_bool = True
        router = False
        freedrive = False

        # self.listener = self.MATLAB_listener()
        self.listener = self.fake_MATLAB_listener()

        self.last_ten_refresh_rate = np.zeros((40,1))

        self.starting_pos = [np.copy(self.robot.pos), np.copy(self.robot.angle)]
        nextpoint = None

        i_rr = 0

        self.t = time.time()
        (new_mag, latest_mag) = next(self.listener)

        self.i=-1
        logger.info('About to start the main loop')
        go_next = True

        while run_bool:
            t0 = time.time()
            self.robot.update()
            self.main_menu_GUI(router, nextpoint, freedrive)


            (new_mag, latest_mag) = next(self.listener)

            if new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%40] = time.time() - self.t
                self.t = time.time()
                
                pos,angle = self.robot.current_relative_target

                if router:
                    self.pathfinder.newMag(((pos,angle), latest_mag), go_next)

            # For clarity, the response to key prseses has been moved to another method
            run_bool,router,nextpoint,freedrive = self.key_press_actions(run_bool,router,nextpoint,freedrive)

            # If there is a running pathfinder:
            if router and not freedrive:
                # Wait for the robot to arrive at current target (log how long it took)
                v = self.robot.wait_for_at_tar()
                logger.info(f"Waiting for the at_tar return took {v} loops")
#TODO Delete joint history items after IK has been understood
                # Store the joint angles of the robot in the joint_history for analysis
                self.robot.get_joint_angles()
                self.joint_history.append((nextpoint, np.copy(self.robot.joints).tolist(), \
                    (np.copy(self.robot.pos).tolist(), np.copy(self.robot.angle).tolist())))

                # Find the next point
                nextpoint = self.pathfinder.next()
                
                if nextpoint == 1: # nextpoint = 1 means end of path reached
                    router = False

                    # Save points
                    self.pathfinder.save_points()

                    # Return robot to starting position (comment out when you don't wanna do this)
                    self.robot.movel_to_target(((0,0,0),(0,0,0)))


                    data={"Start joints": self.starting_joints,\
                        "joints_at_points": self.joint_history,\
                        "TCP_offset": np.copy(self.robot.tcp_offset).tolist(),\
                        "Notes": "No notes yet"}
                    path = os.path.dirname(__file__) + f"\\IK_Scans\\Test_{time.strftime('%m_%d__%H_%M')}.json"
                    with open(path, 'w+') as outfile:
                        json.dump(data, outfile, indent=3)
                else:
                    logger.debug(f"triggering movel to {nextpoint}")
                    self.robot.movel_to_target(nextpoint)
            # Post things to the logs.
            self.main_loop_logs()
        #-----------------------------Bottom of loop-----------------------------------#

        # loop is exited
        self.robot.disconnect()
        logger.info('Robot disconnected.')
        if self.matlab_socket is not None:
            self._disconnect_from_matlab()
            logger.info('Disconnected from server.')
        if router:
            self.pathfinder.save_points()

    def key_press_actions(self, run_bool, router, nextpoint, freedrive):
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
            run_bool = False
        elif '[' in self.keys_pressed: # Increment
            if self.speed_preset < len(self.v_list) - 1:
                self.speed_preset += 1
        elif ']' in self.keys_pressed: # Decrement
            if self.speed_preset > 0:
                self.speed_preset -= 1
        if 'd' in self.keys_pressed and not router: # Start basic pathfinder
            router = True
            self.pathfinder = Pathfinder(25,15,15,12,18)
            self.robot.set_initial_pos()
            self.starting_joints = np.copy(self.robot.initial_joints).tolist()
        if "k" in self.keys_pressed and not router: # Start fullscan pathfinder
            router = True
            self.pathfinder = FullScan((0.5,0.5),6,0,25)
            nextpoint = self.pathfinder.next()
        if 'x' in self.keys_pressed and router: # stop the running pathfinder
            router = False
            self.pathfinder.save_points()
            logger.debug(f"triggering movel to {((0,0,0),(0,0,0))}")
            self.robot.movel_to_target(((0,0,0),(0,0,0)))
        if 'a' in self.keys_pressed and not router: # Change operating range
            self.change_range_gui()
        if 'f' in self.keys_pressed:
            self.robot.toggle_freedrive()
            freedrive = not freedrive

        return run_bool,router,nextpoint,freedrive

    def main_loop_logs(self):
        '''Log information that gets posted every single loop, 
        moved here to reduce crowding on the main screen.'''
        logger.debug(f"In-loop keys pressed: {self.keys_pressed}")
        logger.debug(f"Q in pressed keys: {'q' in self.keys_pressed}")
        logger.debug("----------------------------------------------")
        logger.debug("Position info about the robot:")
        logger.debug(f"Initial pos/angle: ({self.robot.initial_pos.T}, {self.robot.initial_angle.T})")
        logger.debug(f"Current pos/angle: ({self.robot.pos.T}, {self.robot.angle.T}")
        logger.debug(f"Current joint positions (radians): {self.robot.joints.T}")
        logger.debug("----------------------------------------------")

    def _get_delta_pos(self):
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

    def MATLAB_listener(self):
        '''This method creates a generator for listening to the server for new data from
        MATLAB. At each yield statement it returns a tuple containing a boolean and a float,
        the boolean indicating whether the magnitude is new and the float representing the
        magnitude.'''   
        mag,self.latest_loop = -1,-1        
        while True:
            self.matlab_socket.send(b'motion')
            msg = self.matlab_socket.recv(1024)

            mag = float(msg[4:13]) / 1.0E3
            loop = int(msg[1:3])

            # if loop == self.latest_loop:
            #     yield (False, mag)
            #TODO if this causes no problems, delete all instances of the "latest_loop" boolean
            if loop != self.latest_loop:
                self.latest_loop = loop
                yield (True, mag)
    
    def fake_MATLAB_listener(self):
        '''This method fakes the input from the MATLAB listener, can be used to debug the
        motion of the robotic arm on occasions when we aren't using the transducer itself yet.'''
        while True:
            mag = round(time.time()) % 59
            yield (True, mag)

    def change_range_gui(self):
        '''It is not a priority right now but I would ultimately like there to be
        a GUI option for adjusting the default range of motion for the pathfinder.
        By default the range is +/- 8 mm along the z axis and +/- 45 degrees along
        the Rx and Ry axes.'''
        pass

    def main_menu_GUI(self, router, current_target,freedrive):
        pos = self.robot.pos
        angle = self.robot.angle
        joints = self.robot.joints
        d_pos,d_ang = self._get_delta_pos()

        print(f'TCP position in relation to its initial position: ({d_pos.T}(mm),{d_ang.T}(deg))')
        print("=========================")
        print(f"TCP position in base: ({pos.T * 1000}, {np.rad2deg(angle.T)}")
        print(f"Current joint position in degrees: ({np.rad2deg(joints.T)})")
        print(f'Recent refresh rate: {np.mean(self.last_ten_refresh_rate)}')
        print()
        print(f"Freedrive active: {freedrive}")
        print("Press (f) to toggle (f)reedrive mode :)")
        print()
        
        if not router:
            print('Press (d)emo to demonstrate the basic pathrouting module')
            print('Press (k) to trigger a full scan with hard-coded resolution.')
            print('Press (g) to trigger a amplitude max-finding pathrouter.')
            print('Press (q) to quit')
        else:
            if current_target is not None and current_target != 1:
                print()
                t_pos,t_ang = current_target
                print(f"Next target: ({[format(a,'1.2f') for a in t_pos]},{[format(a,'1.2f') for a in t_ang]})")

            print('Press (x) to cancel the running pathfinder')
            print("Press (m) to movel to the next target point.")
            print("")
            print('Progress of the current running pathfinder:')
            for i in self.pathfinder.progress_report():
                print('\t'+ i)

        print()
        print(time.ctime())
        print('----------------------')

    def on_press(self, key):
        logger.debug(f"Pressed the key {key}")
        logger.debug(f"Pressed the character {key.char}")
        self.keys_pressed.add(key.char)
    
    def on_release(self, key):
        self.keys_pressed.remove(key.char)

if __name__=="__main__":
  main()
else:
  print("run from import")