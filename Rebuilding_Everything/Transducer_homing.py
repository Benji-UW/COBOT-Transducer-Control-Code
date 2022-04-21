'''
Unit convention: mm/kg/s/deg
'''
import socket, struct
import time
import pygame
import math
import os
import numpy as np

import logging
date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")
path = os.path.dirname(__file__)
logging.basicConfig(filename = path + "\logging\debug_log " + date_time_str + ".log", encoding='utf-8',\
     level=logging.DEBUG, format='%(levelname)s:%(message)s')
logging.debug("Debug log for the robot starting on " + date_time_str)
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

from pygame.locals import *
from Controller_2 import Controller_2
from UR3e import *
from RobotGUI import *
from datetime import datetime
from DemoPathfinder import *

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class Transducer_homing:
    '''The UR3e robot keeps the position and angle of the TCP in meters/radians, 
    this class mostly stores them in mm/deg'''
    def __init__(self):
        self.robot = None
        self.matlab_socket = None
        self.screen = None
        self.screen_resolution = (480, 940)
        self.controller = None
        self.robot_gui = None
        self.latest_loop = -1

        self.v_list = [0.005, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.4, 0.8]
        self.vR_list = [0.025, 0.05, 0.1, 0.2, 0.4, 0.8, 1.0, 2.0]
        self.a_list = [0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0]
        self.aR_list = [0.1, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 2.0]
        self.speed_preset = 3
        self.joy_min = 0.01

        self.refresh_rate = 112.
        self.lag = 0.2 #10. / self.refresh_rate #0.1
        self.true_refresh_rate = None
        self.button_hold = []
        self.max_disp = 0.5

        self.motion_comp = 0
        self.z_adjust = 0
        self.z_jerk_coeff = 3
        self.amp_hist = []

        self.Kp = 10.
        self.Ki = 0.0
        self.Kd = 0.1  # 0.01
        self.servo_lag = self.lag / 2.

        self.servo_error = 0.0

        self.t_pred = 0.126
        self.mp_t_upd = 0.1
        self.mp_t_lu = 0.

        self.servo_t = 0.02
        self.servo_lh_t = 0.035
        self.servo_gain = 1000

        self.range_of_motion = {'X': 0,'Y': 0,'Z':8,'Rx':30,'Ry':30,'Rz':0}

    def connect_to_matlab(self, server_ip='localhost', port=508):
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            print(f'Connection error to robot: {e}')
            return (False,e)
        print('Connected to MATLAB')
        return (True,'')

# DELETE IF CONNECTION WORKS IN UR3 MODULE
    # def connect_to_data_port(self, server_ip='192.168.0.5', port=50000):
    #     self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     try:
    #         self.data_socket.connect((server_ip, port))
    #     except socket.gaierror as e:
    #         print(f'Connection error due to data port: {e}')
    #         return (False, e)
    #     print('Connected to data_port')
    #     return (True, '')

    def disconnect_from_matlab(self):
        self.matlab_socket.send(b'end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        pygame.init()

        self.screen = pygame.display.set_mode(self.screen_resolution)
        pygame.display.set_caption('Transducer homing')

        pygame.joystick.init()
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            joystick = -1
        self.controller = Controller_2(joystick)

        self.robot_gui = RobotGUI()

        self.robot = UR3e()
        if not self.robot.connect(ip_robot)[0]:
            return False
        print("robot successfully connected to all ports!")
        
        
        self.robot.initialize()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]
        
        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
        self.robot.set_max_displacement(self.max_disp)
        self.robot.set_parameters(base='base')
        print("Robot initialized :)")

    def start(self):
        self.robot_gui.reset()
        run_bool = True
        router = False
        translate = True

        # self.listener = self.MATLAB_listener()
        self.listener = self.fake_MATLAB_listener()

        self.last_ten_refresh_rate = np.zeros((10,0))
        self.mag_loc = set()

        self.starting_pos = [np.copy(self.robot.pos), np.copy(self.robot.angle)]
        changed_preset = False
        nextpoint = None

        i_rr = 0

        self.t = time.time()
        (new_mag, latest_mag) = next(self.listener)

        self.i=-1
        print('About to start the main loop')

        while run_bool:
            t0 = time.time()
            self.robot.update()
            self.main_menu_GUI(router, nextpoint)
            #self.robot.get_tcp_force()

            (new_mag, latest_mag) = next(self.listener)

            # This section of code is entirely for handling a new signal reading from MATLAB
            if new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%10] = time.time() - self.t
                self.t = time.time()
                pos,angle = self.get_delta_pos()

                pos = tuple(map(tuple, pos.T))[0]
                angle = tuple(map(tuple, angle.T))[0]

                self.mag_loc.add(((pos,angle), new_mag))

                if router:
                    self.pathfinder.newMag(((pos,angle), latest_mag))
                    # You are here                

            pygame.event.pump()
            keys = pygame.key.get_pressed()
            pressed_buttons=self.controller.get_buttons(keys)

            # Press esc to escape the program and close out everything.
            if pressed_buttons["exit"] or keys[pygame.key.key_code("q")] == 1:
                run_bool = False
            if pressed_buttons["speed_preset_up"] or keys[pygame.key.key_code("]")] == 1:
                if self.speed_preset < len(self.v_list) - 1:
                    self.speed_preset += 1
                    changed_preset = True
            elif pressed_buttons["speed_preset_down"] or keys[pygame.key.key_code("[")] == 1:
                if self.speed_preset > 0:
                    self.speed_preset -= 1
                    changed_preset = True
            
            # Press d to trigger the demo pathfinder
            if pressed_buttons["switch_space"]:
                translate = not translate
            # Starts a demo pathfinder if the 'd' key is pressed
            if keys[pygame.key.key_code("d")] == 1 and not router:
                router = True
                self.pathfinder = Pathfinder(20,30,30)
                self.robot.set_initial_pos()
            if keys[pygame.key.key_code("k")] == 1 and not router:
                router = True
                self.pathfinder = FullScan((0.5,5),20,60,60)
            # Press x to stop the running pathfinder
            if keys[pygame.key.key_code("x")] == 1 and router:
                router = False
            if keys[pygame.key.key_code("m")] == 1 and router:
                t_pos,t_angle = (nextpoint[0],nextpoint[1])
                t_pos,t_angle = np.array([t_pos]).T,np.array([t_angle]).T

                self.robot.movel(self.starting_pos[0] + t_pos,self.starting_pos[1] +  t_angle)
                
                print("movel engaged")
                print("Target set to the following point:")
                print(self.starting_pos[0] + t_pos,self.starting_pos[1] +  t_angle)
            if keys[pygame.key.key_code("a")] == 1 and not router:
                self.change_range_gui()
            if keys[pygame.key.key_code("h")] == 1 and not router:
                self.robot.movel(self.starting_pos[0], self.starting_pos[1])

            # changes the speed preset of the robot if that occured.
            if changed_preset:
                v = self.v_list[self.speed_preset]
                vR = self.vR_list[self.speed_preset]
                a = self.a_list[self.speed_preset]
                aR = self.aR_list[self.speed_preset]
                self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)


            speed_vect = np.zeros((3,1))
            rot_vect = np.zeros((3,1))
            if router:
                nextpoint = self.pathfinder.next()
                #print(nextpoint)
                (speed_vect,rot_vect) = self.interpolate_motion(nextpoint)
                
                logging.debug("Transmitting to the robot the following motion vectors: ")
                logging.debug(f"Translate: {speed_vect} \nRotate: {rot_vect}")
            else:
                joy_vect = self.controller.get_hat(keys)
                if translate:
                    speed_vect = joy_vect
                else:
                    rot_vect[0] = joy_vect[1]
                    rot_vect[1] = joy_vect[0]
                    rot_vect[2] = joy_vect[2]

            
            self.robot.speedl(speed_vect,rot_vect,self.lag)

            logging.debug("----------------------------------------------")
            logging.debug("Position info about the robot:")
            logging.debug(f"Initial pos/angle: ({self.robot.initial_pos.T}, {self.robot.initial_angle.T})")
            logging.debug(f"Current pos/angle: ({self.robot.pos.T}, {self.robot.angle.T}")
            logging.debug("----------------------------------------------")


            print("sending string...")
            response = self.robot.data_string_io(b'RSR')
            print(response)


            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run_bool = False
            pygame.display.flip()
            if i_rr >= self.refresh_rate:
                i_rr = 0
            t = time.time()
            if 1.0 / self.refresh_rate - (t - t0) > 0:
                time.sleep(1.0 / self.refresh_rate - (t - t0))
        #-----------------------------Bottom of loop-----------------------------------#

        # Return robot to starting position (comment out when you don't wanna do this)
        self.robot.movel(self.starting_pos[0], self.starting_pos[1])

        # loop is exited
        self.robot.disconnect()
        print('Robot disconnected.')
        if self.matlab_socket is not None:
            self.disconnect_from_matlab()
            print('Disconnected from server.')
        if router:
            path = os.path.dirname(__file__)
            path = path + '\\test.json'
            self.pathfinder.save_points(path)

    def get_delta_pos(self):
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

        #print(d_pos, d_angle)
        return (d_pos,d_angle)

    def interpolate_motion(self,next_target):
        '''The bug is in here'''
        # This data is in mm and deg thanks to internal positioning methods
        c_pos,c_angle = self.get_delta_pos()

        logging.debug("\n\n-------------------------------------------------")
        logging.debug("Entering a new round of motion interpolation")
        logging.debug("-------------------------------------------------")

        logging.debug("1. Current change in position from the origin: ")
        logging.debug(f"\n{c_pos}, {c_angle}")
        
        # This data is also in mm and deg due to convention
        t_pos,t_angle = (next_target[0],next_target[1])
        t_pos,t_angle = np.array([t_pos]).T,np.array([t_angle]).T

        logging.debug("2. Current target position relative to the origin: \n (Should be the same shape)")
        logging.debug(f"\n{t_pos}, {t_angle}")

        #print('target pos:')
        #print(t_pos, t_angle)
        delta_pos = t_pos - c_pos
        
        logging.debug("3. Change in position required to get from the current to the target position")
        logging.debug(f"{delta_pos}")

        if (np.max(np.abs(delta_pos)) != 0):
            speed_vect = delta_pos / max(np.max(np.abs(delta_pos)),5)
            logging.debug("4. Calculated speed-vector to reach that delta_pos in a decent amount of time:")
            logging.debug(f"{speed_vect}")
        else:
            speed_vect = delta_pos

        delta_ang = t_angle - c_angle

        logging.debug("5. Change in angle required to get from the current to the target position")
        logging.debug(delta_ang.T)

        rot_vect = np.zeros((3,1))
        
        if (np.max(np.abs(delta_ang)) != 0):
            delta_ang = delta_ang / np.max(np.abs(delta_ang))
            rot_vect[0] = 1 * delta_ang[0] # 1 - Rx
            rot_vect[1] = -1 * delta_ang[1] 
            # rot_vect[2] = -0 * delta_ang[1]# 0 - Rz
            rot_vect[2] = 0
            logging.debug("6. Calculated angular-speed to reach that delta_pos in a decent amount of time:")
            logging.debug(f"{rot_vect}")
        
        
        # logging.debug("7. Translation vector about to be sent to the robot for execution: (trans, rot)")
        # logging.debug(f"{speed_vect}, {rot_vect}")

        return speed_vect,rot_vect


    def MATLAB_listener(self):
        '''This method creates a generator for listening to the server for new data from
        MATLAB. At each yield statement it returns a tuple containing a boolean and a float,
        the boolean indicating whether the magnitude is new and the float representing the
        magnitude.'''   
        mag,self.latest_loop = -1,-1        
        while True:
            print('here i am')
            self.matlab_socket.send(b'motion')

            msg = self.matlab_socket.recv(1024)
            print(msg)
            mag = float(msg[4:13]) / 1.0E3
            loop = int(msg[1:3])
            if loop == self.latest_loop:
                yield (False, mag)
            else:
                self.latest_loop = loop
                yield (True, mag)
    
    def fake_MATLAB_listener(self):
        '''This method fakes the input from the MATLAB listener, can be used to debug the
        motion of the robotic arm on occasions when we aren't using the transducer itself yet.'''
        while True:
            mag = round(time.time()) % 59
            yield (True, mag)

    def MATLAB_next(self):
        '''Hopefully can be deprecated once the generator is used successfully.'''
        self.matlab_socket.send(b'motion')
        msg = self.matlab_socket.recv(1024)
        print(msg)
        mag = float(msg[4:13]) * 1.0E3
        loop = int(msg[1:3])
        if loop == self.latest_loop:
            return (False, mag)
        else:
            self.latest_loop = loop
            return (True, mag)

    def main_menu_GUI(self,router_bool,current_target):
        self.screen.fill(WHITE)
        self.robot_gui.reset()
        self.write_pos_info(router_bool,current_target)

    def change_range_gui(self):
        '''It is not a priority right now but I would ultimately like there to be
        a GUI option for adjusting the default range of motion for the pathfinder.
        By default the range is +/- 8 mm along the z axis and +/- 45 degrees along
        the Rx and Ry axes.'''
        pass
        # not_set = True
        # cursor = 0
        # while not_set:
        #     self.screen.fill(WHITE)

        #     pygame.event.pump()
        #     keys = pygame.key.get_pressed()

    def write_pos_info(self, router, current_target):
        pos = self.robot.pos
        angle = self.robot.angle
        tcp_offset = self.robot.tcp_offset
        tcp_angle = self.robot.tcp_rot
        delta_pos,delta_angle = self.get_delta_pos()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]


        self.robot_gui.tprint(self.screen, 'Current base: %s' % self.robot.base)

        self.robot_gui.tprint(self.screen, 'TCP position in relation to its initial position:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (delta_pos[0], delta_angle[0]))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (delta_pos[1], delta_angle[2]))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (delta_pos[2], delta_angle[1]))
        self.robot_gui.unindent()

        self.robot_gui.skip_line(1)

        # self.robot_gui.tprint(self.screen, 'TCP position in base:')
        self.robot_gui.tprint(self.screen, "TCP position in base: ((%4.1f, %4.1f, %4.1f), (%3.0f,%3.0f,%3.0f))" % 
                            (pos[0]*1000,pos[1]*1000,pos[2]*1000,np.rad2deg(angle[0]),np.rad2deg(angle[1]),np.rad2deg(angle[2])))
        # self.robot_gui.indent()
        # self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
        #                       (pos[0]*1000, np.rad2deg(angle[0])))
        # self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
        #                       (pos[1]*1000, np.rad2deg(angle[1])))
        # self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
        #                       (pos[2]*1000, np.rad2deg(angle[2])))
        # self.robot_gui.unindent()

        if current_target is not None:
            self.robot_gui.skip_line(1)
            t_pos = current_target[0]
            t_ang = current_target[1]
            self.robot_gui.tprint(self.screen, "Next target: ((%4.1f, %4.1f, %4.1f), (%3.0f,%3.0f,%3.0f))" % 
                                (t_pos[0],t_pos[1],t_pos[2],t_ang[0],t_ang[1],t_ang[2]))


        self.robot_gui.skip_line(1)

        self.robot_gui.tprint(self.screen, f'Speed preset: {self.speed_preset}')

        self.robot_gui.skip_line(1)
        self.robot_gui.tprint(self.screen, f'Recent refresh rate: {np.mean(self.last_ten_refresh_rate)}')

        self.robot_gui.skip_line(3)
        
        if not router:
            self.robot_gui.tprint(self.screen, 'Press (d)emo to demonstrate the basic pathrouting module')
            self.robot_gui.tprint(self.screen, 'Press (k) to trigger a full scan with hard-coded resolution.')
            self.robot_gui.indent()
            self.robot_gui.tprint(self.screen, 'Beware this will override the controller until the pathfinder is cancelled or has finished the task.')
            self.robot_gui.unindent()
        else:
            next_target = self.pathfinder.next()
            t_pos,t_angle = (next_target[0],next_target[1])
            self.robot_gui.tprint(self.screen, 'Press (x) to cancel the running pathfinder')
            self.robot_gui.tprint(self.screen, "Press (m) to movel to the next target point.")
            self.robot_gui.skip_line(2)
            self.robot_gui.tprint(self.screen, 'Current target:')
            self.robot_gui.indent()
            self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                                (t_pos[0], t_angle[0]))
            self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                                (t_pos[1], t_angle[1]))
            self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                                (t_pos[2], t_angle[2]))
            self.robot_gui.unindent()

        
        self.robot_gui.skip_line(2)
        self.robot_gui.tprint(self.screen, time.ctime())

        #print('gonna update the gui')


    
    def test_functions(self):
        '''Sends a series of test functions to the robot and then logs them,
        I'm doing this to try to learn more about how to trigger events remotely.
        Each command contains a bytestring with the name of the command we're
        attempting to trigger, and an expected value that we're hoping to get back.'''
        commands = []
        commands.append((b'get_freedrive_status()\n', 'int'))
        commands.append((b'get_freedrive_status()\n', 'int but with less'))
        commands.append((b'force()\n', 'float'))
        commands.append((b'get_actual_joint_positions()\n', 'array of six floats'))
        commands.append((b'get_actual_tcp_pose()\n', 'array of six floats'))
        commands.append((b'get_controller_temp()\n', 'float'))
        commands.append((b'get_tcp_force()\n', 'array of six floats, maybe preceded with a p'))
        commands.append((b'is_steady()\n', 'boolean'))
        commands.append((b'wrench_trans(p[0,0,0,1,1,1], [0,0,0,1,1,2])\n', 'list of floats again'))
        commands.append((b'wrench_trans(p[0,0,0,1,1,1], [0,0,0,1,2,1])\n', 'list of floats again'))

        for command in commands:
            logstring = self.robot.generic_io_response(command[0])
            logging.debug(logstring)
        #     responses = self.robot.generic_test_command_response(command[0])
        #     logging.debug(f"Testing the {command[0]} command, expecting output as {command[1]}.")
        #     logging.debug(f"\nBytestring response:\n{responses[0]}\nHex responses:\n{responses[1]}")
        #     data = responses[0]

        #     # struct.unpack notes: ! represents the byte order (server = (big-endian))
        #     # B represents an unsigned char (b is signed)
        #     # i represents an int (I would be unsigned)
        #     # q represents a long long (Q would be unsigned)
            
        #     packlen = struct.unpack('!i', data[0:4])
        #     timestamp = struct.unpack('!Q', data[10:18])
        #     packtype = struct.unpack('!B', data[4:5])

        #     logging.debug(f"\nPackage length: {packlen}\nTimestamp: {timestamp}\nPackage type (supposed to be an unsigned int): {packtype}")

        
robot = Transducer_homing()
robot.initialize()

robot.connect_to_matlab()
 
# robot.test_functions()
robot.start()