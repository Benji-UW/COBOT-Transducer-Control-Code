'''
Unit convention: mm/kg/s/deg
'''
import socket
import time
import pygame
import math
import os
import numpy as np

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
        self.speed_preset = 2
        self.joy_min = 0.01

        self.refresh_rate = 112.
        self.lag = 0.1 #10. / self.refresh_rate #0.1
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

        self.range_of_motion = {'X': 0,'Y': 0,'Z':8,'Rx':45,'Ry':45,'Rz':0}


    def connect_to_matlab(self, server_ip='localhost', port=508):
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            print(f'Connection error to robot: {e}')
            return (False,e)
        print('Connected to MATLAB')
        return (True,'')

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
        
        
        self.robot.initialize()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]
        
        self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
        self.robot.set_max_displacement(self.max_disp)
        print("Robot initialized :)")

    def start(self):
        self.robot_gui.reset()
        run_bool = True
        router = False
        translate = True

        #self.listener = self.MATLAB_listener()
        self.listener = self.fake_MATLAB_listener()
        self.last_ten_refresh_rate = np.zeros((10,0))
        self.mag_loc = set()
        i_rr = 0

        self.t = time.time()
        (self.new_mag, self.latest_mag) = next(self.listener)

        self.i=-1
        print('About to start the main loop')
        self.robot.update()
        print('Gonna try to turn it off')
        self.robot._powerdown()

        while run_bool:
            t0 = time.time()
            self.robot.update()
            self.main_menu_GUI(router)

            (self.new_mag, self.latest_mag) = next(self.listener)
            #(self.new_mag, self.latest_mag) = self.MATLAB_next()
            #print('made it here')
            # This section of code is entirely for handling a new signal reading from MATLAB
            if self.new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%10] = time.time() - self.t
                self.t = time.time()
                pos,angle = self.get_delta_pos()

                pos = tuple(map(tuple, pos))
                angle = tuple(map(tuple, angle))

                self.mag_loc.add(((pos,angle), self.new_mag))

                if router:
                    self.pathfinder.newMag(((pos,angle), self.latest_mag))
                    # You are here                

            pygame.event.pump()
            keys = pygame.key.get_pressed()
            pressed_buttons=self.controller.get_buttons(keys)

            # Press esc to escape the program and close out everything.
            if pressed_buttons["exit"] or keys[pygame.key.key_code("q")] == 1:
                run_bool = False
            # Press d to trigger the demo pathfinder
            if pressed_buttons["switch_space"]:
                translate = not translate
            if keys[pygame.key.key_code("d")] == 1 and not router:
                router = True
                self.pathfinder = Pathfinder(8,45,45)            
                self.robot.set_initial_pos()
            # Press x to stop the running pathfinder
            if keys[pygame.key.key_code("x")] == 1 and router:
                router = False
            if keys[pygame.key.key_code("a")] == 1 and not router:
                self.change_range_gui()
                

            speed_vect = np.zeros((3,1))
            rot_vect = np.zeros((3,1))
            if router:
                nextpoint = self.pathfinder.next()
                #print(nextpoint)
                (speed_vect,rot_vect) = self.interpolate_motion(nextpoint)
            else:
                joy_vect = self.controller.get_hat(keys)
                if translate:
                    speed_vect = joy_vect
                else:
                    rot_vect = joy_vect
            
            self.robot.speedl(speed_vect,rot_vect, self.lag)


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

        # loop is exited
        self.robot.disconnect()
        print('Robot disconnected.')
        if self.matlab_socket is not None:
            self.disconnect_from_matlab()
            print('Disconnected from server.')
        if router:
            path = os.path.dirname(__file__) + '\\data\\' + time.ctime(time.time())
            self.pathfinder.save_points(path)


    def get_delta_pos(self):
        '''Converts the get_delta_pos method built into the UR3e method into
        the local units used in the pathfinders, mm/kg/s/deg'''
        d_pos,d_angle = self.robot.get_delta_pos()
        d_pos = d_pos*1000
        d_angle = np.rad2deg(d_angle)
        #print(d_pos, d_angle)
        return (d_pos,d_angle)

    def interpolate_motion(self,next_target):
        c_pos,c_angle = self.get_delta_pos()
        #print('current pos:')
        #print(c_pos,c_angle)
        t_pos,t_angle = (next_target[0],next_target[1])

        t_pos,t_angle = np.array([t_pos]).T,np.array([t_angle]).T

        #print('target pos:')
        #print(t_pos, t_angle)
        delta_pos = t_pos - c_pos
        #print('delta pos:')
        #print(delta_pos)
        speed_vect = delta_pos / np.max(delta_pos)

        delta_ang = t_angle - c_angle
        rot_vect = delta_ang / np.max(delta_ang)
        
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


    def main_menu_GUI(self,router_bool):
        self.screen.fill(WHITE)
        self.robot_gui.reset()
        self.write_pos_info(router_bool)

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

    def write_pos_info(self, router):
        pos = self.robot.pos
        angle = self.robot.angle
        tcp_offset = self.robot.tcp_offset
        tcp_angle = self.robot.tcp_rot
        delta_pos,delta_angle = self.robot.get_delta_pos()
        v = self.v_list[self.speed_preset]
        vR = self.vR_list[self.speed_preset]
        a = self.a_list[self.speed_preset]
        aR = self.aR_list[self.speed_preset]

        self.robot_gui.tprint(self.screen, 'Current base: %s' % self.robot.base)

        self.robot_gui.tprint(self.screen, 'TCP offset:')
        self.robot_gui.indent()
        self.robot_gui.tprint(self.screen, 'x= %4.1f mm, Rx= %3.0f deg' %
                              (tcp_offset[0] * 1000., tcp_angle[0] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'y= %4.1f mm, Ry= %3.0f deg' %
                              (tcp_offset[1] * 1000., tcp_angle[1] * 180 / math.pi))
        self.robot_gui.tprint(self.screen, 'z= %4.1f mm, Rz= %3.0f deg' %
                              (tcp_offset[2] * 1000., tcp_angle[2] * 180 / math.pi))
        self.robot_gui.unindent()

        self.robot_gui.skip_line(1)
        self.robot_gui.tprint(self.screen, f'Speed preset: {self.speed_preset}')

        self.robot_gui.skip_line(3)
        
        if not router:
            self.robot_gui.tprint(self.screen, 'Press (d)emo to demonstrate the basic pathrouting module')
            self.robot_gui.indent()
            self.robot_gui.tprint(self.screen, 'Beware this will override the controller until the pathfinder is cancelled or has finished the task.')
            self.robot_gui.unindent()
        else:
            self.robot_gui.tprint(self.screen, 'Press (x) to cancel the running pathfinder')
        
        #print('gonna update the gui')

        
robot = Transducer_homing()
robot.initialize()

robot.connect_to_matlab()
 
robot.start()