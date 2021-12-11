import socket
import time
import pygame
import math
import os
import numpy as np

from pygame.locals import *
from Controller import Controller
from UR3e import *
from RobotGUI import *
from datetime import datetime

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class Transducer_homing:
    def __init__(self):
        self.robot = None
        self.matlab_socket = None
        self.screen = None
        self.screen_resolution = (480, 940)
        self.controller = None
        self.robot_gui = None

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
        self.max_disp = 0.02

        self.waypoints = []
        self.current_waypoint = 0

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

        self.imager_x_vect = np.array([])
        self.x_vect = np.array([])
        self.mp_x_vect = np.array([])
        self.imager_z_vect = np.array([])
        self.z_vect = np.array([])
        self.mp_z_vect = np.array([])
        self.imager_t_vect = np.array([])

    def connect_to_matlab(self, server_ip='localhost', port=50008):
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror as e:
            print(f'Connection error to robot: {e}')
            return (False,e)
        return (True,'')

    def disconnect_from_matlab(self):
        self.matlab_socket.send('end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        pygame.init()

        self.screen = pygame.display.set_mode(self.screen_resolution)
        pygame.display.set_caption('Transducer homing')

        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.controller = Controller(joystick)

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

    def MATLAB_listener(self):
        '''This method creates a generator for listening to the server for new data from
        MATLAB. At each yield statement it returns a tuple containing a boolean and a float,
        the boolean indicating whether the magnitude is new and the float representing the
        magnitude.'''
        mag = -1
        latest_loop = -1
        while True:
            self.matlab_socket.send('motion')
            msg = self.matlab_socket.recv(1024)
            mag = float(msg[4:13]) / 1.0E3
            loop = int(msg[1:3])
            if loop == latest_loop:
                yield (False, mag)
            else:
                yield (True, mag)