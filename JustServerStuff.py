import socket
import time
# import pygame
import math
import os
import keyboard
import numpy as np
# from pygame.locals import *
from Controller import Controller
# from UR3e import *
from RobotGUI import *
# from MotionPrediction import *
from datetime import datetime

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class ServerObject:
    def __init__(self):
        self.matlab_socket = None
        self.screen = None
        self.screen_resolution = (480, 940)

        self.refresh_rate = 110.

    def connect_to_matlab(self, server_ip='localhost', port=50008):
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('attempting to ping MATLAB')
        try:
            self.matlab_socket.connect((server_ip, port))
        except socket.gaierror:
            print('Connection error to robot: %s' % e)
            return False
        print('successfully pinged MATLAB')
        return True, ''

    def disconnect_from_matlab(self):
        self.matlab_socket.send('end')
        self.matlab_socket.close()

    def initialize(self, ip_robot='192.168.0.10'):
        # pygame.init()

        # self.screen = pygame.display.set_mode(self.screen_resolution)
        # pygame.display.set_caption('Robot Treatment v0.14')
        print('Made it this far!')
        # pygame.joystick.init()
        # joystick = pygame.joystick.Joystick(0)
        # joystick.init()
        # self.controller = Controller(joystick)

        # self.robot_gui = RobotGUI()

        # self.robot = UR3e()
        # if not self.robot.connect(ip_robot)[0]:
        #     return False
        # self.robot.initialize()
        # v = self.v_list[self.speed_preset]
        # vR = self.vR_list[self.speed_preset]
        # a = self.a_list[self.speed_preset]
        # aR = self.aR_list[self.speed_preset]
        # self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)
        # self.robot.set_max_displacement(self.max_disp)

        # if not os.path.isdir('./motion'):
        #     os.mkdir('./motion')

    def start(self):
        a = 1
        run_bool = True

        while run_bool:
            # events = pygame.event.get()
            t0 = time.time()


            # self.screen.fill(WHITE)

            # pygame.event.pump()
            # keys = pygame.key.get_pressed()

            self.matlab_socket.send('motion')
            msg = self.matlab_socket.recv(1024)

            print('Just receieved a message from MATLAB that says %0, how interesting!', msg)

            if keyboard.is_pressed('q'):
                run_bool = False
                print('Disconnected from server')

            t = time.time()

            # for event in events:
            #     if event.type == pygame.QUIT:
            #         self.robot.disconnect()
            #         print ('Robot disconnected.')
            #         run_bool = False
            #         if self.matlab_socket is not None:
            #             self.disconnect_from_matlab()
            #             print('Disconnected from server.')   

            # pygame.display.flip()         
            print('refresh')

            if 1.0 / self.refresh_rate - (t - t0) > 0:
                time.sleep(1.0 / self.refresh_rate - (t - t0))
            print('refresh')



print('hello')

object = ServerObject()

object.initialize()
object.connect_to_matlab()

object.start()