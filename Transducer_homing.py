'''
Unit convention: mm/kg/s/deg
'''
import socket
import time
import os
import numpy as np
import logging
# import server
from pynput import keyboard
from logging.handlers import RotatingFileHandler
from logging import Formatter
# from threading import Thread
from UR3e import *
from Pathfinders import *

def logger_setup():
    date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")
    file_i = 0

    path = os.path.dirname(__file__)

    while os.path.exists(path + "\\Scans\\test_%s.json" % file_i):
        file_i += 1

    # Configure the root logger to a particular folder, format, and level. Lower the level when things
    # are working better or worse.
    root_logger = logging.getLogger()
    
    handler = RotatingFileHandler(filename=path+f"\\logging\\runtime_test.log",\
        backupCount=8,encoding="utf-8")
    handler.doRollover()

    formatter = Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)
    root_logger.setLevel(logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.debug("Debug log for the robot starting on " + date_time_str)
    return logger,file_i

logger,file_itr = logger_setup()

def main():
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    # Thread(target=server.start_server).start()

    '''Initialize a transducer_homing object and run that mf'''
    robot = Transducer_homing()
    robot.initialize()
    robot.connect_to_matlab()

    # robot.test_functions()
    robot.start()

class Transducer_homing:
    '''The UR3e robot keeps the position and angle of the TCP in meters/radians, 
    this class mostly stores them in mm/deg'''
    def __init__(self):
        self.robot = None
        self.matlab_socket = None
        self.robot_gui = None
        self.latest_loop = -1

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
        
        logger.debug('Connected to MATLAB')
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

        self.listener = self.MATLAB_listener()
        # self.listener = self.fake_MATLAB_listener()

        self.last_ten_refresh_rate = np.zeros((10,1))

        self.starting_pos = [np.copy(self.robot.pos), np.copy(self.robot.angle)]
        changed_preset = False
        nextpoint = None

        i_rr = 0

        self.t = time.time()
        (new_mag, latest_mag) = next(self.listener)

        self.i=-1
        logger.info('About to start the main loop')
        go_next = True
        path = os.path.dirname(__file__)

        while run_bool:
            t0 = time.time()
            self.robot.update()
            self.main_menu_GUI(router, nextpoint)

            (new_mag, latest_mag) = next(self.listener)

            # This section of code is entirely for handling a new signal reading from MATLAB
            if new_mag:
                # Iterate a counter that keeps track of the magnitude readings we've recieved
                self.i+=1
                # Log how much time has elapsed since the previous reading
                self.last_ten_refresh_rate[self.i%10] = time.time() - self.t
                self.t = time.time()
                
                pos,angle = self.robot.current_relative_target

                if router:
                    self.pathfinder.newMag(((pos,angle), latest_mag), go_next)

            # Press esc to escape the program and close out everything.
            logger.debug(f"In-loop keys pressed: {self.keys_pressed}")
            logger.debug(f"Q in pressed keys: {'q' in self.keys_pressed}")

            if 'q' in self.keys_pressed:
                run_bool = False
            elif '[' in self.keys_pressed:
                if self.speed_preset < len(self.v_list) - 1:
                    self.speed_preset += 1
                    changed_preset = True
            elif ']' in self.keys_pressed:
                if self.speed_preset > 0:
                    self.speed_preset -= 1
                    changed_preset = True
            # Starts a demo pathfinder if the 'd' key is pressed
            if 'd' in self.keys_pressed and not router:
                router = True
                self.pathfinder = Pathfinder(20,25,25)
                self.robot.set_initial_pos()
            if "k" in self.keys_pressed and not router:
                router = True
                self.pathfinder = FullScan((0.5,0.5),6,0,25,path=path + f"\\Scans\\test_{file_itr}.json")
                nextpoint = self.pathfinder.next()
            if 'x' in self.keys_pressed and router: # stop the running pathfinder
                router = False
                path = os.path.dirname(__file__)
                self.pathfinder.save_points(path,file_itr)
                logger.debug(f"triggering movel to {nextpoint}")
                self.robot.movel_to_target(nextpoint)
            if 'a' in self.keys_pressed and not router:
                self.change_range_gui()

            if changed_preset:
                v,vR,a,aR = self.speed_presets[self.speed_preset]
                self.robot.set_parameters(acc=a, velocity=v, acc_rot=aR, vel_rot=vR)

            if router:
                v = self.robot.wait_for_at_tar()
                logger.info(f"Waiting for the at_tar return took {v} loops")
                nextpoint = self.pathfinder.next()
                if nextpoint == 1: # End of path reached
                    router = False
                    path = os.path.dirname(__file__)
                    self.pathfinder.save_points(path,file_itr)
                    self.robot.movel_to_target(((0,0,0),(0,0,0)))
                else:
                    logger.debug(f"triggering movel to {nextpoint}")
                    self.robot.movel_to_target(nextpoint)
                    # literally do not pass go, do not collect $200 until
                    # the target has been reached.

            logger.debug("----------------------------------------------")
            logger.debug("Position info about the robot:")
            logger.debug(f"Initial pos/angle: ({self.robot.initial_pos.T}, {self.robot.initial_angle.T})")
            logger.debug(f"Current pos/angle: ({self.robot.pos.T}, {self.robot.angle.T}")
            logger.debug("----------------------------------------------")

            if i_rr >= self.refresh_rate:
                i_rr = 0
            t = time.time()
            if 1.0 / self.refresh_rate - (t - t0) > 0:
                time.sleep(1.0 / self.refresh_rate - (t - t0))
        #-----------------------------Bottom of loop-----------------------------------#

        # Return robot to starting position (comment out when you don't wanna do this)

        # loop is exited
        self.robot.disconnect()
        logger.info('Robot disconnected.')
        if self.matlab_socket is not None:
            self._disconnect_from_matlab()
            logger.info('Disconnected from server.')
        if router:
            path = os.path.dirname(__file__)
            self.pathfinder.save_points(path,file_itr)

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

    # def main_menu_GUI(self,router_bool,current_target):
    #     self.write_pos_info(router_bool,current_target)

    def change_range_gui(self):
        '''It is not a priority right now but I would ultimately like there to be
        a GUI option for adjusting the default range of motion for the pathfinder.
        By default the range is +/- 8 mm along the z axis and +/- 45 degrees along
        the Rx and Ry axes.'''
        pass

    def main_menu_GUI(self, router, current_target):
        pos = self.robot.pos
        angle = self.robot.angle
        d_pos,d_ang = self._get_delta_pos()

        indents = 0
        print(f'TCP position in relation to its initial position:\t({d_pos.T} (mm),{d_ang.T} (deg))')
        print("=========================")

        # print("TCP position in base: ((%4.1f, %4.1f, %4.1f), (%3.0f,%3.0f,%3.0f))" % 
        #                     (pos[0]*1000,pos[1]*1000,pos[2]*1000,np.rad2deg(angle[0]),np.rad2deg(angle[1]),np.rad2deg(angle[2])))

        print(f"TCP position in base: ({pos.T * 1000}, {np.rad2deg(angle.T)}")

        print(f'Recent refresh rate: {np.mean(self.last_ten_refresh_rate)}')
        print()
        
        if not router:
            print('Press (d)emo to demonstrate the basic pathrouting module')
            print('Press (k) to trigger a full scan with hard-coded resolution.')
            # self.robot_gui.indent()
            indents += 1
            print('\tBeware this will override the controller until the pathfinder is cancelled or has finished the task.')
            # self.robot_gui.unindent()
            indents -= 1
            print('Press (q) to quit')
        else:
            if current_target is not None and current_target != 1:
                print()
                # t_pos = current_target[0]
                # t_ang = current_target[1]
                # t_pos,t_ang = (current_target[0],current_target[1])
                t_pos,t_ang = current_target
                print(f"Next target: ({[format(a,'1.2f') for a in t_pos]},{[format(a,'1.2f') for a in t_ang]})")
                # print("Next target: ((%4.2f, %4.2f, %4.2f), (%3.1f,%3.1f,%3.1f))" % 
                #                     (t_pos[0],t_pos[1],t_pos[2],t_ang[0],t_ang[1],t_ang[2]))

            print('Press (x) to cancel the running pathfinder')
            print("Press (m) to movel to the next target point.")
            print("\n")
            # print('Current target:')
            # indents += 1
            # print('\tx= %4.2f mm, Rx= %3.1f deg' %
            #                     (t_pos[0], t_angle[0]))
            # print('\ty= %4.2f mm, Ry= %3.1f deg' %
            #                     (t_pos[1], t_angle[1]))
            # print('\tz= %4.2f mm, Rz= %3.1f deg' %
            #                     (t_pos[2], t_angle[2]))
            
            print('Progress of the current running pathfinder:')
            
            for i in self.pathfinder.progress_report():
                print('\t'+ i)

        print("\n")
        print(('\t' * indents) + time.ctime())
        c,l = os.get_terminal_size()
        print(c * '-')

    def on_press(self, key):
        # logger.debug(f"Pressed the key {key}")
        # logger.debug(f"Pressed the character {key.char}")
        self.keys_pressed.add(key.char)
    
    def on_release(self, key):
        self.keys_pressed.remove(key.char)

if __name__=="__main__":
  main()
else:
  print("run from import")