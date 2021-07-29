from RobotTreatment import *

robot = RobotTreatment()

robot.initialize(ip_robot='192.168.0.10')
robot.connect_to_matlab()
robot.start()
