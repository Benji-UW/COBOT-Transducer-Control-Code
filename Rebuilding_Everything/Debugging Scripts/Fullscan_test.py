import sys
import os
import time
import json
import random
path = os.path.abspath('')
# print("======================")
# print(path)
sys.path.append(path + '\\Rebuilding_Everything')
# print("======================")
# print(sys.path)
import DemoPathfinder
from DemoPathfinder import *

a = FullScan((0.5,5),8,50,50)


p = a.next()
print(p)

while p != 1:
    ((x,y,z),(Rx,Ry,Rz)) = p

    mag = round(10 - (x + 1)**2 - ((Rx - 15)/10)**2 - (Ry/10)**2 + (random.random() * 2))
    

    a.newMag((p,mag))
    p = a.next()

path = os.path.dirname(__file__)
#path = path + '\\' + time.ctime(time.time())

file_itr = 0
while os.path.exists(path + "\\test_%s.json" % file_itr):
    file_itr += 1

path = path + '\\test_%s.json' % file_itr

a.save_points(path)

