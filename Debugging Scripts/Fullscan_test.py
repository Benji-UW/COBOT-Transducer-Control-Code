import sys
import os
import time
import json
import random
path = os.path.abspath('')
# print("======================")
# print(path)
sys.path.append(path)
# print("======================")
# print(sys.path)
from Pathfinders import *

path = os.path.dirname(__file__)
#path = path + '\\' + time.ctime(time.time())

file_itr = 0
while os.path.exists(path + "\\Scans\\test_%s.json" % file_itr):
    file_itr += 1

path = path + '\\Scans\\test_%s.json' % file_itr


a = FullScan((0.5,2),10,20,20,path=path)


p = a.next()
print(p)

start = time.time()

while p != 1:
    ((x,y,z),(Rx,Ry,Rz)) = p

    # mag = round(10 - (z + 1)**2 - ((Rx - 35)/10)**2 - ((Ry + 25)/10)**2)# + (random.random() * 5))
    mag = max((500 - (5.2*(z + 7))**2 - ((Rx - 35)/0.8)**2 - ((Ry + 25)/0.8)**2), random.random() * 30)

    a.newMag((p,mag))
    p = a.next()


a.save_points(path)

print(time.time() - start)