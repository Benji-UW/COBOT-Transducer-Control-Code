import sys
import os
import time
import json
import random
print(sys.path)

path = os.path.abspath('')
print("======================")
print(path)
sys.path.append(path + '\\Rebuilding_Everything')

print("======================")
print(sys.path)

import DemoPathfinder
from DemoPathfinder import *
a = FullScan((0.5,5),8,50,50)

#for i in range(24):
#    p = next(points)
#    print(p)
#    a.newMag((p,5))

p = a.next()
print(p)

while p is not 1:
    a.newMag((p,round(random.random()*1000)))
    p = a.next()

path = os.path.dirname(__file__)
#path = path + '\\' + time.ctime(time.time())
path = path + '\\ugh.json'
a.save_points(path)

