import sys
import os
import time
import json
import random
sys.path.insert(0,'C:/Users/ander/Documents/COBOT-Transducer-Control-Code/Rebuilding_Everything')

from DemoPathfinder import *

a = FullScan((0.5,5),8,50,50)

#for i in range(24):
#    p = next(points)
#    print(p)
#    a.newMag((p,5))

p = a.next()

while p is not False:
    a.newMag((p,round(random.random()*1000)))
    p = a.next()

path = os.path.dirname(__file__)
#path = path + '\\' + time.ctime(time.time())
path = path + '\\ugh.json'
a.save_points(path)

