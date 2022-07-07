import os
import sys
path = os.path.abspath('')
# print("======================")
# print(path)
sys.path.append(path)
from Pathfinders import *

path = os.path.abspath('')
print(path)

file_itr = 4

pathfinder = FullScan((0.8,1),20,16,16,path=path + f"\\Scans\\test_{file_itr}.json")
pathfinder.max_point = ((0,0,0),(0,0,0),500)


pathfinder.save_points(path + f"\\Scans\\test_5.json")