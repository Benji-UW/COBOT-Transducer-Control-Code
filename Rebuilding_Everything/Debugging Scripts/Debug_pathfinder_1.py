import sys
sys.path.insert(0,'C:/Users/ander/Documents/COBOT-Transducer-Control-Code/Rebuilding_Everything')

from DemoPathfinder import *
a = Pathfinder(8,45,45)

pointer = a.pointYielder()
print(next(pointer))

# well this is a piece of crap :/

