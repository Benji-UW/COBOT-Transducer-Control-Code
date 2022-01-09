import os
print(os.getcwd())
#os.chdir("Rebuilding_Everything")
print(os.getcwd())

from DemoPathfinder import *
a = Pathfinder(8,45,45)

pointer = a.pointYielder()
print(next(pointer))

# well this is a piece of crap :/