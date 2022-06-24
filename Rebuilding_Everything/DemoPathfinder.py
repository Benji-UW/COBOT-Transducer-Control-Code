'''
Unit convention: mm/kg/s/deg
'''
import math
from xml.sax.xmlreader import IncrementalParser
import numpy as np
import json

# from regex import D

class Pathfinder:
    '''A class for controlling where the arm goes. The tl;dr is you define a search space,
    with a starting and ending value for each degree of freedom. It can do between one and
    six degrees of freedom. The units for the linear dimensions are mm, the units for the
    angular dimensions are degrees. Everything is defined relative to the initial starting
    position, so the initial position, as far as this is concerned, is ((0,0,0),(0deg,0deg,0deg))
    All internal points are stored as tuples in the form ((X,Y,Z),(Rx,Ry,Rz))'''

    def __init__(self,z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''Accepts as input a series of values indicating the range of different
        points in space it is allowed to exist between. This defines the search
        space of the object. It will not investigate any points outside these bounds.
        The only range it is require to accept is the z_range, otherwise it will
        default to zero. Z also defines the range to give more room to move backwards
        than forwards, in order to reduce the chances of the robot moving into the
        sample.'''
        z_r = [-z_range * 1.25, z_range * 0.75]
        Rx_r = [-Rx_range, Rx_range]
        Ry_r = [-Ry_range, Ry_range]
        x_r = [-x_range, x_range]
        y_r = [-y_range, y_range]
        Rz_r = [-Rz_range, Rz_range]
        self.range_of_motion = {'X': x_r,'Y': y_r,'Z':z_r,'Rx':Rx_r,'Ry':Ry_r,'Rz':Rz_r}

        self.points = []
        self.to_travel = []

        self.yielder = self.internal_point_yielder()

        '''max_point stores the point and magnitude of the highest magnitude yet scanned,
        stored in a tuple of the form (((X,Y,Z, (Rx,Ry,Rz)), mag), initialized to (-1,-1,-1)
        for simplicity.'''
        self.max_point = (-1,-1,-1)

        '''Sets the degrees of freedom of this pathfinder, only Z defaults to true.'''
        self.degrees_of_freedom = {'X': x_range!=0,'Y': y_range!=0,'Z':z_range!=0,'Rx':Rx_range!=0,'Ry':Ry_range!=0,'Rz':Rz_range!=0}

    def newMag(self, point_mag, override = False):
        '''Accepts as in put a tuple in the form (((X,Y,Z),(Rx,Ry,Rz)), mag), 
        where the first element in the tuple is a 6-member tuple representing
        the point in 6D space, and the second element is a float representing
        the signal magnitude at that point.'''
        self.points.append(point_mag)
        if (point_mag[1] > self.max_point[1]):
            self.max_point = point_mag
        
        latest_point = point_mag[0]

    def next(self):
        return next(self.yielder)

    def internal_point_yielder(self):
        '''This method gets called when the pathfinder is initializes, and adds the center of 
        the search space as well as all the corners to the "to-travel" list.'''
        center_pt = [np.mean(self.range_of_motion[a]) for a in self.range_of_motion.keys()]
        self.to_travel.append((tuple(center_pt[:3]),tuple(center_pt[3:])))

        corners = set()
        q = (0,0,1)

        for i in range(64):
            pos = (self.range_of_motion['X'][q[((-1)**int(i))]],self.range_of_motion['Y'][q[((-1)**int(i/2))]],self.range_of_motion['Z'][q[((-1)**int(i/4))]])
            ang = (self.range_of_motion['Rx'][q[((-1)**int(i/8))]],self.range_of_motion['Ry'][q[((-1)**int(i/16))]],self.range_of_motion['Rz'][q[((-1)**int(i/32))]])
            
            corners.add((pos,ang))
        
        for pt in corners:
            yield pt
            # self.to_travel.append(pt)
            # print(pt)

        yield 1
        
    def close_enough(self, point, override, tolerance=(0.5,2)):
        '''Accepts as input a point and a tuple containing the dimensional tolerances,
        in the form of (mm, deg), where the first element is the toleranace of linear
        dimensions and the second is the angular tolerance. They default to 0.5 mm and 2 degrees.'''
        if override:
            return True
        for i in range(3):
            try:
                if abs(self.to_travel[0][0][i] - point[0][i]) > tolerance[0]:
                    return False
            except TypeError:
                print(self.to_travel[0])
                print(point)
                raise TypeError
            if abs(self.to_travel[0][1][i % 2] - point[1][i % 2]) > tolerance[1]:
                return False
        return True

#TODO: there are some points coming through to close-enough that are wrapped up in too many tuples
    def save_points(self, path):
        '''Called at the end of the test or when the pathfinder has finished, outputs the points
        collected to a json file at a given path, meant to be superceded in each custom class
        in order to save additional information specific to that mode of pathfinder.'''
        with open(path, 'a+') as outfile:
            json.dump(self.points, outfile, indent=4)

class FullScan(Pathfinder):
    def __init__(self, resolution, z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''Acts almost identical to the regular pathfinder module, except it contains an additional
        field for the resolution of the scan. The resolution should be passed in as a tuple in the
        form (mm, deg) where the mm represents the linear mm tolerance for the full scan and the
        deg represents the angular degree tolerance for the full scan. There is a minimum tolerance
        based on the limitations of the robot, those are subject to change experimentally.'''
        # self.yielder = self.internal_point_yielder()
        min_tolerance = (0.2, 2)
        self.resolution = (max(resolution[0],min_tolerance[0]), max(resolution[1],min_tolerance[1]))
        super().__init__(z_range,Rx_range,Ry_range,x_range,y_range,Rz_range)
        
    def starting_point_loader(self):
        '''For object-oriented reasons I can't go back '''
        for i in range(10):
            self.to_travel.append(next(self.internal_yielder))

    def internal_point_yielder(self):
        '''The full scan iterates through every point in the searchspace'''
        r_o_m = self.range_of_motion
        res = self.resolution
        forwards = True
        for x in np.linspace(r_o_m['X'][0], r_o_m['X'][1], int((r_o_m['X'][1]-r_o_m['X'][0])/res[0]) + 1):
            for y in np.linspace(r_o_m['Y'][0], r_o_m['Y'][1], int((r_o_m['Y'][1]-r_o_m['Y'][0])/res[0]) + 1):
                for z in np.linspace(r_o_m['Z'][0], r_o_m['Z'][1], abs(int((r_o_m['Z'][1]-r_o_m['Z'][0])/res[0])) + 1):
                    for Rx in np.linspace(r_o_m['Rx'][0], r_o_m['Rx'][1], int((r_o_m['Rx'][1]-r_o_m['Rx'][0])/res[1]) + 1):
                        forwards = not forwards
                        for Ry in np.linspace(r_o_m['Ry'][forwards], r_o_m['Ry'][not forwards], int((r_o_m['Ry'][1]-r_o_m['Ry'][0])/res[1]) + 1):
                            for Rz in np.linspace(r_o_m['Rz'][0], r_o_m['Rz'][1], int((r_o_m['Rz'][1]-r_o_m['Rz'][0])/res[1]) + 1):
                                yield ((x,y,z),(Rx,Ry,Rz))
        yield 1   
        
    def close_enough(self, point, override, tolerance=(0.5,2)):
        tolerance = (min(self.resolution[0] / 2, tolerance[0]), min(self.resolution[1] / 2, tolerance[1]))
        popped = super().close_enough(point, tolerance)
        if popped:
            try:
                self.to_travel.append(next(self.internal_yielder))
            except:
                self.to_travel.append(False)
        return popped

    def save_points(self, path):
        json_data = { \
            'range of motion' : self.range_of_motion, \
            'resolution' : self.resolution, \
            'points' : self.points\
        } 

        with open(path, 'a+') as outfile:
            json.dump(json_data, outfile, indent=3)

class DivisionSearch(Pathfinder):
    def __init__(self, divisions,z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''This pathfinder divides the searchspace into a set number of pieces (at least three),
        scans those points, and then defines another, smaller searchspace around the highest 
        value it finds of those points. The advantage of this is a very global search of the
        entire space, the downside is it winds up moving a lot. Remains to be seen if it's useful.'''
        self.divisions = divisions
        super().__init__(z_range,Rx_range,Ry_range,x_range,y_range,Rz_range)
        self.yielder = self.division_search(divisions)

    def division_search(self, divisions):
        max_res = (0.05,0.5) # Maximum resolution of the robot (roughly) 0.05 mm and 0.5 deg (eyeballing)
        keep_going = True

        # Alternate psuedocode
        """ Current implementation uses np.linspace to generate a dictionary
        of temp for each of the degrees of freedom. For instance with a 
        10x10x10 mm search vol with 5 divisions, it'll create
        X: -5,-2.5,0,2.5,5
        Y: ',','
        Z: ',','
        And then in a later nested loop, a new point is created for each permuation
        of those divisions.
        
        Alternatively, I could define 'temp' to represent the spacing between
        each stop, this would allow regular checking that the maximum resolution hasn't
        been reached."""


        # Pseudocode
        bounds = self.range_of_motion.copy()
        temp = dict()

        while keep_going:
            # print("I'm in")
            # print(bounds)
            inc_size = dict()
            keys = bounds.keys()

            for DoF in keys:
                # If the bounds of this degree of freedom are not the same (this is a free axis):
                if bounds[DoF][0] != bounds[DoF][1]:
                    # Store all the points along this axis we will visit
                    temp[DoF] = np.linspace(bounds[DoF][0], bounds[DoF][1], divisions)
                    # Terminate the loop if the spacing between those points is too small
                    inc_size[DoF] = (bounds[DoF][1] - bounds[DoF][0]) / divisions
                    if ({'X','Y','Z'}.issuperset(DoF) and inc_size[DoF] < max_res[0]) or ({'Rx','Ry','Rz'}.issuperset(DoF) and inc_size[DoF] < max_res[1]):
                        keep_going = False
                else:
                    # Otherwise keep that bound where it is (should be zero)
                    temp[DoF] = [bounds[DoF][0]]
            
            # Permute them
            for x in temp['X']:
                for y in temp['Y']:
                    for z in temp['Z']:
                        for Rx in temp['Rx']:
                            for Ry in temp['Ry']:
                                for Rz in temp['Rz']:
                                    yield ((x,y,z), (Rx,Ry,Rz))
                                    # all_points_visited_this_round.add(((x,y,z), (Rx,Ry,Rz)))
            
            # while len(all_points_visited_this_round) > 0:
            #     yield all_points_visited_this_round.pop()

            # Represents the 6D point that was highest among the previously scanned
            # ((X,Y,Z), (Rx,Ry,Rz))
            ((temp['X'], temp['Y'], temp['Z']), (temp['Rx'], temp['Ry'], temp['Rz'])) = self.max_point[0]

            for DoF in bounds.keys():
                if bounds[DoF][0] != bounds[DoF][1]:
                    if temp[DoF] == bounds[DoF][0]:
                        bounds[DoF] = [temp[DoF], temp[DoF] + (2 * inc_size[DoF])]
                    elif temp[DoF] == bounds[DoF][1]:
                        bounds[DoF] = [temp[DoF] - (2 * inc_size[DoF]), temp[DoF]]
                    else:
                        bounds[DoF] = [temp[DoF] - inc_size[DoF], temp[DoF] + inc_size[DoF]]

        yield 1
        # divide up the working boundaries by #divisions
        # Iterate through them, center the new search around the highest value
        #       That's gonna require storing the max amplitude so far
        # Set the new working boundaries to the divisions next to the highest
        # Repeat until the space between divisions is smaller than
        
    def next(self):
        return next(self.yielder)

    def save_points(self, path):
        json_data = { \
            'range of motion' : self.range_of_motion, \
            'divisions' : self.divisions, \
            'points' : self.points\
        } 

        with open(path, 'a+') as outfile:
            json.dump(json_data, outfile, indent=3)