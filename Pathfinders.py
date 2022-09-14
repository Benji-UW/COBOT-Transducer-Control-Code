'''
Unit convention: mm/kg/s/deg
'''
from argparse import ArgumentError
import numpy as np
import json
import time
import logging
import os

class Pathfinder:
    '''A class for controlling where the arm goes. The tl;dr is you define a search space,
    with a starting and ending value for each degree of freedom. It can do between one and
    six degrees of freedom. The units for the linear dimensions are mm, the units for the
    angular dimensions are degrees. Everything is defined relative to the initial starting
    position, so the initial position, as far as this is concerned, is ((0,0,0),(0deg,0deg,0deg))
    All internal points are stored as np arrays in the form [X,Y,Z,Rx,Ry,Rz]'''

    def __init__(self, z_range: float, Rx_range: float = 0., Ry_range: float =0,
            x_range: float = 0,y_range: float = 0,Rz_range: float = 0):
        '''Accepts as input a series of values indicating the range of different
        points in space it is allowed to exist between. This defines the search
        space of the object. It will not investigate any points outside these bounds.
        The only range it is require to accept is the z_range, otherwise it will
        default to zero. Z also defines the range to give more room to move backwards
        than forwards, in order to reduce the chances of the robot moving into the
        sample.'''
        # z_r = [-z_range, z_range]
        # Rx_r = [-Rx_range, Rx_range]
        # Ry_r = [-Ry_range, Ry_range]
        # x_r = [-x_range, x_range]
        # y_r = [-y_range, y_range]
        # Rz_r = [-Rz_range, Rz_range]
        self.range_of_motion = {'X': [-x_range, x_range],
                                'Y': [-y_range, y_range],
                                'Z': [-z_range, z_range],
                                'Rx': [-Rx_range, Rx_range],
                                'Ry': [-Ry_range, Ry_range],
                                'Rz': [-Rz_range, Rz_range]}

        self.active_rom: list[str] = []
        
        indices = {'X': 0,'Y': 1,'Z':2,'Rx':3,'Ry':4,'Rz':5,'mag':6}
        self.save_indices: list[int] = []

        '''Stores the character representations of the active degrees of freedom.
        e.g. ['Z', 'Rx', 'Ry']'''
        for degree in self.range_of_motion.keys():
            if self.range_of_motion[degree] != [0,0]:
                self.active_rom.append(degree)
                self.save_indices.append(indices[degree])
        
        self.save_indices.append(6)

        self.points: list[list[float]] = []
        '''Records the point/magnitude pairs that the pathfinder has been given
        in a tuple of the form [X,Y,Z,Rx,Ry,Rz,mag]'''

        self.yielder = self.internal_point_yielder()
        self.logger = logging.getLogger(__name__) 
        self.logger.info("Pathfinder initialized")

        file_itr = 0
        path = os.path.dirname(__file__)
        while os.path.exists(path + f"\\Scans\\test_{file_itr}.json"):
            file_itr += 1

        self.path = path + f'\\Scans\\test_{file_itr}.json'

        self.max_point: np.ndarray = np.ones(7) * -1
        '''max_point stores the point and magnitude of the highest
        magnitude yet scanned, stored in an np.ndarray of the form
        [X,Y,Z,Rx,Ry,Rz,mag], initialized to a (7,) array of -1s to
        distinguish from a real value.'''
        
        self.notes: str = "No notes passed from setup.\n"

    def __str__(self):
        return ("Basic, boilerplate version of a pathfinder module.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" + 
            f"\tHighest magnitude found: {self.max_point}")

    def newMag(self, point_mag:np.ndarray, override = False):
        '''Input ndarry in the form [X,Y,Z,Rx,Ry,Rz,mag].'''
        
        self.points.append(point_mag[self.save_indices].tolist())
        if (point_mag[6] > self.max_point[6]):
            self.max_point = point_mag.copy()

    def next(self) -> np.ndarray:
        return next(self.yielder)

    def progress_report(self) -> list[str]:
        return ["Progress report not implemented!"]

    def internal_point_yielder(self) -> np.ndarray:
        '''This method gets called when the pathfinder is ini tializes,
        and adds the center of the search space as well as all the
        corners to the "to-travel" list. When all the points have been
        visited it returns the integer 1 to indicate the search is
        complete.'''
        center_pt = np.array([np.mean(self.range_of_motion[a]) for 
                                a in self.range_of_motion.keys()])
        
        yield center_pt

        corners: set[np.ndarray] = set()
        q = (0,0,1)
        for i in range(64):
            pt:np.ndarray = np.array((
                self.range_of_motion['X'][q[((-1)**int(i))]],
                self.range_of_motion['Y'][q[((-1)**int(i/2))]],
                self.range_of_motion['Z'][q[((-1)**int(i/4))]],
                self.range_of_motion['Rx'][q[((-1)**int(i/8))]],
                self.range_of_motion['Ry'][q[((-1)**int(i/16))]],
                self.range_of_motion['Rz'][q[((-1)**int(i/32))]]))
            
            corners.add(pt)
        
        for pt in corners:
            yield pt

        yield 1

# TODO: Maybe delete the _close_enough method since it's been rendered obselete by the current functionality.        
    # def _close_enough(self, point, override, tolerance=(0.5,2)):
    #     '''Accepts as input a point and a tuple containing the dimensional tolerances,
    #     in the form of (mm, deg), where the first element is the toleranace of linear
    #     dimensions and the second is the angular tolerance. They default to 0.5 mm and 2 degrees.'''
    #     if override:
    #         return True
    #     for i in range(3):
    #         try:
    #             if abs(self.to_travel[0][0][i] - point[0][i]) > tolerance[0]:
    #                 return False
    #         except TypeError:
    #             print(self.to_travel[0])
    #             print(point)
    #             raise TypeError
    #         if abs(self.to_travel[0][1][i % 2] - point[1][i % 2]) > tolerance[1]:
    #             return False
    #     return True

    def save_points(self):
        '''Called at the end of the test or when the pathfinder has finished, outputs the points
        collected to a json file at a given path, meant to be superceded in each custom class
        in order to save additional information specific to that mode of pathfinder.'''
        json_data = { 
            'range of motion' : self.range_of_motion,
            'max_point' : self.max_point.tolist(),
            'points' : self.points
        }

        self.write_json_data(json_data)

    def write_json_data(self, data):
        self.notes += self.__str__()
        data['notes'] = self.notes
        data['active_ROM'] = self.active_rom

        with open(self.path, 'w+') as outfile:
            json.dump(data, outfile, indent=3)

class FullScan(Pathfinder):
    def __init__(self, resolution, z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''Initialize a pathfinder object that traverses the searchspace at resolution
        
        Acts almost identical to the regular pathfinder module, except it contains an additional
        field for the resolution of the scan. The resolution should be passed in as a tuple in the
        form (mm, deg) where the mm represents the linear mm tolerance for the full scan and the
        deg represents the angular degree tolerance for the full scan. There is a minimum tolerance
        based on the limitations of the robot, those are subject to change experimentally.'''
        min_tolerance = (0.1,0.5)
        self.resolution = (max(resolution[0],min_tolerance[0]), 
                            max(resolution[1],min_tolerance[1]))
        super().__init__(z_range,Rx_range,Ry_range,x_range,y_range,Rz_range)
        self.will_visit = 1
        self.start_time = time.time()
    
    def __str__(self):
        return ("Fullscan version of a pathfinder module, scans " + 
            "the entire search-space at a fixed resolution.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" +  
            f"\tResolution: {self.resolution}\n" + 
            f"\tHighest magnitude found: {self.max_point}")

    def internal_point_yielder(self) -> np.ndarray:
        '''The full scan iterates through every point in the searchspace'''
        res = self.resolution
        self.visited_so_far = 0
        (x0,x1) = self.range_of_motion['X']
        (y0,y1) = self.range_of_motion['Y']
        (z0,z1) = self.range_of_motion['Z']
        (Rx0,Rx1) = self.range_of_motion['Rx']
        (Ry0,Ry1) = self.range_of_motion['Ry']
        (Rz0,Rz1) = self.range_of_motion['Rz']

        # for x in np.linspace(x0, x1, int((x1-x0)/res[0])+1):
        #     for y in np.linspace(y0, y1, int((y1-y0)/res[0])+1):
        #         for z in np.linspace(z0, z1, abs(int((z1-z0)/res[0]))+1):
        #             for Rx in np.linspace(Rx0, Rx1, int((Rx1-Rx0)/res[1])+1):
        #                 (Ry0,Ry1) = (Ry1,Ry0)
        #                 for Ry in np.linspace(Ry0, Ry1, int(abs(Ry1-Ry0)/res[1]) + 1):
        #                     for Rz in np.linspace(Rz0, Rz1, int((Rz1-Rz0)/res[1]) + 1):
        #                         self.visited_so_far += 1
        #                         yield np.array([x,y,z,Rx,Ry,Rz])
        #             if (self.visited_so_far % 2000 == 0):
        #                 self.logger.info(f"Doing a dump of the latest 2000 points, at")
        #                 self.periodic_dump()
        # yield 1
        all_points = np.mgrid[
            x0:x1:(int((x1-x0)/res[0])+1)*1j,
            y0:y1:(int((y1-y0)/res[0])+1)*1j,
            z0:z1:(int((z1-z0)/res[0])+1)*1j,
            Rx0:Rx1:(int((Rx1-Rx0)/res[1])+1)*1j,
            Ry0:Ry1:(int((Ry1-x0)/res[1])+1)*1j,
            Rz0:Rz1:(int((Rz1-x0)/res[1])+1)*1j].reshape(6,-1,order='F').T
        
        # all_points = np.vstack((X.flatten(),Y.flatten(),Z.flatten(),
        #         Rx.flatten(),Ry.flatten(),Rz.flatten())).T

        self.will_visit = all_points.shape[0]
        
        for i in range(all_points.shape[0]):
            yield all_points[i]
            self.visited_so_far += 1
            if i % 2000 == 0:
                self.logger.info(f"Doing a dump of the latest 2000 points")
                # self.periodic_dump()

        yield 1

    def periodic_dump(self):
        now = time.time()
        
        with open(self.path, 'w') as outfile:
            json.dump(self.points, outfile, indent=3)
        self.logger.info(f"Periodic dumping of the points took {time.time() - now}")
        
    def progress_report(self) -> list[str]:
        now = time.time()
        elapsed = now - self.start_time
        projected = elapsed * (self.will_visit / self.visited_so_far)
        remaining = projected - elapsed

        finish_time = time.strftime("%H:%M:%S", time.localtime(self.start_time 
            + projected))
        remaining = time.strftime("%H:%M:%S", time.gmtime(remaining))
        # remaining = f"{int(remaining/3600)}:{int((remaining%3600)/60)}:{remaining%60:2.2f}"

        return [f"Visited {self.visited_so_far}/{self.will_visit} points. " + 
            f"({100 * self.visited_so_far/self.will_visit:1.2f}%)",
            f"Estimated completion time: {finish_time}", 
            f"Estimated time remaining: {remaining}"]

# TODO: See above, maybe delete this if no problems are caused.
    # def _close_enough(self, point, override, tolerance=(0.5,2)):
    #     tolerance = (min(self.resolution[0] / 2, tolerance[0]), min(self.resolution[1] / 2, tolerance[1]))
    #     popped = super()._close_enough(point, tolerance)
    #     if popped:
    #         try:
    #             self.to_travel.append(next(self.internal_yielder))
    #         except:
    #             self.to_travel.append(False)
    #     return popped

    def save_points(self):
        json_data = { 
            'range of motion' : self.range_of_motion, 
            'resolution' : self.resolution, 
            'max_point' : self.max_point.tolist(),
            'points' : self.points
        }
        self.write_json_data(json_data)

class DivisionSearch(Pathfinder):
    def __init__(self, divisions,z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0):
        '''Loads a pathfinder implementing a Division Search algorithm

        This pathfinder divides the searchspace into a set number of pieces (at least three),
        scans those points, and then defines another, smaller searchspace around the highest 
        value it finds of those points. The advantage of this is a very global search of the
        entire space, the downside is it winds up moving a lot. Remains to be seen if it's useful.
        
        Keyword arguments:
        divisions -- the number of divisions each axis of the search space 
            should be split into
        z_range -- The distance the pathfinder should move in either direction
            along the local z axis
        Rx/Ry/x/y/Rz_range -- Same as z_range in the other directions.
        '''
        self.divisions = divisions
        super().__init__(z_range,Rx_range,Ry_range,x_range,y_range,Rz_range)

    def __str__(self):
        return ("Division search pathfinder module.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" +
            f"\tDivisions: {self.divisions}\n" + 
            f"\tHighest magnitude found: {self.max_point}")

    def internal_point_yielder(self) -> np.ndarray:
        '''Divides the search space into n divisions along each of the active dimensions,
        checking each of them, then shrinks the search space to the box surrounding
        just the highest point, and then repeats until the problem converges to below
        the resolution of the robot.'''
        max_res = (0.05,0.5) # Maximum resolution of the robot (roughly) 0.05 mm and 0.5 deg (eyeballing)
        keep_going = True

        bounds = self.range_of_motion.copy()
        temp = dict()

        while keep_going:
            inc_size = dict()
            keys = bounds.keys()

            for DoF in keys:
                # If the bounds of this degree of freedom are not the same (this is a free axis):
                if bounds[DoF][0] != bounds[DoF][1]:
                    # Store all the points along this axis we will visit
                    temp[DoF] = np.linspace(bounds[DoF][0], bounds[DoF][1], self.divisions)

                    # Terminate the loop if the spacing between those points is too small
                    inc_size[DoF] = (bounds[DoF][1] - bounds[DoF][0]) / self.divisions
                    if (({'X','Y','Z'}.issuperset(DoF) and inc_size[DoF]
                            < max_res[0]) or ({'Rx','Ry','Rz'}.issuperset(DoF)
                            and inc_size[DoF] < max_res[1])):
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
                                    yield np.array((x,y,z,Rx,Ry,Rz))
            
            # Store the maxpoint in the temporary dictionary
            (temp['X'], temp['Y'], temp['Z'], temp['Rx'],
                temp['Ry'], temp['Rz']) = self.max_point[:6].tolist()

            for DoF in bounds.keys():
                # If this is an active degree of freedom, 
                if bounds[DoF][0] != bounds[DoF][1]:
                    # If the maxpoint exists in the fringe of the searchspace
                    if temp[DoF] == bounds[DoF][0]:
                        # Set the bounds to 
                        bounds[DoF] = [temp[DoF], temp[DoF] + (2 * inc_size[DoF])]
                    elif temp[DoF] == bounds[DoF][1]:
                        bounds[DoF] = [temp[DoF] - (2 * inc_size[DoF]), temp[DoF]]
                    else:
                        bounds[DoF] = [temp[DoF] - inc_size[DoF], temp[DoF] + inc_size[DoF]]

        yield 1

    def save_points(self):
        json_data = {
            'range of motion' : self.range_of_motion,
            'divisions' : self.divisions,
            'max_point' : self.max_point.tolist(),
            'points' : self.points
        }
        self.write_json_data(json_data)

class Discrete_degree(Pathfinder):
    '''This pathfinder uses a naive approximation of the search space where it optimizes
    one dimensions at a time, and loops until it converges on the apparent global max.'''
    def __init__(self,z_range=0,Rx_range=0,
                Ry_range=0,x_range=0, y_range=0,
                Rz_range=0,r_o_m=None,max_point=None):

        super().__init__(z_range, Rx_range, Ry_range, x_range, 
            y_range, Rz_range)
        
        if r_o_m is None:
            if z_range is None:
                raise ArgumentError("Please supply a working argument :/")
        else:
            self.range_of_motion = r_o_m

        #TODO Test the discrete degree module again and delete this code if possible
        # self.notes: str = "No notes passed from setup"

        # self.active_rom: list[str] = []
        
        # indices = {'X': 0,'Y': 1,'Z':2,'Rx':3,'Ry':4,'Rz':5,'mag':6}
        # self.save_indices: list[int] = []

        # '''Stores the character representations of the active degrees of freedom.
        # e.g. ['Z', 'Rx', 'Ry']'''
        # for degree in self.range_of_motion.keys():
        #     if self.range_of_motion[degree] != [0,0]:
        #         self.active_rom.append(degree)
        #         self.save_indices.append(indices[degree])
        
        # self.save_indices.append(6)

        # self.points: list[list[float]] = []
        # '''Records the point/magnitude pairs that the pathfinder has been given
        # in a tuple of the form [X,Y,Z,Rx,Ry,Rz,mag]'''

        # self.yielder = self.internal_point_yielder()
        # self.logger = logging.getLogger(__name__) 
        # self.logger.info("Pathfinder initialized")

        # file_itr = 0
        # path = os.path.dirname(__file__)
        # while os.path.exists(path + f"\\Scans\\test_{file_itr}.json"):
        #     file_itr += 1

        # self.path = path + f'\\Scans\\test_{file_itr}.json'

        # self.max_point: np.ndarray = np.ones(7) * -1
        

        '''max_point stores the point and magnitude of the highest magnitude yet
        scanned, stored in a ndarray in the form (X,Y,Z,Rx,Ry,Rz,mag),
        initialized to a (7,) array of -1s for distinction.'''
        if max_point is not None:
            self.max_point = max_point

    def __str__(self):
        return ("Discrete degree pathfinder module.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" +
            f"\tHighest magnitude found: {self.max_point}")
        
    def internal_point_yielder(self) -> np.ndarray:
        '''This yielder optimizes one degree of freedom at a time, looping in case
        optimizing along more than one direction isn't appropriate.
        
        Pseudocode:
        1. Start with an internal list of all of the active ranges of motion in this module
            1a. Set the current best to the center of the full range of motion
        2. Select one active axis and move along the entire range of motion in that
        direction. All of the non-active degrees of freedom are kept at their
        "current best" value.
        3. Save the highest magnitude from that span, and update that axis' "current best" value
        4. Move to the next degree of freedom and do the same thing'''
        starting_res = (2,5) # 2 mm, 5 deg
        max_res = (0.03,0.5)

        indices = {'X': 0, 'Y': 1, 'Z': 2, 'Rx': 3, 'Ry': 4, 'Rz': 5}
        current_best = dict()

        if self.max_point[6] != -1.0:
            for i in indices.keys():
                current_best[i] = self.max_point[indices[i]]
        else:
            for i in indices.keys():
                current_best[i] = np.mean(self.range_of_motion[i])

        i = 1
        while i < 9:
            last_best = current_best.copy()
            for slider in self.active_rom:
                if {'X','Y','Z'}.issuperset(slider):
                    res = max(starting_res[0] * (1 / (i**2)), max_res[0])
                else:
                    res = max(starting_res[1] * (1 / (i**2)), max_res[1])

                steps = int((self.range_of_motion[slider][1] - self.range_of_motion[slider][0]) / res)
                g = current_best.copy()

                for j in np.linspace(self.range_of_motion[slider][0] * (2/(2+i))
                                    ,self.range_of_motion[slider][1] * (2/ (2+i)), steps):
                    g[slider] = j
                    yield ((g['X'],g['Y'],g['Z']),(g['Rx'],g['Ry'],g['Rz']))

                current_best[slider] = self.max_point[indices[slider]]

            keep_going = False
            i += 1
            for k in last_best.keys():
                if abs(last_best[k] - current_best[k]) > 0.5:
                    keep_going = True
            if not keep_going:
                i = 8
        yield 1

class Greedy_discrete_degree(Pathfinder):
    '''This pathfinder uses a naive approximation of the search space where it optimizes
    one dimensions at a time, and loops until it converges on the apparent global max.
    Unlike the standard discrete degree, this one doesn't scan the entire space.'''
    def __init__(self, z_range, Rx_range=0, Ry_range=0,x_range=0,
                y_range=0, Rz_range=0,bias=10,steps=3,inc=1.6):
        super().__init__(z_range, Rx_range, Ry_range, x_range, y_range, Rz_range)
        self.bias=bias
        self.steps = steps
        self.inc = inc

    def __str__(self):
        return ("Greedy discrete degree pathfinder module.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" +
            f"\tBias: {self.bias}\n" + 
            f"\tSteps: {self.steps}\n" + 
            f"\tHighest magnitude found: {self.max_point}")
    
    def internal_point_yielder(self) -> np.ndarray:
        '''This yielder optimizes one degree of freedom at a time, looping in case
        optimizing along more than one direction isn't appropriate.

        min_tolerance = (0.1,0.5)
        '''
        # First find the magnitude at the origin (don't move)
        yield np.zeros(6)

        # Iterate through each D_o_f thrice
        l = len(self.active_rom)
        yield np.zeros(6)
        i = 0

        while self.inc > 0.1:
            if i%l == 0:
                self.inc = self.inc / 2
                self.logger.info(f"Increment size {self.inc}")
            # Set the starting point to the point with the current max magnitude
            t = self.max_point.copy()

            # Iterate through the active degrees of freedom
            axis = self.active_rom[i % l]
            # Start moving in the positive direction
            positive = False

            # Iterate self.steps in the positive direction, exploratory
            for i in range(self.steps):
                t = self.increment_appropriate_axis(t,axis,positive)
                yield t
            
            # If you went downhill twice in a row, that's a bunk direction
            while not self.recent_downhill():
                t = self.increment_appropriate_axis(t,axis,positive)
                yield t
            
            positive = True
            t = self.max_point.copy()

            # Iterate self.steps in the negative direction, exploratory
            for i in range(self.steps):
                t = self.increment_appropriate_axis(t,axis,positive)
                yield t

            # As long as you don't go downhill twice in a row, keep going in this direction
            while not self.recent_downhill():
                t = self.increment_appropriate_axis(t,axis,positive)
                yield t
            
            i += 1
            # Go back to start of the loop and try again with another degree of freedom.
        yield 1

    def recent_downhill(self) -> bool:
        '''Returns True if the pathfinder has gone downhill constantly for the past
        'self.steps' points
        Returns False if any of the past 'self.steps' points increased. '''

        for i in range(self.steps):
            if (self.points[-1 - i][-1] + self.bias > self.points[-2- i][-1]):
                return False
        
        return True

    def increment_appropriate_axis(self,max_point,axis,positive) \
            -> np.ndarray:
        try:
            (x,y,z,Rx,Ry,Rz)=max_point[:6].tolist()
        except ValueError as e:
            print(max_point)
            print(e)
        
        inc = self.inc

        if not positive:
            inc = inc * -1
        
        if axis=='X':
            x+=inc
        elif axis=='Y':
            y+=inc
        elif axis=='Z':
            z+=inc
        elif axis=='Rx':
            Rx+=inc
        elif axis=='Ry':
            Ry+=inc
        elif axis=='Rz':
            Rx+=inc

        return np.array((x,y,z,Rx,Ry,Rz))

class DivisionDiscreteDegree(Pathfinder):
    def __init__(self,divisions,z_range,Rx_range=0,Ry_range=0,x_range =0,y_range=0,Rz_range=0,cutoff_mag=-1):
        '''This pathfinder balances the breadth of search of the division search method while
        using the discrete degree method when a peak has been found'''
        self.divisions = divisions
        self.second_stage = -1
        self.cutoff_mag = cutoff_mag
        super().__init__(z_range,Rx_range,Ry_range,x_range,y_range,Rz_range)

    def __str__(self):
        return ("Division + discrete degree pathfinder module.\n" + 
            f"\tRange of motion: {self.range_of_motion}\n" +
            f"\tDivisions: {self.divisions}\n" + 
            f"\tCutoff magnitude: {self.cutoff_mag}\n" + 
            f"\tHighest magnitude found: {self.max_point}")

    def newMag(self, point_mag, override=False):
        if self.second_stage != -1:
            self.second_stage.newMag(point_mag, override)
        super().newMag(point_mag)

    def internal_point_yielder(self) -> np.ndarray:
        bounds = self.range_of_motion.copy()
        temp = dict()

        inc_size = dict()
        keys = bounds.keys()

        for DoF in keys:
            # If the bounds of this degree of freedom are not the same (this is a free axis):
            if bounds[DoF][0] != bounds[DoF][1]:
                # Store all the points along this axis we will visit
                temp[DoF] = np.linspace(bounds[DoF][0], bounds[DoF][1], self.divisions)
                inc_size[DoF] = (bounds[DoF][1] - bounds[DoF][0]) / self.divisions
            else:
                temp[DoF] = [bounds[DoF][0]]
                inc_size[DoF] = 0
        
        # Permute them
        for x in temp['X']:
            for y in temp['Y']:
                for z in temp['Z']:
                    for Rx in temp['Rx']:
                        for Ry in temp['Ry']:
                            for Rz in temp['Rz']:
                                if self.cutoff_mag == -1 or self.max_point[1] < self.cutoff_mag:
                                    yield np.array((x,y,z,Rx,Ry,Rz))
        
        # print(self.max_point)
        (temp['X'], temp['Y'], temp['Z'], 
            temp['Rx'], temp['Ry'], temp['Rz'],fk) = self.max_point.tolist()

        # print("======================")
        # print(f"Bounds: {bounds}")
        # print(f"Temp: {temp}")
        # print(f"Inc_size: {inc_size}")
        # print("======================")

        for DoF in bounds.keys():
            if bounds[DoF][0] != bounds[DoF][1]:
                if temp[DoF] == bounds[DoF][0]:
                    bounds[DoF] = [temp[DoF], temp[DoF] + (2 * inc_size[DoF])]
                elif temp[DoF] == bounds[DoF][1]:
                    bounds[DoF] = [temp[DoF] - (2 * inc_size[DoF]), temp[DoF]]
                else:
                    bounds[DoF] = [temp[DoF] - inc_size[DoF], temp[DoF] + inc_size[DoF]]

        m = ((np.mean(bounds['X']), np.mean(bounds['Y']),np.mean(bounds['Z'])),(np.mean(bounds['Rx']), np.mean(bounds['Ry']),np.mean(bounds['Rz'])))

        print(f"center of the second stage rom: {m}")
        print(f"Bounds of the second stage: {bounds}")
        print(inc_size)

        self.second_stage = Discrete_degree(r_o_m=bounds, max_point=self.max_point)
        p = self.second_stage.next()

        while p != 1:
            yield(p)
            p = self.second_stage.next()
        
        yield 1