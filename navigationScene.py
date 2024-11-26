from myLib.kinematics.kinematics import SerialArm
from obstacle import Obstacle
from typing import List,Tuple
from myLib.visualization.visualization import VizScene
import time
import numpy as np
import random
import math
import matplotlib.pyplot as plt

class Solution():
    def __init__(self, jointConfigs: List[List[float]]=None, message: str=None, c_space_collision_points=None, c_space_NONcollision_points=None):
        """
        if only a message is defined, it will be assumed no solution was found and all other properties will be set to None
        """
        self.jointConfigs = jointConfigs
        self.message = message
        self.c_space_collision_points = c_space_collision_points
        self.c_space_NONcollision_points = c_space_NONcollision_points

class NavigationScene():

    def __init__(self, arm: SerialArm, obstacles: List[Obstacle], start: List, target: Tuple[float,float], linkWidth):
        """
        navScene = NavigationScene(SerialArm(dh,jointTypes), [Obstacle([x,y,z], r), Obstacle(...)], [q1,q2], (x,y))

        Abstract:
            defines a "scene" containing the arm, obstacles, start config and end position for path finding using PRM algorithm
            one obstacle scene is only good for computing one path from a single set of start and end positions
            (if you want to find a path for a different start and end position, you need to make a new obstacle scene)  

        arguments:
            arm         - define the robot arm as outlined in the Serial Arm class
            obstacles   - a list of Obstacle objects
            start       - list of JOINT ANGLES that defines the starting configuration of the robot arm
            target      - tuple (x,y) representing the goal COORDINATES (the point in space we want to bring the robot's end-effector to) 
            linkWidth   - width of the links used when calculating collisions (generated circles will have diameter equal to linkWidth)

        Notes:
            _function() - functions and variables starting with underscore are not intended to be access by the user
        """
        
        self._arm = arm
        self._obstacles = obstacles
        # self._obstacleCenter = obstacle.location
        # self._obstacleRadius = obstacle.radius
        self._start = start
        self._target = target

        self._link1Length = self._arm.dh[0][2]
        self._link2Length = self._arm.dh[1][2]
        self._reachRadius = self._link1Length*self._link2Length
        self._linkWidth = linkWidth

        self._solutionWasFound = False
        self._solution: Solution = None




    #-------------------------------------------------------------------------------- PRM and helpers ---------------------------------------------------------------------------------------------------

    def PRM(self, numLearnPhasePoints: int=8000):
        """
        jointConfigsToReachTarget = PRM(500)  

        [qValsTime1, qValsTime2, ... ] = PRM(500)
        [[q1,q2], [q1,q2], [...], ...] = PRM(500)

        Abstract:
            runs the Probablistic Road Map path finding algorithm on the defined scene and
            returns the solution path from the defined starting configuration to the defined target destination while avoid all defined obstacles
            "solution path" is in the form of a list of joint configurations (a list of lists)

        Arguments:
            numLearnPhasePoints - the number of points to be generated during the learning phase

        Returns:
            a list of lists representing the solution path from start to target (if a solution was found)
            None - if a solution was not found
        """
        
        c_space_non_collision_points = []
        c_space_collision_points = []

        # before doing anything, make sure the start configuation is not in collision
        isStartConfigInCollision = self._checkIfConfigIsInCollision(self._start)
        if isStartConfigInCollision:
            self._solutionWasFound = False
            self._solution = Solution(message="start configuration is in collision. a valid solution could not be found")
            return
        # also make sure the goal is not in collision with an object
        isTargetPointInCollision = self._checkIfPointIsInCollision(self._target)
        if isTargetPointInCollision:
            self._solutionWasFound = False
            self._solution = Solution(message="target point is in collision. a valid solution could not be found")
            return

        # -------------------------- LEARNING PHASE ------------------------
        for i in range(numLearnPhasePoints):

            #----------option 1 - generate point in cartesian space then convert to joint angles------------------
            # coordinates = self.generatePointCoordinatesInCircle()
            # coordinateArray = [coordinates[0], coordinates[1], 0]
            # config = self.arm.ik_position(coordinateArray, method='J_T')

            #----------option 2 - generate "point"(joint angles) in c-space------------------
            q1 = random.uniform(-2*np.pi, 2*np.pi)
            q2 = random.uniform(-2*np.pi, 2*np.pi)
            config = [q1,q2]

            # check if it a collision point or not
            isCollision= self._checkIfConfigIsInCollision(config)
            # make the point in C-space
            c_point = config
            # decide whether to save the point or not
            if not isCollision:
                # add the point to a list of points
                c_space_non_collision_points.append(c_point)     # (c_space_non_collision_points  is a class variables)
            else:
                c_space_collision_points.append(c_point)   # you don't really need these to find the path, but they would be nice to have to visualize the c-space

        # ------------------------- PATH FINDING PHASE -----------------------

        # ------------------------- COMPOSE SOLUTION -----------------------------
        # FIXME: this is a fake solution
        self._solutionWasFound = True
        self._solution = Solution(
            jointConfigs=[[0,0], [0,np.pi/6], [0,np.pi/5], [0,np.pi/4], [0,np.pi/3], [0,np.pi/2]],
            message="pathfinding success",
            c_space_collision_points = c_space_collision_points,
            c_space_NONcollision_points = c_space_non_collision_points
        )



    # UNUSED
    def generatePointCoordinatesInCircle(self):
        # generate a single point within the robot reach
        while True:
            x = random.uniform(0 - self._reachRadius, 0 + self._reachRadius)
            y = random.uniform(0 - self._reachRadius, 0 + self._reachRadius)
                
            # Check if the point is inside the circle
            if (x - 0)**2 + (y - 0)**2 <= self._reachRadius**2:
                return x, y



    #----------------------------------------------------------------------------- COLLISION FUNCTIONS -------------------------------------------------------------------------------------
    def _checkIfConfigIsInCollision(self, jointConfig) -> bool:
        """
        Usage:  
            isCollision = _checkIfConfigIsInCollision([q1,q2])

        Abstract:
            pass in (single set of) joint angles, returns whether robot is in collision or not
            uses the obstacles and arm defined in the class to do calculations

        Arguments:
            qVals - a list of the joint angles for a single configuration

        Returns:
            boolean: isInCollision
        """
        
        circles = self.generateArmCircles(jointConfig)
        isInCollision = self.checkCircles(circles)

        return isInCollision

    def _checkIfPointIsInCollision(self, point: Tuple[float,float]):
        """
        checks if a single point is in collision with any of the obstacles. 
        basically the same as _checkIfConfigIsInCollision, it even calls the same checkCircles() function,
        but it calls checkCircles() and only passes in a single "circle" that represents the target
        the target circle has radius of .001 so that calculations can still be performed but the target circle acts more like a point with radius of zero
        """
        negligibleRadius = .001

        # a bit weird, but must be defiend as a list so that it works with the checkCircles function
        circles = [([point[0], point[1], 0], negligibleRadius)]
        isInCollision = self.checkCircles(circles)

        return isInCollision


    def checkCircles(self, armCircles: List[Tuple[List[float],float]] ):
        """
        generates circles along the arm (use mx+b and basic trig to find points along line that represent center of circles
        """
        # innocent until proven guilty method
        linkCollision = 0

        # check the generated circles
        for circle in armCircles:
            armCircleCenterCoords = circle[0]
            armCircleRadius = circle[1]
            # c^2 = sqrt( a^2 + b^2)
            for obstacle in self._obstacles:
                distBetweenCircleCenters = np.sqrt((obstacle.location[0] - armCircleCenterCoords[0])**2 + (obstacle.location[1] - armCircleCenterCoords[1])**2)
                if  distBetweenCircleCenters <= armCircleRadius + obstacle.radius:
                    linkCollision = 1
		
        return linkCollision

    def generateArmCircles(self, jointConfig) -> List[Tuple[List[float],float]]:
        """
        generates a list of circles that approximate the space that the arm takes up
        in order to check for collisions
        (generates for entire arm, joints 1 and 2)

        arguments:
            - jointConfig: joint angles of the arm 

        returns:
            - list of lists representing the coordinates of each circle
                [ ([x1,y1,z1], r1), ([x2,y2,z2], r2), (circle3), etc...]
            - radius of the circles
                (a single radius is used across the entire robot arm - assume consistent cross section of robot arm)
        """
        intermediateCircles = []

        x0 = 0
        y0 = 0
        T1 = self._arm.fk(jointConfig, 1)
        x1 = T1[0][3]
        y1 = T1[1][3]
        T2 = self._arm.fk(jointConfig)
        x2 = T2[0][3]
        y2 = T2[1][3]

        armCircleRadius = self._linkWidth/2
        #-------------link 1----------------------
        numCirclesL1 = int(self._link1Length/armCircleRadius) + 1
        for i in range(1, numCirclesL1 + 1):  # Divide the line into (num_points + 1) segments
            fraction = i / (numCirclesL1)  # Fraction of the way along the line
            x_intermediate = x0 + fraction * (x1 - x0)
            y_intermediate = y0 + fraction * (y1 - y0)
            intermediateCircles.append(([x_intermediate, y_intermediate,0],armCircleRadius))

        #-------------link 2----------------------
        numCircles = int(self._link2Length/armCircleRadius) + 1
        for i in range(1, numCircles + 1):  # Divide the line into (num_points + 1) segments
            fraction = i / (numCircles)  # Fraction of the way along the line
            x_intermediate = x1 + fraction * (x2 - x1)
            y_intermediate = y1 + fraction * (y2 - y1)
            intermediateCircles.append(([x_intermediate, y_intermediate,0],armCircleRadius))

        return intermediateCircles
    

    #-------------------------------------------------------------------------------- DRAWING AND ANIMATION ---------------------------------------------------------------------------------------------------
    def drawScene(self,jointConfig=None,drawFrames: bool=False, drawArm: bool=True, drawCollisionCircles: bool=False):
        """
        draws the initial position, any obstacles defined, and the target point in a visualization

        if you pass in a joint configuration, it will plot that configuration
        else it will use the starting configuration
        """
        if jointConfig:
            config = jointConfig
        else:
            config = self._start

        viz = VizScene()
        if drawArm:
            viz.add_arm(self._arm, draw_frames=drawFrames)
            viz.update(qs=config)
        # obstacle(s)
        for obstacle in self._obstacles:
            viz.add_obstacle(pos=obstacle.location, rad=obstacle.radius, color=(0.1,0.1,0.1,.25))
        # goal (even though we use the add_obstacle() function)
        viz.add_obstacle(pos = [self._target[0],self._target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))

        if drawCollisionCircles:
            circles = self.generateArmCircles(config)
            for circle in circles:
                viz.add_obstacle(pos=circle[0],rad=circle[1],color=(1,.25,0,.75))
        viz.hold()

    def drawCspace(self):
        if self._solutionWasFound:
            # Sample data
            list1 = self._solution.c_space_collision_points
            list2 = self._solution.c_space_NONcollision_points

            # Unpack the lists into x and y coordinates
            x1, y1 = zip(*list1)  # Unpacks [[x, y], [x, y], ...] into separate x and y
            x2, y2 = zip(*list2)

            # start config
            xStart = self._start[0]
            yStart = self._start[1]

            # Create a scatter plot
            plt.scatter(x1, y1, color='red', label='collision')
            plt.scatter(x2, y2, color='blue', label='free')
            plt.scatter(xStart, yStart, color='green', label='starting configuration')

            # Add labels, title, and legend
            plt.xlabel('q1')
            plt.ylabel('q2')
            plt.title('C-Space')
            plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=3)


            # Show the plot
            plt.show()  
        else:
            print("ERROR - no solution has been found. run PRM to find a solution")

    def animateSolution(self, animationDelay: float=0.5):
        """
        animateSolution(0.01)

        assuming PRM() has been run and a solution has been found, this will animate each 
        """
        if self._solutionWasFound:

            viz = VizScene()
            viz.add_arm(self._arm)

            # obstacles
            for obstacle in self._obstacles:
                viz.add_obstacle(pos=obstacle.location, rad=obstacle.radius, color=(0,0,0,1))

            # goal
            viz.add_obstacle(pos = [self._target[0],self._target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))

            # animate arm movement
            for jointConfig in self._solution.jointConfigs:
                viz.update(qs=[jointConfig])
                time.sleep(animationDelay)
            viz.hold()
        else:
            print("ERROR - no solution has been found. run PRM to find a solution")


        