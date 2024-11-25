from myLib.kinematics.kinematics import SerialArm
from obstacle import Obstacle
from typing import List,Tuple
from myLib.visualization.visualization import VizScene
import time
import numpy as np
import random

class NavigationScene():

    def __init__(self, arm: SerialArm, obstacle: Obstacle, start: List, target: Tuple[float,float], linkWidth):
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
        self._obstacleCenter = obstacle.location
        self._obstacleRadius = obstacle.radius
        self._start = start
        self._target = target

        self._link1Length = self._arm.dh[0][2]
        self._link2Length = self._arm.dh[1][2]
        self._reachRadius = self._link1Length*self._link2Length
        self._linkWidth = linkWidth

        self._solutionWasFound = False
        self._solution = None

    #-------------------------------------------------------------------------------- PRM and helpers ---------------------------------------------------------------------------------------------------

    def PRM(self, numLearnPhasePoints: int) -> List[List[float]]:
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
        self._solutionWasFound = True    # FIXME: once you actually implement the algorithm, only set this to true if a solution actually was found

        c_space_non_collision_points = []
        c_space_collision_points = []


        for i in range(numLearnPhasePoints):
            coordinates = self.generatePointCoordinatesInCircle()
            # get joint angles for the given point
            coordinateArray = [coordinates[0], coordinates[1], 0]
            qs = self.arm.ik_position(coordinateArray, method='J_T')
            # check if it a collision point or not
            isCollision= self.checkCollision(qs)
            # make the point in C-space
            c_point = qs
            # decide whether to save the point or not
            if not isCollision:
                # add the point to a list of points
                c_space_non_collision_points.append(c_point)     # (c_space_non_collision_points  is a class variables)
            else:
                c_space_collision_points.append(c_point)   # you don't really need these to find the path, but they would be nice to have to visualize the c-space

    def generatePointCoordinatesInCircle(self):
        # generate a single point within the robot reach (may need to pass in reach radius, or access as a class variable
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

    def checkCircles(self, circles: List[Tuple[List[float],float]] ):
        """
        generates circles along the arm (use mx+b and basic trig to find points along line that represent center of circles
        """
        # innocent until proven guilty method
        linkCollision = 0

        # check the generated circles
        for circle in circles:
            armCircleCenterCoords = circle[0]
            armCircleRadius = circle[1]
            # c^2 = sqrt( a^2 + b^2)
            distBetweenCircleCenters = np.sqrt((self._obstacleCenter[0] - armCircleCenterCoords[0])**2 + (self._obstacleCenter[1] - armCircleCenterCoords[1])**2)
            if  distBetweenCircleCenters <= armCircleRadius + self._obstacleRadius:
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
            config = [self._start]

        viz = VizScene()
        if drawArm:
            viz.add_arm(self._arm, draw_frames=drawFrames)
            viz.update(qs=config)
        viz.add_obstacle(pos=self._obstacleCenter, rad=self._obstacleRadius, color=(0,0,0,1))
        # goal (even though we use the add_obstacle() function)
        viz.add_obstacle(pos = [self._target[0],self._target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))

        if drawCollisionCircles:
            pass
        # Plot intermediate circles
        # for i in range(len(self.intermediateCircles)):
        #     position = self.intermediateCircles[i][0]
        #     radius = self.intermediateCircles[i][1]
        #     viz.add_obstacle(pos=position, rad=radius, color = (0,0,0,1))
        viz.hold()

    def animateSolution(self, animationDelay: float=0.01):
        """
        animateSolution(0.01)

        assuming PRM() has been run and a solution has been found, this will animate each 
        """
        if self._solutionWasFound:
            print("animating solution")
            viz = VizScene()
            viz.add_arm(self._arm)
            for obstacle in self._obstacles:
                viz.add_obstacle(pos=obstacle.location, rad=obstacle.radius, color=(0,0,0,1))
            # goal (even though we use the add_obstacle() function)
            viz.add_obstacle(pos = [self._target[0],self._target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))

            for jointConfig in self._solution:
                viz.update(qs=[jointConfig])
                time.sleep(animationDelay)
        else:
            print("ERROR - no solution has been found. run PRM to find a solution")


        