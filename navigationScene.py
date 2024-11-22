from kinematics.kinematics import SerialArm
from obstacle import Obstacle
from typing import List,Tuple
from myLib.visualization.visualization import VizScene
import time
import numpy as np

class NavigationScene():

    def __init__(self, arm: SerialArm, obstacle: Obstacle, start: List, target: Tuple[float,float]):
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
            end         - tuple (x,y) representing the goal COORDINATES (the point in space we want to bring the robot's end-effector to) 

        Notes:
            _function() - functions and variables starting with underscore are not intended to be access by the user
        """
        
        self._arm = arm
        self._obstacleCenter = obstacle.location
        self._obstacleRadius = obstacle.radius
        self._start = start
        self._target = target

        self._linkLength = self._arm.dh[0][2]
        self._linkWidth = 0.5

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

    def _checkIfConfigIsInCollision(qVals) -> bool:
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
        pass    # FIXME: pickup where you left off

    def checkCircles(self, x1, y1, x2, y2):
        # generate circles along the arm (use mx+b and basic trig to find points along line that represent center of circles
        numCircles = int(self._linkLength/(self._linkWidth/2))
        smallCircleRadius = self._linkWidth/2
        intermediatePoints = []
        linkCollision = 0
        for i in range(1, numCircles + 1):  # Divide the line into (num_points + 1) segments
            fraction = i / (numCircles)  # Fraction of the way along the line
            x_intermediate = x1 + fraction * (x2 - x1)
            y_intermediate = y1 + fraction * (y2 - y1)
            intermediatePoints.append((x_intermediate, y_intermediate))
        for j in range(numCircles-1):
            smallCircleCenter = intermediatePoints[j]
            if np.sqrt((self._obstacleCenter[0] - smallCircleCenter[0])**2 + (self._obstacleCenter[1] - smallCircleCenter[1])**2) <= smallCircleRadius +self._obstacleRadius:
                linkCollision = 1
		
        return linkCollision

    #-------------------------------------------------------------------------------- DRAWING AND ANIMATION ---------------------------------------------------------------------------------------------------
    def drawScene(self):
        """
        draws the initial position, any obstacles defined, and the target point in a visualization
        """
        viz = VizScene()
        viz.add_arm(self._arm, draw_frames=True)
        viz.add_obstacle(pos=self._obstacleCenter, rad=self._obstacleRadius, color=(0,0,0,1))
        # goal (even though we use the add_obstacle() function)
        viz.add_obstacle(pos = [self._target[0],self._target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))
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


        