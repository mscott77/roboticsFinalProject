from kinematics.kinematics import SerialArm
from obstacle import Obstacle
from typing import List,Tuple
from myLib.visualization.visualization import VizScene
import time

class NavigationScene():

    def __init__(self, arm: SerialArm, obstacles: List[Obstacle], start: List, target: Tuple[float,float]):
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
        """
        
        self.arm = arm
        self.obstacles = obstacles
        self.start = start
        self.target = target

        self.solutionWasFound = False
        self.solution = None

    def drawScene(self):
        """
        draws the initial position, any obstacles defined, and the target point in a visualization
        """
        viz = VizScene()
        viz.add_arm(self.arm, draw_frames=True)
        for obstacle in self.obstacles:
            viz.add_obstacle(pos=obstacle.location, rad=obstacle.radius, color=(0,0,0,1))
        # goal (even though we use the add_obstacle() function)
        viz.add_obstacle(pos = [self.target[0],self.target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))
        viz.hold()

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
        self.SolutionWasFound = True    # FIXME: once you actually implement the algorithm, only set this to true if a solution actually was found


    def animateSolution(self, animationDelay: float=0.01):
        """
        animateSolution(0.01)

        assuming PRM() has been run and a solution has been found, this will animate each 
        """
        if self.solutionWasFound:
            print("animating solution")
            viz = VizScene()
            viz.add_arm(self.arm)
            for obstacle in self.obstacles:
                viz.add_obstacle(pos=obstacle.location, rad=obstacle.radius, color=(0,0,0,1))
            # goal (even though we use the add_obstacle() function)
            viz.add_obstacle(pos = [self.target[0],self.target[1],0], rad=0.5, color=(0, 0.8, 0, 0.75))

            for jointConfig in self.solution:
                viz.update(qs=[jointConfig])
                time.sleep(animationDelay)
        else:
            print("ERROR - no solution has been found. run PRM to find a solution")