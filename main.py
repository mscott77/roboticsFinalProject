from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene
from obstacle import Obstacle


#-----------user defined paramaters--------------------------*
# robot
link1_length = 6    # (meters)
link2_length = 6    # (meters)
# obstacles
obstacles = [Obstacle([4,4,0],2), Obstacle([10,-6,0], 2)]
# start config (q1,q2) for planar robot
qStart = [0,0]
# target point
target = (0,-12)
#------------------------------------------------------------*

# define robot arm based on user-defined parameters
dh = [  [0,0,link1_length,0],
        [0,0,link2_length,0]]
joint_types = ['r','r']
arm = kin.SerialArm(dh,joint_types)

navScene = NavigationScene(arm,obstacles,qStart,target)

navScene.drawScene()

navScene.animateSolution()



