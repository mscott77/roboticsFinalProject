from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene
from obstacle import Obstacle


#-----------user defined paramaters--------------------------*
# robot
link_length = 6    # (meters)
# obstacles
obstacle = Obstacle([2.125,3,0],2)
# start config (q1,q2) for planar robot
qStart = [0,0]
# target point
target = (0,-12)
#------------------------------------------------------------*

# define robot arm based on user-defined parameters
dh = [  [0,0,link_length,0],
        [0,0,link_length,0]]
joint_types = ['r','r']
arm = kin.SerialArm(dh,joint_types)

navScene = NavigationScene(arm,obstacle,qStart,target)

# navScene.drawScene()

isCollision = navScene.checkCircles(0,0,0,6)
print(f'collision occured: {isCollision}')

# navScene._solution = [
#     [10,10],
#     [20,10],
#     [30,0]
# ]*(np.pi/180)
# navScene.animateSolution()



