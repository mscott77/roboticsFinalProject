from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from obstacleScene import ObstacleScene


#-----------user defined paramaters-------------
link1_length = 6    # (inches)
link2_length = 6    # (inches)
obstacleLocation = [4,4,0]
obstacleRadius = 2


scaleFactor = 2
dh = [
    [0,0,link1_length,0],
    [0,0,link2_length,0]
    ]
joint_types = ['r','r']

arm = kin.SerialArm(dh,joint_types)

q_init = [
    0,          # q1 initial
    0           # q2 initial
    ]


viz = VizScene()
viz.add_arm(arm,draw_frames=True)
viz.add_obstacle(obstacleLocation,rad=obstacleRadius)
viz.hold()
