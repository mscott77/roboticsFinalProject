from myLib.kinematics import kinematics as kin
from myLib.visualization import visualization as viz
import numpy as np
from obstacleScene import ObstacleScene


#-----------user defined paramaters-------------
link1_length = 6    # (inches)
link2_length = 6    # (inches)




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


viz.ArmPlayer(arm)
