from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene
from obstacle import Obstacle


def test_checkIfConfigIsInCollision_1():
    jointConfig = [0,0]
    obstacle = Obstacle([3,0,0],2)

    link_length = 6 
    dh = [  [0,0,link_length,0],
            [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,-12)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True