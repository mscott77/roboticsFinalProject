from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene
from obstacle import Obstacle


# basic tests
def test_1_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([3,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_2_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([0,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_3_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([6,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_4_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([9,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_5_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([12,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_6_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([14.25,0,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,[0,0],target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

# kissing obstacle test

# kissing obstacle in different joint configuration










# run tests from here to see visualization (if there is one)
if __name__ == "__main__":
    test_1_checkIfConfigIsInCollision()
        