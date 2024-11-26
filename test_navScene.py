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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

# kissing obstacle test
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
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

# bigger spheres for representing the arm (bigger link width) so that the circles show up in visulization
def test_601_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacle = Obstacle([14.5,0,0],2)
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# kissing obstacle in different joint configuration
def test_7_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacle = Obstacle([6,8.25,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
            [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# just shy of kissing
def test_701_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacle = Obstacle([6,8.26,0],2)
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
            [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# different link lengths, link width, object size. kissing
def test_8_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacle = Obstacle([5,2,0],0.5)
    linkWidth = 1
    #
    link1_length = 4 
    link2_length = 2

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

def test_801_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacle = Obstacle([5.2,2,0],0.5)
    linkWidth = 1
    #
    link1_length = 4 
    link2_length = 2

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacle,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# run tests from here to see visualization (if there is one)
if __name__ == "__main__":
    test_801_checkIfConfigIsInCollision()
        