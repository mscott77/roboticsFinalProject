from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene, Solution
from obstacle import Obstacle


#-------------------basic collision tests---------------
def test_1_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([3,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_2_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([0,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_3_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([6,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_4_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([9,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

def test_5_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([12,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

# kissing obstacle test
def test_6_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([14.25,0,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()

# bigger spheres for representing the arm (bigger link width) so that the circles show up in visulization
def test_6_01_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([14.5,0,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# kissing obstacle in different joint configuration
def test_7_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([6,8.25,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
            [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# just shy of kissing
def test_7_01_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([6,8.26,0],2)]
    linkWidth = 0.5

    link_length = 6 
    dh = [  [0,0,link_length,0],
            [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# different link lengths, link width, object size. kissing
def test_8_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([5,2,0],0.5)]
    linkWidth = 1
    #
    link1_length = 4 
    link2_length = 2

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

def test_8_01_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([5.2,2,0],0.5)]
    linkWidth = 1
    #
    link1_length = 4 
    link2_length = 2

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

#-----------------multiple obstacles collision tests-------------
def test_9_checkIfConfigIsInCollision():
    jointConfig = [0,0]
    obstacles = [Obstacle([13.5,0,0],1), Obstacle([6,8.5,0],2)]
    linkWidth = 1
    #
    link1_length = 6
    link2_length = 6

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

def test_9_01_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([13.5,0,0],1), Obstacle([6,8.5,0],2)]
    linkWidth = 1
    #
    link1_length = 6
    link2_length = 6

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == True


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

def test_9_02_checkIfConfigIsInCollision():
    jointConfig = [0,np.pi/4]
    obstacles = [Obstacle([13.5,0,0],1), Obstacle([6,8.5,0],2)]
    linkWidth = 1
    #
    link1_length = 6
    link2_length = 6

    dh = [  [0,0,link1_length,0],
            [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    isCollision = navScene._checkIfConfigIsInCollision(jointConfig)
    assert isCollision == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

#---------------- finding non-collision joint configuration for target position-----------------

# valid
def test_11_findJointConfigForTarget():
    target = (6,6)
    jointConfig = [0,0]
    obstacles = [Obstacle([3,-3,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    assert targetJointConfig is not None, "couldn't find a joint configuration for the target position"
    print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True)

# valid
def test_11_01_findJointConfigForTarget():
    target = (4,3)
    jointConfig = [0,0]
    obstacles = [Obstacle([3,-3,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    assert targetJointConfig is not None, "couldn't find a joint configuration for the target position"
    print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True)

# same as 11_01 but an obstacle is in the way so second joint config must be used
def test_11_02_findJointConfigForTarget():
    target = (4,3)
    jointConfig = [0,0]
    obstacles = [Obstacle([0,3,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    assert targetJointConfig is not None, "couldn't find a joint configuration for the target position"
    print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)

# same as 11_02 but two obstacles are in the way, so no joint configuration can be found
def test_11_03_findJointConfigForTarget():
    target = (4,3)
    jointConfig = [0,0]
    obstacles = [Obstacle([0,3,0],2), Obstacle([3,-2.25,0],1.25)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    assert targetJointConfig is None, "progam wasn't supposed to be able to find a solution configuration with the given obstacles"
    # print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)

# based on test 07 but with a target
def test_11_07_findJointConfigForTarget():
    target = (9,7)
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([6,8.25,0],2)]
    linkWidth = 1



    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    assert targetJointConfig is not None, "couldn't find a joint configuration for the target position"
    print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)

# 11_07 but target is mirrored ( this tests the programs ability to check both of the two possible configurations)
def test_11_07_01_findJointConfigForTarget():
    target = (3.5,9)
    jointConfig = [0,np.pi/2]
    obstacles = [Obstacle([6,8.25,0],2)]
    linkWidth = 1



    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    targetJointConfig = navScene._findJointConfigForTarget()
    #assert targetJointConfig is not None, "couldn't find a joint configuration for the target position"
    print(f'target joint config (rads) = {targetJointConfig}')


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)


#----------------------------------------------- PRM tests ----------------------------------------------------

# same parameters as test 6
# invalid starting position
def test_10_PRM_intialCollisionCheck():
    jointConfig = [0,0]
    obstacles = [Obstacle([14.5,0,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    navScene.PRM(100)
    assert navScene._solution.jointConfigs == None
    assert navScene._solution.message == "start configuration is in collision. a valid solution could not be found"
    assert navScene._solutionWasFound == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# invalid target point
def test_10_01_PRM_intialCollisionCheck():
    target = (6,6)
    jointConfig = [0,0]
    obstacles = [Obstacle([6,6,0],2)]
    linkWidth = 1

    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,jointConfig,target,linkWidth)

    navScene.PRM(100)
    assert navScene._solution.jointConfigs == None
    assert navScene._solution.message == "target point is in collision. a valid solution could not be found"
    assert navScene._solutionWasFound == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

#---------------------------------------------- animation and plotting "tests" -----------------------------------
# but they're not actually unit tests/pytests.

def pest_animateSolution():
    startingConfig = [0,0]
    obstacles = [Obstacle([6,6,0],2)]
    linkWidth = 0.5

    link1_length = 6 
    link2_length = 6
    dh = [  
        [0,0,link1_length,0],
        [0,0,link2_length,0]]
    joint_types = ['r','r']

    target = (0,6)
    arm = kin.SerialArm(dh, joint_types)
    navScene = NavigationScene(arm, obstacles, startingConfig, target, linkWidth) 
    navScene.PRM(10)
    navScene.animateSolution()

# must rotate to negative joint angles in order to get to target
def pest_drawCspace_negativeRotationNecessary():
    startingConfig = [0,0]
    obstacles = [Obstacle([6,6,0],2)]
    target = (0,6)
    linkWidth = 0.5

    link1_length = 6 
    link2_length = 6
    dh = [  
        [0,0,link1_length,0],
        [0,0,link2_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh, joint_types)
    navScene = NavigationScene(arm, obstacles, startingConfig, target, linkWidth) 
    navScene.PRM()
    navScene.drawCspace()
    navScene.drawScene()

# no possible solution
def pest_drawCspace_noSolution():
    startingConfig = [0,0]
    obstacles = [Obstacle([4,4,0],2), Obstacle([4,-4,0],2)]
    target = (0,6)
    linkWidth = 0.5

    link1_length = 6 
    link2_length = 6
    dh = [  
        [0,0,link1_length,0],
        [0,0,link2_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh, joint_types)
    navScene = NavigationScene(arm, obstacles, startingConfig, target, linkWidth) 
    navScene.PRM()
    navScene.drawCspace()
    navScene.drawScene()

#-------------------------------------------------------------------------------- MAIN -------------------------------------------------------------------------------------------------
# run tests from here to see visualization (if there is one)
if __name__ == "__main__":
    test_11_07_01_findJointConfigForTarget()       