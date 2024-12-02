from myLib.kinematics import kinematics as kin
from myLib.visualization.visualization import VizScene, ArmPlayer
import numpy as np
from navigationScene import NavigationScene, Solution
from obstacle import Obstacle


#---------------------------------------------basic collision tests----------------------------------------------
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





#------------------------------------------------multiple obstacles collision tests----------------------------------
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





#------------------------------- finding non-collision joint configuration for target position-----------------------
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




#----------------------------------------------graph construction and solving tests---------------------------------





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

# invalid target point (point coincides with obstacle)
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
    assert navScene._solution.message == "target point coicides with an obstacle. a valid solution could not be found"
    assert navScene._solutionWasFound == False


    if __name__ == "__main__":
        navScene.drawScene()
        navScene.drawScene(drawCollisionCircles=True)
        navScene.drawScene(drawArm=False,drawCollisionCircles=True)

# invalid target point (point not coinciding in an obstacle, but arm is in collision)
def test_10_02_findJointConfigForTarget():
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

    navScene.PRM(100)
    assert navScene._solution.jointConfigs == None
    assert navScene._solution.message == "could not find a valid arm configuration for the target position. a solution could not be found. try a different target position or move the obstacle(s)"
    assert navScene._solutionWasFound == False


    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)






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

# based on 
def pest_fullSolution_01():
    target = (3.5,9)
    startConfig = [0,0]
    obstacles = [Obstacle([6,8.25,0],2)]
    linkWidth = 1



    link_length = 6 
    dh = [  [0,0,link_length,0],
    [0,0,link_length,0]]
    joint_types = ['r','r']

    arm = kin.SerialArm(dh,joint_types)
    navScene = NavigationScene(arm,obstacles,startConfig,target,linkWidth)
    navScene.PRM(100)

    if __name__ == "__main__":
        navScene.drawScene(drawTargetConfig=True, drawArm=True, drawCollisionCircles=True)
        navScene.drawCspace()
        navScene.drawGraph()
        navScene.animateSolution()

#----------------------------------------------- messing around with networkx ---------------------------------------

def mess_networkx():

    import matplotlib.pyplot as plt
    from scipy.spatial import KDTree
    import networkx as nx

    # OPTION 1

    freePoints = [
        [-1,-1],
        [-1,2],
        [2,-1],
        [1,1],
        [1,2],
        [2,1],
    ]
    startPoint = [3,-2]
    endPoint = [-2,2]
    freePoints.insert(0, startPoint)
    freePoints.append(endPoint)

    # OPTION 2
    # import random
    # # Number of points to generate
    # num_points = 20
    # # Generate random points in the range [-2, 2]
    # freePoints = [[random.randint(-4, 4), random.randint(-4, 4)] for _ in range(num_points)]
    # print( freePoints)

    # OPTION 3
    # freePoints = [[-3, 1], [-1, 0], [0, -3], [-2, -1], [-3, -4], [-1, 0], [0, 1], [-2, 1], [-4, 2], [0, -4], [3, -3], [4, 4], [-1, 4], [-3, 2], [0, -3], [4, 4], [3, -4], [1, 3], [0, -4], [4, 3]]


    graph = nx.Graph()
    k = 3

    for i, point in enumerate(freePoints):
        graph.add_node(i, pos=point)

    tree = KDTree(freePoints)

    for i, point in enumerate(freePoints):
        distances, indices = tree.query(point, k=k+1)  # Include k+1 to skip self
        for neighbor_index in indices[1:]:  # Skip the first index (the point itself)
            graph.add_edge(i, neighbor_index)


    path = nx.shortest_path(graph, source=0, target=7)
    print("Shortest path from node 0 to node 7:", path)

    # Extract positions for visualization
    pos = nx.get_node_attributes(graph, "pos")

    # plot the graph only
    plt.figure(figsize=(8, 6))
    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_size=500,
        node_color="skyblue",
        edge_color="gray",
        font_size=10,
        font_weight="bold",
    )
    plt.title("Graph Visualization with Nodes and Edges", fontsize=16)
    plt.show()

    # plot the graph with the shortest path solution
    path_edges = list(zip(path, path[1:]))
    plt.figure(figsize=(8, 6))
    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_size=500,
        node_color="skyblue",
        edge_color="gray",
        font_size=10,
        font_weight="bold",
    )
    nx.draw_networkx_edges(
        graph,
        pos,
        edgelist=path_edges,
        width=2,
        edge_color="red",
    )

    plt.title("Graph with Shortest Path Highlighted", fontsize=16)
    plt.show()


#-------------------------------------------------------------------------------- MAIN -------------------------------------------------------------------------------------------------
# run tests from here to see visualization (if there is one)
if __name__ == "__main__":
    pest_fullSolution_01()       