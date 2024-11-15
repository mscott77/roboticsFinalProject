"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022 and Sept 21, 2023
"""

from transforms.transforms import *
import numpy as np
from numpy.linalg import norm
import random

eye = np.eye(4)
pi = np.pi


# this is a convenience class that makes it easy to define a function that calculates "A_i(q)", given the
# DH parameters for link and joint "i" only. 
# FIXME: this class is used like a function, not sure why it needs to be
# a class it just makes things more confusing because when you use it you have
# to do f.A to access the function (see SerialArm.__init__())
class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts a list of 4 dh parameters corresponding to the transformation for one link 
    and returns a function "f" that will generate a homogeneous transform "A" given 
    "q" as an input. A represents the transform from link i-1 to link i. This follows
    the "standard" DH convention. 

    Parameters:
    dh - 1 x 4 list from dh parameter table for one transform from link i-1 to link i,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the homogeneous transform 
        from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            # although A(q) is only a function of "q", the dh parameters are available to these next functions 
            # because they are passed into the "init" function above. 

            def A(q): 
                # See eq. (2.52), pg. 64

                theta = dh[0] + q       # revolute joint - q is along the z axis according to DH convention so add it to theta
                d = dh[1]
                a = dh[2]
                alpha = dh[3]

                return DH_Transform(theta,d,a,alpha)




        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                # See eq. (2.52), pg. 64

                theta = dh[0]
                d = dh[1] + q       # prismatic joint - according to DH convention q is offset along z axis, so add do d (Translation along z)
                a = dh[2]
                alpha = dh[3]

                return DH_Transform(theta,d,a,alpha)




        self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """


    def __init__(self, dh_params, jt=None, base=eye, tip=eye, joint_limits=None):
        """
        arm = SerialArm(dh_params, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list where each entry in list is another list of length 4, representing dh parameters, [theta d a alpha]
        :param jt: n length list of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy array representing SE3 transform from world or inertial frame to frame 0
        :param tip: 4x4 numpy array representing SE3 transform from frame n to tool frame or tip of robot
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        self.dh = dh_params
        self.n = len(dh_params)

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # using the code we wrote above to generate the function A(q) for each set of DH parameters
        for i in range(self.n):
            f = dh2AFunc(self.dh[i], self.jt[i])        # make a dh2A object    (this will internally generate the function 'A' as an attribute of the object)
            self.transforms.append(f.A)                 # append the dh2A object's 'A' attribute (the function) to the list of transform functions 'transforms'


        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits

        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][0]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[0], dh[2]]))


    def fk(self, q, index=None, base: bool=False, tip: bool=False) -> np.array:
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index: integer or list of two integers. 
                    - If a list of two integers, the first integer represents the starting JOINT 
                      (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    - If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                      the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        ###############################################################################################
        # the following lines of code are data type and error checking. You don't need to understand
        # all of it, but it is helpful to keep. 

        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        ###############################################################################################        
        ###############################################################################################


        T = np.eye(4)
        if (start_frame == 0) & (base == True):
            T = T @ self.base

        for i in range(start_frame,end_frame):
            funct = self.transforms[i]
            Ti = funct(q[i])
            T = T @ Ti

        if (end_frame == self.n) & (tip==True):
            T = T @ self.tip

        return T
    
    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the frame defined by "index" (index=POI), which corresponds
        to a frame on the arm, with the arm in a given configuration defined by "q"

        Parameters:
        q - list or numpy array of joint positions of length N
        index - integer, the joint frame at which to calculate the Jacobian 
            (what joint/frame are you trying to find the twist of?)
            (usually this is the end effector)
        index - the POI 

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """


        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        N = len(q)

        # start by declaring a zero matrix that is the correct size for the Jacobian
        J = np.zeros((6,N))

        # find the current position of the point of interest (usually origin of frame "n") 
        # using your fk function 
        # ( "current position of POI" = Opoi_in_0 - it's used any time you want to find Jv for a revolute joint)
        # ( it's used more than once so that's why we calculate it outside of the for loop)
        R = self.fk(q,index,base,tip)
        Opoi_in_0 = R[:3,3]                 


        # calculate all the necessary values using your "fk" function, and fill every column
        # of the jacobian using this "for" loop. Functions like "np.cross" may also be useful. 
        # ( you need to use fk because fk(index-1) will give you Zi-1_in_0 and Oi-1_in_0 )
        # ( for simplicity I will just use R,z,O but it's really Ri-1_in_0, Zi-1_in_0, etc.)
        for i in range(0,index):

            R = self.fk(q, i, base, tip)
            z = R[:3,2]
            O = R[:3,3]

            # revolute
            if self.jt[i] == 'r':
                Jvi = np.cross(z,(Opoi_in_0 - O))
                Jwi = z
                J[:3,i] = Jvi
                J[3:,i] = Jwi


            # prismatic
            else:
                Jvi = z
                Jwi = np.array([0,0,0])   # (3x1 column vector)
                J[:3,i] = Jvi
                J[3:,i] = Jwi   

        return J


    # You don't need to touch this function, but it is helpful to be able to "print" a description about
    # the robot that you make.
    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string
    
    #-------------functions specific to ik_position()----------------
    def ik_position(self, target, q0=None, method='J_T', force=True, tol=1e-4, K=None, kd=0.001, max_iter=100):
        """
        (qf, ef, iter, reached_max_iter, status_msg) = arm.ik2(target, q0=None, method='jt', force=False, tol=1e-6, K=None)
        Description:
            Returns a solution to the inverse kinematics problem finding
            joint angles corresponding to the position (x y z coords) of target

        Args:
            target: 3x1 numpy array that defines the target location. 

            q0: length of initial joint coordinates, defaults to q=0 (which is
            often a singularity - other starting positions are recommended)

            method: String describing which IK algorithm to use. Options include:
                - 'pinv': damped pseudo-inverse solution, qdot = J_dag * e * dt, where
                J_dag = J.T * (J * J.T + kd**2)^-1
                - 'J_T': jacobian transpose method, qdot = J.T * K * e

            force: Boolean, if True will attempt to solve even if a naive reach check
            determines the target to be outside the reach of the arm

            tol: float, tolerance in the norm of the error in pose used as termination criteria for while loop

            K: 3x3 numpy matrix. For both pinv and J_T, K is the positive definite gain matrix used for both. 

            kd: is a scalar used in the pinv method to make sure the matrix is invertible. 

            max_iter: maximum attempts before giving up.

        Returns:
            qf: 6x1 numpy matrix of final joint values. If IK fails to converge the last set
            of joint angles is still returned

            ef: 3x1 numpy vector of the final error

            count: int, number of iterations

            flag: bool, "true" indicates successful IK solution and "false" unsuccessful

            status_msg: A string that may be useful to understanding why it failed. 

            qlist: a list of the q values that the algorithm took on the way to the solution
        """
        # Fill in q if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        # Try basic check for if the target is in the workspace.
        # Maximum length of the arm is sum(sqrt(d_i^2 + a_i^2)), distance to target is norm(A_t)
        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)
        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)
        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"
        else:
            if target_distance > maximum_reach:
                print("Target out of workspace, but finding closest solution anyway")
            else:
                print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(float(target_distance), float(maximum_reach)))

        # check for a valid gain matrix
        if not isinstance(K, np.ndarray):
            return q, error, count, False,  "No gain matrix 'K' provided"

        # check for a valid method
        if method not in ('J_T', 'pinv'):
            return q, error, count, False,  "invalid method: specify 'J_T' or 'pinv'"

        qList = []
        error = self.getError(target, q)
        while np.linalg.norm(error) > tol and count < max_iter:

            if method == 'pinv':
                step = self.getStep_pinv(q, K, error, kd)
            else:
                step = self.getStep_JT(q, K, error)

            q = q + step

            error = self.getError(target, q)

            count += 1
            qList.append(q)

        return (q, error, count, count < max_iter, 'No errors noted', qList)

    def getError(self, target, q):
        curPos = self.fk(q)[:3,3]
        return target - curPos
    
    def getStep_pinv(self,q, K, e, kd):
        J = self.jacob(q)[:3,:]
        JT = J.transpose()
        I = np.eye(3)

        step = JT @ np.linalg.inv((J @ JT) + (kd**2)*I) @ (K @ e)
        return step

    def getStep_JT(self,q, K, e):
        J = self.jacob(q)[:3,:]
        JT = J.transpose()
        
        step = JT @ K @ e
        return step



    #--------------------- functions specific to modified_ik_collision ---------------------------
    def add_obstacle_sphere(self, position, radius):
        self.obst_pos = position
        self.obst_rad = radius
    
    def check_if_colliding(self, qs) -> bool:
        """use a joint radius of .25 because that's about what it looks like in the diagram"""
        for i in range(len(qs)+1):
            # do forward kinematics at each joint
            T = self.fk(q=qs,index=i)
            t = T[:3,3]
            distToCenterOfObstacle = self.getDistToObstacle(t)
            if distToCenterOfObstacle <= self.obst_rad:
                return True
        
        return False
    
    def getDistToObstacle(self, t):
        """"
        t = a 3x1 matrix representing a point in frame 0
        calculates distance from t to self.obst_pos using pythagorean thm.
        """
        x1 = t[0]
        y1 = t[1]
        z1 = t[2]

        x2 = self.obst_pos[0]
        y2 = self.obst_pos[1]
        z2 = self.obst_pos[2]

        return np.sqrt( (x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    
    def collision_ik(self, q0, target):
        """
        inputs
            - q0: starting configuration
            - target: ending position 
        outputs
            - collisionOccured: did a collision occur in trying to get from q0 to target
            - q: final joint configuration
            - qList: all the joint configurations that the robot took to get to the final
            - foundTarget: did the robot succeed in reaching it's target (within a certain tolerance)
        """

        collisionOccured = False

        tol=.001
        max_iter=1000
        gain = 0.01
        K=gain*np.eye(3)

        # Fill in q if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        qList = []
        error = self.getError(target, q)
        while (np.linalg.norm(error) > tol) and (count < max_iter):


            step = self.getStep_JT(q, K, error)

            q = q + step
            collisionOccured = self.check_if_colliding(q)
            if collisionOccured:
                break

            error = self.getError(target, q)

            count += 1
            qList.append(q)
        
        foundTarget = (count < max_iter)

        return (collisionOccured, q, qList,foundTarget )

    # FIXME: based on the animation, it seems as though the algorithm is accepting paths that collide with the obstacle...
    def findPathAroundObstacle(self, qStart, target):
        """"
        inputs:
            - qStart:       starting joint configuration (list of all joint angles)
            - target:       target (list of x,y,z target points)
        outputs:
            - path:         the set of random configurations generated that led to finding the path.
            - fullPath:     the set of random configuration AND the intermediary configurations calculated using the collision_ik() method
        you must define the obstacle usign the 'add_obstacle_sphere()' method
    
        """
        qCur = qStart
        path = [qStart]
        fullPath = []
        targetReached = False
        while not targetReached:
            qNew = self.getRandq()
            TNew = self.fk(qNew)
            pNew = TNew[:3,3]
            newPointIsValid,q,qList,foundTarget = self.collision_ik(qCur,pNew)
            if newPointIsValid and foundTarget:
                path.append(q)
                fullPath.extend(qList)
                qCur = qNew
                targetReached,q,qList,foundTarget = self.collision_ik(qNew,target)
                if targetReached and foundTarget:
                    path.append(q)
                    fullPath.extend(qList)
        
        return path, fullPath

        
    def getRandq(self):
        q = []
        for i in range(self.n):
            angle = random.uniform(0,(2*pi))
            q.append(angle)
        return q





if __name__ == "__main__":
    from visualization import VizScene
    import time
    import numpy as np
    

    # Defining a table of DH parameters where each row corresponds to another joint.
    # The order of the DH parameters is [theta, d, a, alpha] - which is the order of operations. 
    # The symbolic joint variables "q" do not have to be explicitly defined here. 
    # This is a two link, planar robot arm with two revolute joints. 
    dh = [[0, 0, 0.3, 0],
          [0, 0, 0.3, 0]]

    # make robot arm (assuming all joints are revolute)
    arm = SerialArm(dh)

    # defining joint configuration
    q = [pi/4.0, pi/4.0]  # 45 degrees and 45 degrees

    # show an example of calculating the entire forward kinematics
    Tn_in_0 = arm.fk(q)
    with np.printoptions(precision=3,suppress=True):
        print("Tn_in_0:\n", Tn_in_0, "\n")

    # show an example of calculating the kinematics between frames 0 and 1
    T1_in_0 = arm.fk(q, index=[0,1])
    with np.printoptions(precision=3,suppress=True):
        print("T1_in 0:\n", T1_in_0, "\n")

    # showing how to use "print" with the arm object
    print(arm)

    # now visualizing the coordinate frames that we've calculated
    viz = VizScene()

    viz.add_frame(arm.base, label='base')
    viz.add_frame(Tn_in_0, label="Tn_in_0")
    viz.add_frame(T1_in_0, label="T1_in_0")

    time_to_run = 30
    refresh_rate = 60

    for i in range(refresh_rate * time_to_run):
        viz.update()
        time.sleep(1.0/refresh_rate)
    
    viz.close_viz()
    