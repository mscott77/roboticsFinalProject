"""
Transforms Module - Contains code for to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell, former TA. 
"""

import numpy as np
from numpy import sin, cos, sqrt, transpose
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """

    R = np.array(
        [[np.cos(th),-np.sin(th)],
         [np.sin(th),np.cos(th)]]
        )
    
    return R


## 3D Transformations

def translation_se3(x,y,z):
    """
    given translations in x,y,z 
    returns a 4x4 transformation matrix with only the translation portion filled out
    """
    t = np.array(
    [[1,0,0,x],
     [0,1,0,y],
     [0,0,1,z],
     [0,0,0,1]]
    )
    return t

def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """

    R = np.array(
        [[1,0,0],
         [0,np.cos(th),-np.sin(th)],
         [0,np.sin(th),np.cos(th)]]
        )

    return R

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """

    R = np.array(
        [[np.cos(th),0,np.sin(th)],
         [0,1,0],
         [-np.sin(th),0,np.cos(th)]]
         )

    return R
    

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    R = np.array(
        [[np.cos(th),-np.sin(th),0],
         [np.sin(th),np.cos(th),0],
         [0,0,1]]
         )

    return R

# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''

    return np.transpose(R)


def clean_rotation_matrix(R, eps=1e-12):
    '''
    This function is not required, but helps to make sure that all
    matrices we return are proper rotation matrices
    '''

    for i in range(R.shape[0]):
        for j in range(R.shape[1]):
            if np.abs(R[i, j]) < eps:
                R[i, j] = 0.
            elif np.abs(R[i, j] - 1) < eps:
                R[i, j] = 1.
    return R



def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    
    # create a 4x4 identity matrix to be manipulated
    T = np.eye(4)

    # paste the respective Rotation/Translation matrices in the transform matrix
    T[:3,:3] = R
    T[:3,3] = p


    return T

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    R = T[:3,:3]
    p = T[:3,3]
    R_inv = np.transpose(R)
    p_inv = -R_inv @ p
    T_inv = np.eye(4)
    T_inv[:3,:3] = R_inv
    T_inv[:3,3] = p_inv

    return T_inv

def DH_Transform(theta,d,a,alpha):
    """
        Returns the homogenous transform matrix T for the given DH parameters
        theta   (Z-rotate)
        d       (Z-translate)
        a       (X-translate)
        alpha   (X-rotate)
    """

    T = np.array(
                [[np.cos(theta),-np.sin(theta)*np.cos(alpha),np.sin(alpha)*np.sin(theta),a*np.cos(theta)],
                 [np.sin(theta),np.cos(alpha)*np.cos(theta),-np.sin(alpha)*np.cos(theta),a*np.sin(theta)],
                 [0,np.sin(alpha),np.cos(alpha),d],
                 [0,0,0,1]]
                 )

    return T


#---------------------------------------HW4-alternate Representations---------------------------------------------
def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """
    
    # follow formula in book, use functions like "np.atan2" 
    # for the arctangent and "**2" for squared terms. 
    # TODO - fill out this equation for rpy

    roll = np.atan2(R[1,0],R[0,0])
    pitch = np.asin(-R[2,0])
    yaw = np.atan2(R[2,1],R[2,2])

    return np.array([roll, pitch, yaw])


def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.acos," "np.sin," etc. 
    theta = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1) / 2)
    temp = np.array([(R[2,1]-R[1,2]), 
                     (R[0,2]-R[2,0]), 
                     (R[1,0]-R[0,1])])
    r = (1/(2*np.sin(theta)))*temp
    axis_angle = np.array([theta,0,0,0])
    axis_angle[1:] = r

    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """

    rx = v[0]
    ry = v[1]
    rz = v[2]
    theta = ang

    R00 = (rx**2)*(1-np.cos(theta)) + np.cos(theta)
    R01 = rx*ry*(1-np.cos(theta)) - rz*np.sin(theta)
    R02 = rx*rz*(1-np.cos(theta)) + ry*np.sin(theta)
    #
    R10 = rx*ry*(1-np.cos(theta)) + rz*np.sin(theta)
    R11 = (ry**2)*(1-np.cos(theta)) + np.cos(theta)
    R12 = ry*rz*(1-np.cos(theta)) - rx*np.sin(theta)
    #
    R20 = rx*rz*(1-np.cos(theta)) - ry*np.sin(theta)
    R21 = ry*rz*(1-np.cos(theta)) + rx*np.sin(theta)
    R22 = (rz**2)*(1-np.cos(theta)) + np.cos(theta)
    #
    R = np.array([[R00,R01,R02],
                 [R10,R11,R12],
                 [R20,R21,R22]])
    return R

def R2quat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """

    nu = 0.5 * np.sqrt(R[0,0] + R[1,1] + R[2,2] + 1)
    eps = 0.5 * np.array([np.sign(R[2,1] - R[1,2]) * np.sqrt(R[0,0] - R[1,1] - R[2,2] + 1),
                          np.sign(R[0,2] - R[2,0]) * np.sqrt(R[1,1] - R[2,2] - R[0,0] + 1),
                          np.sign(R[1,0] - R[0,1]) * np.sqrt(R[2,2] - R[0,0] - R[1,1] + 1)
                          ])

    result = np.array([nu,0,0,0])
    result[1:] = eps
    return result



def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """

    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]

    R =  np.array([[2*((nu**2) +(ex**2))-1,     2*(ex*ey - nu*ez),      2*(ex*ez + nu*ey)],
                   [2*(ex*ey + nu*ez),          2*((nu**2)+(ey**2))-1,  2*(ey*ez - nu*ex)],
                   [2*(ex*ez - nu*ey),          2*(ey*ez + nu*ex),      2*((nu**2)+(ez**2))-1]
                   ]) 
    
    return R


def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """

    # TODO - fill out each expression for R based on the condition 
    # (hint: use your rotx, roty, rotz functions)
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return R
