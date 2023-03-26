import math
import numpy as np
from transforms3d.euler import euler2mat


##DEBUG #######################################################################

# TRACE macro
DEBUG = True
DEBUG = False

def log(s):
    if DEBUG:
        print(s)

##DEBUG #######################################################################

# Constant Rotation from BRF to LRF reference frame
R_BRF_to_LRF = np.array([
        [0,0,1],
        [0,1,0],
        [-1,0,0]
    ])


def leg_forward_kinematics_LRF(joint_position, leg_index, config):
    """Find the foot position in the Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3)
        Array of corresponding foot positions in Leg Reference Frame
        [X,Y,Z] in LRF
    """
    c1 = math.cos(joint_position[0])
    s1 = math.sin(joint_position[0])
    c2 = -math.sin(joint_position[1])
    s2 = math.cos(joint_position[1])
    c3 = -math.sin(joint_position[2])
    s3 = math.cos(joint_position[2])
    x = c1*c3*config.LEG_LT + c1*c2*config.LEG_LF - s1*config.ABDUCTION_OFFSETS[leg_index]
    y = s1*c3*config.LEG_LT + s1*c2*config.LEG_LF + c1*config.ABDUCTION_OFFSETS[leg_index]
    z =   -s3*config.LEG_LT -    s2*config.LEG_LF
    return np.array([x,y,z])


def four_legs_forward_kinematics_LRF(joint_position, config):
    """Find the position of the four feet in their respective Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3x4) 
        Array of the joint angles of four legs
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of corresponding foot positions in Leg Reference Frame
        [X,Y,Z] in LRF
    """    
    feet_position_LRF = np.zeros((3,4))
    # for each leg
    for i in range(4):
        feet_position_LRF[:,i] = leg_forward_kinematics_LRF(joint_position[:,i],i,config)
    return feet_position_LRF


def four_legs_forward_kinematics_BRF(joint_position, config):
    """Find the position of the four feet in the Body Reference Frame

    BRF : centered body, X forward, Y leftward, Z upward for all leg

    Parameters
    ----------
    joint_position : numpy array (3x4) 
        Array of the joint angles of four legs
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of corresponding foot positions in Body Reference Frame
        [X,Y,Z] in LRF
    """    
    feet_position_BRF = np.zeros((3,4))
    # for each leg
    for i in range(4):
        feet_position_LRF = leg_forward_kinematics_LRF(joint_position[:,i],i,config)
        feet_position_BRF[:,i] = R_BRF_to_LRF.transpose().dot(feet_position_LRF)+config.LEG_ORIGINS[:,i]
    return feet_position_BRF

def jacobian(joint_position,leg_index,config):
    """Compute Jacobian matrixin Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x3)
        J so that Foot XYZ Velocity in LRF = J x Joint rotation speed
    """      
    c1 = math.cos(joint_position[0])
    s1 = math.sin(joint_position[0])
    c2 = -math.sin(joint_position[1])
    s2 = math.cos(joint_position[1])
    c3 = -math.sin(joint_position[2])
    s3 = math.cos(joint_position[2])    
    return np.array(
        [
            [
                -s1*c3*config.LEG_LT - s1*c2*config.LEG_LF - c1*config.ABDUCTION_OFFSETS[leg_index],
                -c1*s3*config.LEG_LT - c1*s2*config.LEG_LF,
                -c1*s3*config.LEG_LT
            ],
            [
                 c1*c3*config.LEG_LT + c1*c2*config.LEG_LF - s1*config.ABDUCTION_OFFSETS[leg_index],
                -s1*s3*config.LEG_LT - s1*s2*config.LEG_LF,
                -s1*s3*config.LEG_LT
            ],
            [
                0,
                -c3*config.LEG_LT - c2*config.LEG_LF,
                -c3*config.LEG_LT
            ]
        ]
    )


def leg_inverse_kinematics_LRF(target_foot_position_LRF,initial_joint_position,leg_index,config,alpha=1.0):
    """Find the joint position from a foot position in the Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_foot_position_LRF : numpy array (3)
        Array of corresponding foot position in Leg Reference Frame
        [X,Y,Z] in LRF
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = initial_joint_position

    for counter in range(20):

        J = jacobian(joint_position,leg_index,config)
        #Jt = J.transpose()
        Jt = np.linalg.pinv(J)

        foot_position_LRF = leg_forward_kinematics_LRF(joint_position, leg_index, config)
        error = target_foot_position_LRF-foot_position_LRF
        step = alpha * (Jt @ error)
        joint_position += np.clip(step, -math.pi/4, math.pi/4)
        #joint_position = np.clip(joint_position,[-math.pi/2,0.0,0.0],[math.pi/2,1.2*math.pi,math.pi])
        if  np.linalg.norm(error) < 0.0001:
            break

    joint_position[1:2] = np.fmod(joint_position[1:2] + 2*np.pi, 2 * np.pi) 
    return joint_position 


def four_legs_inverse_kinematics_LRF(target_foot_position_LRF,initial_joint_position,config,alpha=1.0):
    """Find the position of the four feet in their respective Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_foot_position_LRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS        
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        joint_position[:,i] = leg_inverse_kinematics_LRF(
            target_foot_position_LRF[:,i],
            initial_joint_position[:,i],
            i,
            config,
            alpha
        )
    return joint_position


def four_legs_inverse_kinematics_BRF(target_feet_position_BRF,initial_joint_position,config,alpha=1.0):
    """Find the joint position of the four feet from their position in the Body Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_feet_position_BRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS        
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        target_foot_position_LRF = R_BRF_to_LRF.dot(target_feet_position_BRF[:,i]-config.LEG_ORIGINS[:,i])

        joint_position[:,i] = leg_inverse_kinematics_LRF(
            target_foot_position_LRF,
            initial_joint_position[:,i],
            i,
            config
        )
    return joint_position





