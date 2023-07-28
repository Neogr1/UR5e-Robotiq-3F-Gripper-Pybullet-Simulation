import os
import time
import math
import pybullet as p
import pybullet_data

# constatns for control mode
PALM_FREE = 0
PALM_DOWN = 1

# constants for calculating IK
LOWER = [ -math.pi] * 6
UPPER = [  math.pi] * 6
JOINT_RANGES = [2*math.pi] * 6
REST = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
END_LINK_AS_WRIST = 7
END_LINK_AS_PALM = 10 

# constants for calculating gripper joint angles
COEF_SCISSORS = 26 / 220
COEF_JOINT_1 = 62 / 140
COEF_JOINT_2 = 90 / 100

# constants for set joint angles
UR5_INDICES = (1,2,3,4,5,6)
GRIPPER_INDICES = (9,10,11,12,13,14,15,16,17,18,19,20)


def load_urdfs():
    global tableId, robotId, objectId
    urdfRootPath=pybullet_data.getDataPath()
    tableId = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.63], baseOrientation=[0,0,0,1])
    robotId = p.loadURDF("ur5e-gripper.urdf", basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    objectId = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.5,0,0.1])

    # boxColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02,0.15,0.02])
    # boxVisId = p.createVisualShape(p.GEOM_BOX,  halfExtents=[0.02,0.15,0.02])
    # boxBodyId = p.createMultiBody(baseMass=0.1,
    #                                 baseCollisionShapeIndex=boxColId,
    #                                 baseVisualShapeIndex=boxVisId,
    #                                 basePosition=[0.5,0,0.1],
    # )

def add_sliders():
    global x_slider,y_slider,z_slider,rx_slider,ry_slider,rz_slider,rPOA_slider,rPOB_slider,rPOC_slider,rPOS_slider,rICF_slider
    x_slider = p.addUserDebugParameter("X", 0, 1, 0.5)
    y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
    z_slider = p.addUserDebugParameter("Z", 0.2, 1, 0.3) # end link wrist
    # z_slider = p.addUserDebugParameter("Z", 0.12, 1, 0.3) # end link palm
    if mode == PALM_FREE:
        rx_slider = p.addUserDebugParameter("Rx", -math.pi/2, math.pi/2, 0)
        ry_slider = p.addUserDebugParameter("Ry", -math.pi/2, math.pi/2, 0)
    rz_slider = p.addUserDebugParameter("Rz", -math.pi/2, math.pi/2, 0)

    rPOA_slider = p.addUserDebugParameter("rPOA", 0, 255, 0)
    rPOB_slider = p.addUserDebugParameter("rPOB", 0, 255, 0)
    rPOC_slider = p.addUserDebugParameter("rPOC", 0, 255, 0)
    rPOS_slider = p.addUserDebugParameter("rPOS", 0, 255, 137)
    rICF_slider = p.addUserDebugParameter("rICF", 0, 1, 0)


def read_sliders():
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    if mode == PALM_FREE:
        rx = p.readUserDebugParameter(rx_slider)
        ry = p.readUserDebugParameter(ry_slider)
    elif mode == PALM_DOWN:
        rx = None
        ry = None
    rz = p.readUserDebugParameter(rz_slider)

    rPOA = p.readUserDebugParameter(rPOA_slider)
    rPOB = p.readUserDebugParameter(rPOB_slider)
    rPOC = p.readUserDebugParameter(rPOC_slider)
    rPOS = p.readUserDebugParameter(rPOS_slider)
    rICF = p.readUserDebugParameter(rICF_slider)

    return x,y,z,rx,ry,rz,rPOA,rPOB,rPOC,rPOS,rICF

def get_ur5_joint_angles(x,y,z,rx,ry,rz):
    position = [x,y,z]
    if   mode == PALM_FREE: orientation = [rx,ry,rz]
    elif mode == PALM_DOWN: orientation = [0,math.pi/2,rz]

    quaternion = p.getQuaternionFromEuler(orientation)

    joint_angles = p.calculateInverseKinematics(
        robotId, endEffectorLinkIndex=END_LINK_AS_WRIST,
        # robotId, endEffectorLinkIndex=END_LINK_AS_PALM,
        targetPosition=position, targetOrientation=quaternion, 
        upperLimits=UPPER, lowerLimits=LOWER,
        jointRanges=JOINT_RANGES, restPoses=REST
    )

    return joint_angles

def get_gripper_joint_angles(rPOA,rPOB,rPOC,rPOS,rICF):
    # position of scissor axis
    fa0 = 0
    fb0 = COEF_SCISSORS * min(rPOS, 220) - 16
    fc0 = -fb0

    # position of finger A
    fa1 = COEF_JOINT_1 * min(rPOA, 140)
    fa2 = COEF_JOINT_2 * min(max(rPOA-140, 0), 100)
    fa3 = - COEF_JOINT_1 * min(rPOA, 110)

    # individual control of fingers
    if rICF < 0.5:
        fb1, fb2, fb3 = fa1, fa2, fa3
        fc1, fc2, fc3 = fa1, fa2, fa3
    else:
        fb1 = COEF_JOINT_1 * min(rPOB, 140)
        fb2 = COEF_JOINT_2 * min(max(rPOB-140, 0), 100)
        fb3 = - COEF_JOINT_1 * min(rPOB, 110)
        fc1 = COEF_JOINT_1 * min(rPOC, 140)
        fc2 = COEF_JOINT_2 * min(max(rPOC-140, 0), 100)
        fc3 = - COEF_JOINT_1 * min(rPOC, 110)

    gripper_joint_angles = list(map(lambda x: math.radians(x), [fa0,fa1,fa2,fa3,fb0,fb1,fb2,fb3,fc0,fc1,fc2,fc3]))
    return gripper_joint_angles





mode = PALM_FREE
# mode = PALM_DOWN

p.connect(p.GUI)
p.setRealTimeSimulation(True)
p.setGravity(0, 0, -9.81)

load_urdfs()
add_sliders()

while 1:
    x,y,z,rx,ry,rz,rPOA,rPOB,rPOC,rPOS,rICF = read_sliders()

    ur5_joint_angles = get_ur5_joint_angles(x,y,z,rx,ry,rz)[:6]
    gripper_joint_angles = get_gripper_joint_angles(rPOA,rPOB,rPOC,rPOS,rICF)
 
    p.setJointMotorControlArray(robotId, UR5_INDICES, p.POSITION_CONTROL, targetPositions=ur5_joint_angles)
    p.setJointMotorControlArray(robotId, GRIPPER_INDICES, p.POSITION_CONTROL, targetPositions=gripper_joint_angles)

    p.stepSimulation()
    time.sleep(0.1)