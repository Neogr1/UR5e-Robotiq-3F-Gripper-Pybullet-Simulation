'''
    [joints]
     0 palm_finger_A_joint
     1 finger_A_joint_1
     2 finger_A_joint_2
     3 finger_A_joint_3
     4 palm_finger_B_joint
     5 finger_B_joint_1
     6 finger_B_joint_2
     7 finger_B_joint_3
     8 palm_finger_C_joint
     9 finger_C_joint_1
    10 finger_C_joint_2
    11 finger_C_joint_3
'''


import time
import math
import pybullet as p
import pybullet_data

COEF_SCISSORS = 26 / 220
COEF_JOINT_1 = 62 / 140
COEF_JOINT_2 = 90 / 100

client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
# gripperId = p.loadURDF("robotiq-3f-gripper_articulated.urdf", basePosition=[0,0,0.05], baseOrientation=[1,0,0,1], useFixedBase=True)
mainpulatorId = p.loadURDF("ur10-gripper.urdf", basePosition=[0,0,0], baseOrientation=[0,0,0,1], useFixedBase=True)


# num_joints = p.getNumJoints(gripperId)

# rPOA_slider = p.addUserDebugParameter("rPOA", 0, 255, 0)
# rPOB_slider = p.addUserDebugParameter("rPOB", 0, 255, 0)
# rPOC_slider = p.addUserDebugParameter("rPOC", 0, 255, 0)
# rPOS_slider = p.addUserDebugParameter("rPOS", 0, 255, 137)
# rICF_slider = p.addUserDebugParameter("rICF", 0, 1, 0)


# boxColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02,0.15,0.02])
# boxVisId = p.createVisualShape(p.GEOM_BOX,  halfExtents=[0.02,0.15,0.02])
# boxBodyId = p.createMultiBody(baseMass=0,
#                                    baseCollisionShapeIndex=boxColId,
#                                    baseVisualShapeIndex=boxVisId,
#                                    basePosition=[0,0,0.22],
# )

while 1:
    # rPOA = p.readUserDebugParameter(rPOA_slider)
    # rPOB = p.readUserDebugParameter(rPOB_slider)
    # rPOC = p.readUserDebugParameter(rPOC_slider)
    # rPOS = p.readUserDebugParameter(rPOS_slider)
    # rICF = p.readUserDebugParameter(rICF_slider)

    # # position of scissor axis
    # fa0 = 0
    # fb0 = COEF_SCISSORS * min(rPOS, 220) - 16
    # fc0 = -fb0

    # # position of finger A
    # fa1 = COEF_JOINT_1 * min(rPOA, 140)
    # fa2 = COEF_JOINT_2 * min(max(rPOA-140, 0), 100)
    # fa3 = - COEF_JOINT_1 * min(rPOA, 110)

    # # individual control of fingers
    # if rICF < 0.5:
    #     fb1, fb2, fb3 = fa1, fa2, fa3
    #     fc1, fc2, fc3 = fa1, fa2, fa3
    # else:
    #     fb1 = COEF_JOINT_1 * min(rPOB, 140)
    #     fb2 = COEF_JOINT_2 * min(max(rPOB-140, 0), 100)
    #     fb3 = - COEF_JOINT_1 * min(rPOB, 110)
    #     fc1 = COEF_JOINT_1 * min(rPOC, 140)
    #     fc2 = COEF_JOINT_2 * min(max(rPOC-140, 0), 100)
    #     fc3 = - COEF_JOINT_1 * min(rPOC, 110)
    
    # p.setJointMotorControl2(gripperId,  0, p.POSITION_CONTROL, targetPosition=math.radians(fa0))
    # p.setJointMotorControl2(gripperId,  1, p.POSITION_CONTROL, targetPosition=math.radians(fa1))
    # p.setJointMotorControl2(gripperId,  2, p.POSITION_CONTROL, targetPosition=math.radians(fa2))
    # p.setJointMotorControl2(gripperId,  3, p.POSITION_CONTROL, targetPosition=math.radians(fa3))
    # p.setJointMotorControl2(gripperId,  4, p.POSITION_CONTROL, targetPosition=math.radians(fb0))
    # p.setJointMotorControl2(gripperId,  5, p.POSITION_CONTROL, targetPosition=math.radians(fb1))
    # p.setJointMotorControl2(gripperId,  6, p.POSITION_CONTROL, targetPosition=math.radians(fb2))
    # p.setJointMotorControl2(gripperId,  7, p.POSITION_CONTROL, targetPosition=math.radians(fb3))
    # p.setJointMotorControl2(gripperId,  8, p.POSITION_CONTROL, targetPosition=math.radians(fc0))
    # p.setJointMotorControl2(gripperId,  9, p.POSITION_CONTROL, targetPosition=math.radians(fc1))
    # p.setJointMotorControl2(gripperId, 10, p.POSITION_CONTROL, targetPosition=math.radians(fc2))
    # p.setJointMotorControl2(gripperId, 11, p.POSITION_CONTROL, targetPosition=math.radians(fc3))

    p.stepSimulation()
    time.sleep(0.1)