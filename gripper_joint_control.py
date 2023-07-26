import time
import math
import pybullet as p
import pybullet_data

client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
gripperId = p.loadURDF("robotiq-3f-gripper_articulated.urdf", basePosition=[0,0,0.05], baseOrientation=[1,0,0,1], useFixedBase=True)

# joint_index = -1  # Replace with the actual index of the joint you want to adjust

num_joints = p.getNumJoints(gripperId)

'''
[joints]
 0 palm_finger_1_joint
 1 finger_1_joint_1
 2 finger_1_joint_2
 3 finger_1_joint_3
 4 palm_finger_2_joint
 5 finger_2_joint_1
 6 finger_2_joint_2
 7 finger_2_joint_3
 8 palm_finger_middle_joint
 9 finger_middle_joint_1
10 finger_middle_joint_2
11 finger_middle_joint_3
'''

scissor_slider = p.addUserDebugParameter("Finger C - Scissor", -16, 10, 0)
f1_1_slider = p.addUserDebugParameter("Finger C - Joint 1", -35, 35, -27)
f1_2_slider = p.addUserDebugParameter("Finger C - Joint 2", 0, 90, 0)
f1_3_slider = p.addUserDebugParameter("Finger C - Joint 3", -25, 73, 27)
f2_0_slider = p.addUserDebugParameter("Finger B - Scissor", -10, 16, 0)
f2_1_slider = p.addUserDebugParameter("Finger B - Joint 1", -35, 35, -27)
f2_2_slider = p.addUserDebugParameter("Finger B - Joint 2", 0, 90, 0)
f2_3_slider = p.addUserDebugParameter("Finger B - Joint 3", -25, 73, 27)
fm_1_slider = p.addUserDebugParameter("Finger A - Joint 1", -35, 35, -27)
fm_2_slider = p.addUserDebugParameter("Finger A - Joint 2", 0, 90, 0)
fm_3_slider = p.addUserDebugParameter("Finger A - Joint 3", -25, 73, 27)

while 1:
    f1_0 = p.readUserDebugParameter(scissor_slider)
    f1_1 = p.readUserDebugParameter(f1_1_slider) + 35
    f1_2 = p.readUserDebugParameter(f1_2_slider)
    f1_3 = p.readUserDebugParameter(f1_3_slider) - 27
    f2_0 = -f1_0
    f2_1 = p.readUserDebugParameter(f2_1_slider) + 35
    f2_2 = p.readUserDebugParameter(f2_2_slider)
    f2_3 = p.readUserDebugParameter(f2_3_slider) - 27
    fm_0 = 0
    fm_1 = p.readUserDebugParameter(fm_1_slider) + 35
    fm_2 = p.readUserDebugParameter(fm_2_slider)
    fm_3 = p.readUserDebugParameter(fm_3_slider) - 27

    p.setJointMotorControl2(gripperId,  0, p.POSITION_CONTROL, targetPosition=math.radians(f1_0))
    p.setJointMotorControl2(gripperId,  1, p.POSITION_CONTROL, targetPosition=math.radians(f1_1))
    p.setJointMotorControl2(gripperId,  2, p.POSITION_CONTROL, targetPosition=math.radians(f1_2))
    p.setJointMotorControl2(gripperId,  3, p.POSITION_CONTROL, targetPosition=math.radians(f1_3))
    p.setJointMotorControl2(gripperId,  4, p.POSITION_CONTROL, targetPosition=math.radians(f2_0))
    p.setJointMotorControl2(gripperId,  5, p.POSITION_CONTROL, targetPosition=math.radians(f2_1))
    p.setJointMotorControl2(gripperId,  6, p.POSITION_CONTROL, targetPosition=math.radians(f2_2))
    p.setJointMotorControl2(gripperId,  7, p.POSITION_CONTROL, targetPosition=math.radians(f2_3))
    p.setJointMotorControl2(gripperId,  8, p.POSITION_CONTROL, targetPosition=math.radians(fm_0))
    p.setJointMotorControl2(gripperId,  9, p.POSITION_CONTROL, targetPosition=math.radians(fm_1))
    p.setJointMotorControl2(gripperId, 10, p.POSITION_CONTROL, targetPosition=math.radians(fm_2))
    p.setJointMotorControl2(gripperId, 11, p.POSITION_CONTROL, targetPosition=math.radians(fm_3))

    p.stepSimulation()
    time.sleep(0.1)