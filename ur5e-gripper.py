import os
import time
import math
import pybullet as p
import pybullet_data

# https://github.com/josepdaniel/ur5-bullet/UR5/UR5Sim.py
LOWER = [ -math.pi] * 6
UPPER = [  math.pi] * 6
JOINT_RANGES = [2*math.pi] * 6
REST = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
END_LINK_AS_WRIST = 7
END_LINK_AS_PALM = 10
def calculate_ik(position, orientation):
    quaternion = p.getQuaternionFromEuler(orientation)

    joint_angles = p.calculateInverseKinematics(
        robotId, endEffectorLinkIndex=END_LINK_AS_PALM, # set the end link as what you want
        targetPosition=position, targetOrientation=quaternion, 
        upperLimits=UPPER, lowerLimits=LOWER,
        jointRanges=JOINT_RANGES, restPoses=REST
    )
    return joint_angles


COEF_SCISSORS = 26 / 220
COEF_JOINT_1 = 62 / 140
COEF_JOINT_2 = 90 / 100

p.connect(p.GUI)
p.setRealTimeSimulation(True)
urdfRootPath=pybullet_data.getDataPath()
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.63], baseOrientation=[0,0,0,1])
robotId = p.loadURDF("ur5e-gripper.urdf", basePosition=[0,0,0], baseOrientation=[0,0,0,1])
# objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[1,0,0.1])

x_slider = p.addUserDebugParameter("X", 0, 1, 0.4)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1, 0.4)
rx_slider = p.addUserDebugParameter("Rx", -math.pi/2, math.pi/2, 0)
ry_slider = p.addUserDebugParameter("Ry", -math.pi/2, math.pi/2, 0)
rz_slider = p.addUserDebugParameter("Rz", -math.pi/2, math.pi/2, 0)

rPOA_slider = p.addUserDebugParameter("rPOA", 0, 255, 0)
rPOB_slider = p.addUserDebugParameter("rPOB", 0, 255, 0)
rPOC_slider = p.addUserDebugParameter("rPOC", 0, 255, 0)
rPOS_slider = p.addUserDebugParameter("rPOS", 0, 255, 137)
rICF_slider = p.addUserDebugParameter("rICF", 0, 1, 0)



ur5_joint_indices = [1,2,3,4,5,6]
ur5_joint_forces = [150,150,150,28,28,28]
while 1:
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    rx = p.readUserDebugParameter(rx_slider)
    ry = p.readUserDebugParameter(ry_slider)
    rz = p.readUserDebugParameter(rz_slider)
    t1,t2,t3,t4,t5,t6,*_ = calculate_ik([x,y,z], [rx,ry,rz])
    # t1,t2,t3,t4,t5,t6,*_ = calculate_ik([x,y,z], [0,math.pi/2,0])
    # print(t1,t2,t3,t4,t5,t6)

    rPOA = p.readUserDebugParameter(rPOA_slider)
    rPOB = p.readUserDebugParameter(rPOB_slider)
    rPOC = p.readUserDebugParameter(rPOC_slider)
    rPOS = p.readUserDebugParameter(rPOS_slider)
    rICF = p.readUserDebugParameter(rICF_slider)

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


    p.setJointMotorControl2(robotId,  1, p.POSITION_CONTROL, targetPosition=t1)
    p.setJointMotorControl2(robotId,  2, p.POSITION_CONTROL, targetPosition=t2)
    p.setJointMotorControl2(robotId,  3, p.POSITION_CONTROL, targetPosition=t3)
    p.setJointMotorControl2(robotId,  4, p.POSITION_CONTROL, targetPosition=t4)
    p.setJointMotorControl2(robotId,  5, p.POSITION_CONTROL, targetPosition=t5)
    p.setJointMotorControl2(robotId,  6, p.POSITION_CONTROL, targetPosition=t6)

    p.setJointMotorControl2(robotId,  9, p.POSITION_CONTROL, targetPosition=math.radians(fa0), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 10, p.POSITION_CONTROL, targetPosition=math.radians(fa1), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 11, p.POSITION_CONTROL, targetPosition=math.radians(fa2), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 12, p.POSITION_CONTROL, targetPosition=math.radians(fa3), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 13, p.POSITION_CONTROL, targetPosition=math.radians(fb0), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 14, p.POSITION_CONTROL, targetPosition=math.radians(fb1), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 15, p.POSITION_CONTROL, targetPosition=math.radians(fb2), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 16, p.POSITION_CONTROL, targetPosition=math.radians(fb3), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 17, p.POSITION_CONTROL, targetPosition=math.radians(fc0), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 18, p.POSITION_CONTROL, targetPosition=math.radians(fc1), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 19, p.POSITION_CONTROL, targetPosition=math.radians(fc2), positionGain=0.04, targetVelocity=0)
    p.setJointMotorControl2(robotId, 20, p.POSITION_CONTROL, targetPosition=math.radians(fc3), positionGain=0.04, targetVelocity=0)

    p.stepSimulation()
    time.sleep(0.1)