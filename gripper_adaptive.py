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
POS_MARGIN = 0.01
MAX_TORQUE = 10

client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
gripperId = p.loadURDF("robotiq-3f-gripper_articulated.urdf", basePosition=[0,0,0.05], baseOrientation=[1,0,0,1], useFixedBase=True)

rPOA_slider = p.addUserDebugParameter("rPOA", 0, 255, 0)
rPOB_slider = p.addUserDebugParameter("rPOB", 0, 255, 0)
rPOC_slider = p.addUserDebugParameter("rPOC", 0, 255, 0)
rPOS_slider = p.addUserDebugParameter("rPOS", 0, 255, 137)
rICF_slider = p.addUserDebugParameter("rICF", 0, 1, 0)


# cylinderColId = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.15)
# cylinderVisId = p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=0.15)
# cylinderBodyId = p.createMultiBody(baseMass=0,
#                                    baseCollisionShapeIndex=cylinderColId,
#                                    baseVisualShapeIndex=cylinderVisId,
#                                    basePosition=[0,0,0.15],
#                                    baseOrientation=[1,0,0,1]
#                                 )
# boxColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02,0.15,0.02])
# boxVisId = p.createVisualShape(p.GEOM_BOX,  halfExtents=[0.02,0.15,0.02])
# boxBodyId = p.createMultiBody(baseMass=0,
#                                    baseCollisionShapeIndex=boxColId,
#                                    baseVisualShapeIndex=boxVisId,
#                                    basePosition=[0,0,0.22],
#                                 )

joint_index_list = [0,1,2,3,4,5,6,7,8,9,10,11]

req_fa0 = 0
req_fa1 = 0
req_fa2 = 0
req_fa3 = 0
req_fb0 = 0
req_fb1 = 0
req_fb2 = 0
req_fb3 = 0
req_fc0 = 0
req_fc1 = 0
req_fc2 = 0
req_fc3 = 0

while 1:
    rPOA = p.readUserDebugParameter(rPOA_slider)
    rPOB = p.readUserDebugParameter(rPOB_slider)
    rPOC = p.readUserDebugParameter(rPOC_slider)
    rPOS = p.readUserDebugParameter(rPOS_slider)
    rICF = p.readUserDebugParameter(rICF_slider)

    # position of scissor axis
    req_fa0 = 0
    req_fb0 = COEF_SCISSORS * min(rPOS, 220) - 16
    req_fc0 = -req_fb0

    # position of finger A
    req_fa1 = COEF_JOINT_1 * min(rPOA, 140)
    req_fa2 = COEF_JOINT_2 * min(max(rPOA-140, 0), 100)
    req_fa3 = - COEF_JOINT_1 * min(rPOA, 110)

    # individual control of fingers
    if rICF < 0.5:
        req_fb1, req_fb2, req_fb3 = req_fa1, req_fa2, req_fa3
        req_fc1, req_fc2, req_fc3 = req_fa1, req_fa2, req_fa3
    else:
        req_fb1 = COEF_JOINT_1 * min(rPOB, 140)
        req_fb2 = COEF_JOINT_2 * min(max(rPOB-140, 0), 100)
        req_fb3 = - COEF_JOINT_1 * min(rPOB, 110)
        req_fc1 = COEF_JOINT_1 * min(rPOC, 140)
        req_fc2 = COEF_JOINT_2 * min(max(rPOC-140, 0), 100)
        req_fc3 = - COEF_JOINT_1 * min(rPOC, 110)




    joint_states = p.getJointStates(gripperId, joint_index_list)

    # scissors
    if abs(joint_states[4][0] - req_fb0) <= POS_MARGIN and abs(joint_states[8][0] - req_fc0) <= POS_MARGIN:
        fb0 = req_fb0
        fc0 = req_fc0
    elif joint_states[4][3] >= MAX_TORQUE or joint_states[8][3] >= MAX_TORQUE:
        fb0 = joint_states[4][0]
        fc0 = joint_states[8][0]
    else:
        if joint_states[4][0] < req_fb0: fb0 = min(fb0+1, req_fb0)
        else: fb0 = max(fb0-1, req_fb0)
        if joint_states[8][0] < req_fc0: fc0 = min(fc0+1, req_fc0)
        else: fc0 = max(fc0-1, req_fc0)

    '''
    [ finger A ]
    '''
    # third link on stuck
    if joint_states[3][3] >= MAX_TORQUE:
        fa1 = joint_states[1][0]
        fa2 = joint_states[2][0]
        fa3 = joint_states[3][0]
    # second link on stuck
    elif joint_states[2][3] >= MAX_TORQUE:
        fa1 = joint_states[1][0]
        fa2 = joint_states[2][0]
        fa3 = min(- COEF_JOINT_1 * min(rPOA, 110)) # ???????????????????????????
    # first link on stuck
    else:
        pass
    '''
    g_ob가 어디인지 아는 것이 중요. 그래야 각도 계산 가능함
    -> stuck위치로부터 역연산 할 수밖에 없는듯
    ex)
        joint2==0이면 g_ob<110이므로 g_ob = toDegree(joint1) / coef_joint_1
        joint2>0이면 g_ob = toDegree(joint2) / coef_joint_2 + 110
    '''


    # # first link on req
    # if abs(joint_states[1][0] - req_fa1) <= POS_MARGIN:
    #     fa1 = req_fa1
    #     ## second link on req
    #     if abs(joint_states[2][0] - req_fa2) <= POS_MARGIN:
    #         fa2 = req_fa2
    #         ### third link on req
    #         if abs(joint_states[3][0] - req_fa3) <= POS_MARGIN:
    #             fa3 = req_fa3
    #         ### third link on stuck
    #         elif joint_states[3][3] >= MAX_TORQUE:
    #             fa3 = joint_states[3][0]
    #         ### move third link
    #         else:
    #             if joint_states[3][0] < req_fa3: fa3 = min(fa3+1, req_fa3)
    #             else: fa3 = max(fa3-1, req_fa3)
    #     ## second link on stuck
    #     elif joint_states[2][3] >= MAX_TORQUE:
    #         fa2 = joint_states[2][0]
    #         ### third link on req
    #     ## third link on stuck
    #     elif joint_states[3][3] >= MAX_TORQUE:
    #         fa2 = joint_states[2][0]
    #         pass
    #     ## move second link
    #     else:
    #         if joint_states[2][0] < req_fa2: fa2 = min(fa2+1, req_fa2)
    #         else: fa2 = max(fa2-1, req_fa2)
    # # first link on stuck
    # elif joint_states[1][3] >= MAX_TORQUE:
    #     fa1 = joint_states[1][0]
    # # second link on stuck
    # elif joint_states[2][3] >= MAX_TORQUE:
    #     fa1 = joint_states[1][0]
    # # third link on stuck
    # elif joint_states[3][3] >= MAX_TORQUE:
    #     fa1 = joint_states[1][0]
    # # move first link
    # else:
    #     if joint_states[1][0] < req_fa1: fa1 = min(fa1+1, req_fa1)
    #     else: fa1 = max(fa1-1, req_fa1)

    
    p.setJointMotorControl2(gripperId,  0, p.POSITION_CONTROL, targetPosition=math.radians(fa0))
    p.setJointMotorControl2(gripperId,  1, p.POSITION_CONTROL, targetPosition=math.radians(fa1))
    p.setJointMotorControl2(gripperId,  2, p.POSITION_CONTROL, targetPosition=math.radians(fa2))
    p.setJointMotorControl2(gripperId,  3, p.POSITION_CONTROL, targetPosition=math.radians(fa3))
    p.setJointMotorControl2(gripperId,  4, p.POSITION_CONTROL, targetPosition=math.radians(fb0))
    p.setJointMotorControl2(gripperId,  5, p.POSITION_CONTROL, targetPosition=math.radians(fb1))
    p.setJointMotorControl2(gripperId,  6, p.POSITION_CONTROL, targetPosition=math.radians(fb2))
    p.setJointMotorControl2(gripperId,  7, p.POSITION_CONTROL, targetPosition=math.radians(fb3))
    p.setJointMotorControl2(gripperId,  8, p.POSITION_CONTROL, targetPosition=math.radians(fc0))
    p.setJointMotorControl2(gripperId,  9, p.POSITION_CONTROL, targetPosition=math.radians(fc1))
    p.setJointMotorControl2(gripperId, 10, p.POSITION_CONTROL, targetPosition=math.radians(fc2))
    p.setJointMotorControl2(gripperId, 11, p.POSITION_CONTROL, targetPosition=math.radians(fc3))

    p.stepSimulation()

    print(p.getJointState(gripperId, 1)[0]-math.radians(fa1))   

    # info = p.getJointInfo(gripperId, 1)
    # msg = str(info[1]) + '\n'
    # msg += "lower: " + str(info[8]) + '\n'
    # msg += "upper: " + str(info[9]) + '\n'
    # msg += "maxForce: " + str(info[10]) + '\n'
    # msg += "maxVelocity: " + str(info[11]) + '\n'
    # print(msg)

    time.sleep(0.1)