import sys
import numpy as np
import math
import time
import random




try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", sim.simx_opmode_blocking)


# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, sim.simx_opmode_oneshot)

# You do not need to modify the code above

#-----------------------------------------------------------------------------------------------------------------------

# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def movearm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))
    sim.simxPauseCommunication(clientID,True)
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot) 
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)    
    sim.simxPauseCommunication(clientID,False)
    time.sleep(3)


thetas = np.array([50, 60, 80, 60, -90, 0])
thetasclose = np.array([-40, 52, 51, 45, 35, 47])
thetafar = np.array([-11, -37, -50, -45, 54, 52])

movearm(thetas)
time.sleep(3)
movearm(thetasclose)
time.sleep(3)
movearm(thetafar)
time.sleep(3)

# randomly creating 100 set of 6 theta angles
def randomPoses():
    global thetas
    for i in range(0, 100):
        # thetas = np.append(thetas, np.random.randint(-60, high=60, size=6))
        thetas = np.append(thetas, [[random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     ]], axis=0)
    return thetas

# testing = randomPoses()



# control the movement of the arm to [30, 30, 30, 30, 30, 30]
# target_arm = [30, 30, 30, 30, 30, 30]
# movearm(testing)
# for i in range(1):
#     movearm(testing[i])
    # return_code, state_self = sim.simxGetIntegerSignal(clientID, "Table2", sim.simx_opmode_blocking)
    # return_code, state_ground = sim.simxGetIntegerSignal(clientID, "Table2", sim.simx_opmode_blocking)
    # return_code, state_box = sim.simxGetIntegerSignal(clientID, "Table2", sim.simx_opmode_blocking)
    # time.sleep(3)
    #
    # if state_self:
    #     print('########################')
    #     print('##Collide with itself##')
    #     print('########################')
    #
    # if state_ground:
    #     print('########################')
    #     print('##Collide with ground##')
    #     print('########################')
    #
    # if state_box:
    #     print('########################')
    #     print('##Collide with box##')
    #     print('########################')
    #
    # if state_self == 0 and state_ground == 0 and state_box == 0:
    #     print('########################')
    #     print('##No Collision!!##')
    #     print('########################')

    # ------------------------------------------------------------------------------------------------------------------------------
    # err, contact_self = sim.simxGetCollisionHandle(clientID, 'co_self', sim.simx_opmode_blocking)
    # err, state_self = sim.simxReadCollision(clientID, contact_self, sim.simx_opmode_streaming)
    #
    # err, contact_objects = sim.simxGetCollisionHandle(clientID, "Table2", sim.simx_opmode_blocking)
    # err, state_objects = sim.simxReadCollision(clientID, contact_objects, sim.simx_opmode_streaming)
    #
    #
    #
    # print("state_self", state_self)
    # print("state_objects", state_objects)
    # # state_self = False
    #
    # if state_self is True or state_objects is True:
    #
    #     print("Collided")
    # else:
    #     print("NO Collisions")

    #------------------------------------------------------------------------------------------------------------------------------

    return_code, state_self=sim.simxGetIntegerSignal(clientID, "co_self", sim.simx_opmode_blocking)
    err, contact_objects =sim.simxGetCollisionHandle(clientID, "Table2", sim.simx_opmode_blocking)
    err, state_objects = sim.simxReadCollision(clientID, contact_objects, sim.simx_opmode_streaming)

    print("stateself: ", state_self)
    print("state_objects", state_objects)

    if state_self == 1 or state_objects:
        print("collision!")

    else:
        print("NO Collisions!")







#------------------------------------------------------------------------------------------------------------------------------

# read the position and orientation of the end-effector
position = sim.simxGetObjectPosition(clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
orientation = sim.simxGetObjectOrientation(clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
for i in range(3):
    orientation[i] = round(orientation[i] / math.pi * 180, 2)
print('The position of the end-effector is ' + str(position))
print('The orientation of the end-effector is ' + str(orientation))

print ('Program ended')


#------------------------------------------------------------------------------------------------------------------------------
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)


sim.simxGetPingTime(clientID)


sim.simxFinish(clientID)