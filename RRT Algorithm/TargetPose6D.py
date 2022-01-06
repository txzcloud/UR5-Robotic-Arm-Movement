import sys
import numpy as np

sys.path.append('PythonAPI')
import math
import time
import random

try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server\n')
else:
    print('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID, "UR5_joint1", sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID, "UR5_joint2", sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID, "UR5_joint3", sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID, "UR5_joint4", sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID, "UR5_joint5", sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID, "UR5_joint6", sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID, "suctionPad", sim.simx_opmode_blocking)

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

# get the collision handles
collision_handle_list = []
for i in range(101):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" + str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)


# You do not need to modify the code above

# --------------------------------------------------------------------------------------------------------------------------------------------------

# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def move_arm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i] / 180 * math.pi, 3))
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID, False)
    time.sleep(0.5)


# --------------------------------------------------------------------------------------------------------------------------------------------------

# function to check collision
def check_collision():
    collision_reading = np.zeros(101)
    is_collision = 0
    for i in range(101):
        collision_reading[i] = sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_buffer)[1]
        if collision_reading[i] == 1:
            is_collision = 1
    if is_collision == 1:
        print('Collision detected!')
        return 1
    else:
        return 0
    return collision_reading


# ------------------------------------------------------------------------------------

print(
    "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
print("Start of HW 7 Q2")

# hardcoded the start and goal target provided by TA in file
start = [[-20, 20, 70, 0, -90, 0],
         [-10, 35, 50, 30, -90, 0],
         [-25, 20, 70, 0, -90, 0],
         [-15, 10, 80, 0, -90, 0],
         [-30, 10, 75, 25, -90, 0],
         [10, -20, -75, 0, 90, 0],
         [5, -50, -45, 0, 90, 0],
         [0, -30, -60, 0, 90, 0],
         [0, -20, -70, 0, 90, 0],
         [15, -20, -70, 0, 90, 0]]

goalPoseSet = [[10, -20, -75, 0, 90, 0],
               [5, -50, -45, 0, 90, 0],
               [0, -30, -60, 0, 90, 0],
               [0, -20, -70, 0, 90, 0],
               [15, -20, -70, 0, 90, 0],
               [-20, 20, 70, 0, -90, 0],
               [-10, 35, 50, 30, -90, 0],
               [-25, 20, 70, 0, -90, 0],
               [-15, 10, 80, 0, -90, 0],
               [-30, 10, 75, 25, -90, 0]]


# increments or decrements the current pose until the goal pose is achieved
def poseMovement(startPose, goalPose):
    for i in range(6):
        if startPose[i] != goalPose[i]:
            if startPose[i] < goalPose[i]:
                startPose[i] += 1
            elif startPose[i] > goalPose[i]:
                startPose[i] -= 1
    return startPose


# initializing the time variables
t_i = 0
t_f = 1


def cubicTrajectory(iniV, finV, time):
    # time is between 0 and 1
    global t_i, t_f

    trajM = np.array([[1, t_i, t_i ** 2, t_i ** 3],
                      [0, 1, 2 * t_i, 3 * (t_i ** 2)],
                      [1, t_f, t_f ** 2, t_f ** 3],
                      [0, 1, 2 * t_f, 3 * (t_f ** 2)]])

    temp = np.array([iniV[0], iniV[1], finV[0], finV[1]])
    trajM = np.linalg.inv(trajM)
    a = trajM.dot(temp)

    qd = a[0] * 1 + a[1] * time + a[2] * (time ** 2) + a[3] * (time ** 3)
    vd = a[1] * 1 + 2 * a[2] * time + 3 * a[3] * (time ** 2)
    ad = 2 * a[2] * 1 + 6 * a[3] * time

    print("The Trajectory velocity at time  {}:\n".format(round(time, 5)), round(qd, 5))
    print("The END Trajectory velocity at time {}:\n".format(round(time, 5)), round(vd, 5))
    print("The Trajectory matrix is: ")
    return trajM


for i in range(len(goalPoseSet)):
    nextPose = poseMovement(start[i], goalPoseSet[i])
    move_arm(nextPose)
    check_collision()

x = np.array([90, 90, 90, 50, 50, 50])
y = np.array([60, 60, 60, 50, 50, 50])

print(cubicTrajectory(x, y, 0.5))

# ------------------------------------------------------------------------------------

# no need to modify the code below
# close the communication between collision handles
for i in range(101):
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)

print('Program ended')

# ------------------------------------------------------------------------------------
