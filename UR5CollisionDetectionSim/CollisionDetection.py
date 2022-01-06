import sys
import numpy as np
sys.path.append('PythonAPI')
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
    print ('Connected to remote API server\n')
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

# get the collision handles
collision_handle_list = []
for i in range(101):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" + str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)

# You do not need to modify the code above

#--------------------------------------------------------------------------------------------------------------------------------------------------

# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def move_arm(armpose):
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
    time.sleep(0.2)

#--------------------------------------------------------------------------------------------------------------------------------------------------
#global variable for number of set poses -- change var setRange to increase/decrease # of sets of poses
setRange = 101

#first set given by TA example, this set of poses gives a collision, for testing purposes
thetas = np.array([[50, 60, 80, 60, -90, 0]])
# randomly creating 100 set of 6 theta angles from range -60 to 60 degrees
def randomPoses():
    global thetas
    for i in range(0, setRange):
        # thetas = np.append(thetas, np.random.randint(-60, high=60, size=6))
        thetas = np.append(thetas, [[random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     random.randint(-60, 60),
                                     ]], axis=0)
    return thetas


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

#empty array to hold all indices of collision and non collision in the 100 sets of poses
indexTemp = []
#storing randomly generated 100 set of poses
setsOfPoses = randomPoses()
#looping through each pose movement into sim and checking for collision then appending indices of collision into temp array;
for i in range(setRange):
    move_arm(setsOfPoses[i])
    indexTemp.append(check_collision())
print()
print("Array of collision Indices: \n", indexTemp)
print()

# first set of poses provided by TA, for testing purposes. It will be removed in the end
F = np.array([[50, 60, 80, 60, -90, 0]])
# looping through index array to find all collisionFREE poses and store in array called F
for i in range(1,setRange):
    if indexTemp[i] == 0:
        F = np.append(F, [[thetas[i][0],
                                     thetas[i][1],
                                     thetas[i][2],
                                     thetas[i][3],
                                     thetas[i][4],
                                     thetas[i][5],
                                     ]], axis=0)
# deleting the first given collision poses provided by TA's example
F = np.delete(F, 0, 0)
print("The following is the set of COLLISION-FREE Poses: \n", F)
print()
print("End of Q1 Answer")
print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
#======================================================================================================================
# Above, q1 is completed and all free pose theta set is stored in the list freepose
# Below, q2, take output F, randomly select a set pose from F
# a). closest neighbor pose B in F, which can go to pose A on a straight line in the c-space without collision
# b). farthest pose C in F, which can go to A on a straight line in the c-space without collide with anything
#======================================================================================================================

# randomly selects a free pose A from F
randomA = random.randint(0,len(F)-1)
A = F[randomA]
print()
print("Randomly Selected Pose A is: \n", A)
print()
# this deletes the selected Pose A from the remaining freepose list to avoid the closest neighbor to be A itself
newF = np.delete(F, randomA, 0)

# setting an arbitrary max and min distance for comparison measures
minDist = 66666
maxDist = -66666
closestPose = np.empty([1,6])
farPose = np.empty([1,6])

# looping through the new set of freespace poses to find the closest neighbor and furthest neighbor to the CHOSEN Pose A
for i in range(len(newF)):
    # calculating the distance
    distance = np.linalg.norm(np.array(A)-np.array(newF[i]))
    if distance < minDist:
        closestPose = newF[i]
        minDist = distance
    if distance > maxDist:
        farPose = newF[i]
        maxDist = distance

print("Closest Neighbor Pose B to A: \n", np.round(closestPose))
print("With a minimum distance of: ", round(minDist,5))
print()
print("Furthest Pose C to A: \n", np.round(farPose))
print("With a maximum distance of: ", round(maxDist,5))
print()
print("End of Q2 Answer")
print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")


#-----------------------------------------------------------------------------------------------------------------------
# no need to modify the code below
# close the communication between collision handles
for i in range(101):
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)

print ('Program ended')

#-------------------------------------------------------------------------------------------------------------------------------------------------