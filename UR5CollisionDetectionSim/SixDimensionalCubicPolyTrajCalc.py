import sys
import numpy as np
# ------------------------------------------------------------------------------------
try:
    import sim
except:
    print('---------------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file')
    print('or appropriately adjust the file "sim.py"')
    print('---------------------------------------------------------------------')
    print('')

print('Program Started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')

# ------------------------------------------------------------------------------------


# initializing the time variables
t_i = 0
t_f = 1

def cubicTrajectory(iniV, finV, time):
    # time is between 0 and 1
    global t_i, t_f

    trajM = np.array([[1,t_i,t_i ** 2, t_i ** 3],
                      [0, 1, 2 * t_i, 3 * (t_i ** 2)],
                      [1, t_f, t_f ** 2, t_f ** 3],
                      [0, 1, 2 * t_f, 3 * (t_f ** 2)]])

    temp = np.array([iniV[0], iniV[1], finV[0], finV[1]])
    trajM = np.linalg.inv(trajM)
    a = trajM.dot(temp)


    qd = a[0]*1 + a[1] * time + a[2] * (time**2) + a[3] * (time**3)
    vd = a[1]*1 + 2*a[2]*time + 3*a[3]*(time**2)
    ad = 2*a[2]*1 + 6*a[3]*time

    print("The Trajectory velocity at time  {}:\n".format(round(time, 5)), round(qd, 5))
    print("The END Trajectory velocity at time {}:\n".format(round(time, 5)), round(vd, 5))
    print("The Trajectory matrix is: ")
    return trajM


x = np.array([90, 90, 90, 50, 50, 50])
y = np.array([60, 60, 60, 50, 50, 50])

print(cubicTrajectory(x, y, 0.5))


# ------------------------------------------------------------------------------------


# ------------------------------------------------------------------------------------
