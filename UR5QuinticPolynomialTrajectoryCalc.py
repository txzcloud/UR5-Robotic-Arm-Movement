# Created by: Tom Zheng U98418371
# Python program generating a one dimensional quintic polynomial trajectory that takes an initial p0 and final p1 at a time t

import numpy as np

t_i = 0
t_f = 1


def quinticTrajectory(iniV, finV, time):
    # time is between 0 and 1
    global t_i, t_f

    trajM = np.array([[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
                      [0, 1, 2 * t_i, 3 * (t_i ** 2), 4 * (t_i ** 3), 5 * (t_i ** 4)],
                      [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
                      [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
                      [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
                      [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]])
    temp = np.array([iniV[0], iniV[1], iniV[2], finV[0], finV[1], finV[2]])

    trajM = np.linalg.inv(trajM)
    trajV = trajM.dot(temp)
    trajPosition = trajV[0] * 1 + trajV[1] * time + trajV[2] * (time ** 2) + trajV[3] * (time ** 3) + trajV[4] * (
                time ** 4) + trajV[5] * (time ** 5)
    trajVelo = trajV[1] + 2 * trajV[2] * time + 3 * trajV[3] * (time ** 2) + 4 * trajV[4] * (time ** 3) + 5 * trajV[
        5] * (time ** 4)

    print("The Trajectory position at time {}:\n".format(round(time, 5)), round(trajPosition, 5))
    print("The Trajectory velocity at time {}:\n".format(round(time, 5)), round(trajVelo, 5))
    print("The Trajectory matrix is: ")
    return trajM


x = np.array([0, 0, 0])
y = np.array([1, 1, 1])

print(quinticTrajectory(x, y, 0.5))
