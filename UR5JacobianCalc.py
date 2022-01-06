# Created by: Tom Zheng U98418371
# Python program generating 100 poses of UR5
# With each pose, the jacobian matrix and its determinant is calculated
# Display a histogram of the 100 determinants failed.. Had issues figuring out how to get det of J and J to create histo

import numpy as np
import random
import sympy as sym

thetas = np.array([[0, 0, 0, 0, 0, 0]])


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


# Convert theta to radians
def cTheta(theta):
    theta = np.radians(theta)
    return theta


# Convert alpha to radians
def cAlpha(alpha):
    alpha = np.radians(alpha)
    return alpha


def FK(DH):
    DH[1] = cAlpha(DH[1])
    DH[3] = cTheta(DH[3])
    A = np.array([[np.cos(DH[3]), -np.sin(DH[3]) * np.cos(DH[1]), np.sin(DH[3]) * np.sin(DH[1]), DH[0] * np.cos(DH[3])],
                  [np.sin(DH[3]), np.cos(DH[3]) * np.cos(DH[1]), -np.cos(DH[3]) * np.sin(DH[1]), DH[0] * np.sin(DH[3])],
                  [0, np.sin(DH[1]), np.cos(DH[1]), DH[2]],
                  [0, 0, 0, 1]])
    return np.round(A, 5)


# a, alpha, d, theta
def Jacobian(thetas):
    urs5_DH = [
        [0, 90, 0.0892, thetas[0]],
        [0.425, 0, 0, thetas[1]],
        [0.392, 0, 0, thetas[2]],
        [0, -90, 0.01093, thetas[3]],
        [0, 90, 0.09475, thetas[4]],
        [0, 0, 0.0825, thetas[5]]]

    A = np.zeros((4, 4))
    transforms = []
    Z_s = []
    O_s = []

    # appending all matrices together
    for i in range(len(urs5_DH)):
        trans = FK(urs5_DH[i])
        transforms.append(trans)

    for i in range(len(transforms)):
        if i == 0:
            A = transforms[i]
        else:
            A = np.dot(A, transforms[i])

        Z = (A[0][2], A[1][2], A[1][2])
        O = (A[0][3], A[1][3], A[1][3])
        Z_s.append(Z)
        O_s.append(O)
    # taking Z,O from H to create the Jacobian Matrix
    j = np.zeros((6, 6))
    for y in range(6):
        j[0][y] = np.cross(Z_s[y][0], np.subtract(O_s[5], O_s[y]))
        j[1][y] = np.cross(Z_s[y][1], np.subtract(O_s[5], O_s[y]))
        j[2][y] = np.cross(Z_s[y][2], np.subtract(O_s[5], O_s[y]))
        j[3][y] = Z_s[y]
        j[4][y] = Z_s[y]
        j[5][y] = Z_s[y]

    return j


print(randomPoses())
print(Jacobian(thetas))
