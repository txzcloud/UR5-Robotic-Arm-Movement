# Created by: Tom Zheng U98418371
# Python Program calculating the forward Kinematic Matrix based on UR5's DH parameter using 10 sets of
# randomly generated Theta angle.

import numpy as np


# Convert theta to radians
def cTheta(theta):
    theta = np.radians(theta)
    return theta


# Convert alpha to radians
def cAlpha(alpha):
    alpha = np.radians(alpha)
    return alpha


# rotation matrix around X axis with provided theta
def FK(a, alpha, d, theta):
    alpha = cAlpha(alpha)
    theta = cTheta(theta)
    A = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                  [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return np.round(A, 5)


# creating the DH table by taking theta from random generated set of angles
def DH(t1, t2, t3, t4, t5, t6):
    DHTable = np.array([[0, 90, 0.0892, t1-90],
                        [0.425, 0, 0, t2],
                        [0.392, 0, 0, t3],
                        [0, -90, 0.1093, t4],
                        [0, 90, 0.09475, t5],
                        [0, 0, 0.0825, t6],
                        [0, 180, 0, 90]])

    return DHTable


# randomly creating a set of 6 theta angles
def randomTheta():
    randomAngles = np.random.randint(-60, high=60, size=6)
    return randomAngles


# computing the final Forward Kinematics of all 6 joints
def FKJoints(setA):
    joint1 = FK(setA[0][0], setA[0][1], setA[0][2], setA[0][3])
    joint2 = FK(setA[1][0], setA[1][1], setA[1][2], setA[1][3])
    joint3 = FK(setA[2][0], setA[2][1], setA[2][2], setA[2][3])
    joint4 = FK(setA[3][0], setA[3][1], setA[3][2], setA[3][3])
    joint5 = FK(setA[4][0], setA[4][1], setA[4][2], setA[4][3])
    joint6 = FK(setA[5][0], setA[5][1], setA[5][2], setA[5][3])
    joint7 = FK(setA[6][0], setA[6][1], setA[6][2], setA[6][3])

    return np.round(joint1.dot(joint2).dot(joint3).dot(joint4).dot(joint5).dot(joint6).dot(joint7), 5)


print()
print("DH Table format: a, alpha, d, theta.")
print("Each set given below is an array of size 6 of Theta angles randomly generated: ")
print()

set1 = randomTheta()
set2 = randomTheta()
set3 = randomTheta()
set4 = randomTheta()
set5 = randomTheta()
set6 = randomTheta()
set7 = randomTheta()
set8 = randomTheta()
set9 = randomTheta()
set10 = randomTheta()

table1 = FKJoints(DH(set1[0], set1[1], set1[2], set1[3], set1[4], set1[5]))
table2 = FKJoints(DH(set2[0], set2[1], set2[2], set2[3], set2[4], set2[5]))
table3 = FKJoints(DH(set3[0], set3[1], set3[2], set3[3], set3[4], set3[5]))
table4 = FKJoints(DH(set4[0], set4[1], set4[2], set4[3], set4[4], set4[5]))
table5 = FKJoints(DH(set5[0], set5[1], set5[2], set5[3], set5[4], set5[5]))
table6 = FKJoints(DH(set6[0], set6[1], set6[2], set6[3], set6[4], set6[5]))
table7 = FKJoints(DH(set7[0], set7[1], set7[2], set7[3], set7[4], set7[5]))
table8 = FKJoints(DH(set8[0], set8[1], set8[2], set8[3], set8[4], set8[5]))
table9 = FKJoints(DH(set9[0], set9[1], set9[2], set9[3], set9[4], set9[5]))
table10 = FKJoints(DH(set10[0], set10[1], set10[2], set10[3], set10[4], set10[5]))

print("Set 1 Angles: ")
print(set1)
print()
print("Forward Kinematics: ")
print(table1)
print()
print()

print("Set 2 Angles: ")
print(set2)
print()
print("Forward Kinematics: ")
print(table2)
print()
print()

print("Set 3 Angles: ")
print(set3)
print()
print("Forward Kinematics: ")
print(table3)
print()
print()

print("Set 4 Angles: ")
print(set4)
print()
print("Forward Kinematics: ")
print(table4)
print()
print()

print("Set 5 Angles: ")
print(set5)
print()
print("Forward Kinematics: ")
print(table5)
print()
print()

print("Set 6 Angles: ")
print(set6)
print()
print("Forward Kinematics: ")
print(table6)
print()
print()

print("Set 7 Angles: ")
print(set7)
print()
print("Forward Kinematics: ")
print(table7)
print()
print()

print("Set 8 Angles: ")
print(set8)
print()
print("Forward Kinematics: ")
print(table8)
print()
print()

print("Set 9 Angles: ")
print(set9)
print()
print("Forward Kinematics: ")
print(table9)
print()
print()

print("Set 10 Angles: ")
print(set10)
print()
print("Forward Kinematics: ")
print(table10)
