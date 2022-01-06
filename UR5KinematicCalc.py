# Created by: Tom Zheng U98418371
# Python program calculating the Forward Kinematics Matrix based on UR5's DH parameters

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
    DHTable = np.array([[0, 90, 0.0892, t1],
                        [0.425, 0, 0, t2],
                        [0.392, 0, 0, t3],
                        [0, -90, 0.1093, t4],
                        [0, 90, 0.09475, t5],
                        [0, 0, 0.0825, t6]])

    return DHTable


theta1 = int(input("Enter theta1 value: "))
theta2 = int(input("Enter theta2 value: "))
theta3 = int(input("Enter theta3 value: "))
theta4 = int(input("Enter theta4 value: "))
theta5 = int(input("Enter theta5 value: "))
theta6 = int(input("Enter theta6 value: "))

tableUR5 = DH(theta1, theta2, theta3, theta4, theta5, theta6)

joint1 = FK(tableUR5[0][0], tableUR5[0][1], tableUR5[0][2], tableUR5[0][3])
joint2 = FK(tableUR5[1][0], tableUR5[1][1], tableUR5[1][2], tableUR5[1][3])
joint3 = FK(tableUR5[2][0], tableUR5[2][1], tableUR5[2][2], tableUR5[2][3])
joint4 = FK(tableUR5[3][0], tableUR5[3][1], tableUR5[3][2], tableUR5[3][3])
joint5 = FK(tableUR5[4][0], tableUR5[4][1], tableUR5[4][2], tableUR5[4][3])
joint6 = FK(tableUR5[5][0], tableUR5[5][1], tableUR5[5][2], tableUR5[5][3])

print("UR5 DH parameters: ")
print(tableUR5)

print()
print("DH Table format: a, alpha, d, theta")

print("Joint 1: ")
print(tableUR5[0][0], tableUR5[0][1], tableUR5[0][2], tableUR5[0][3])
print(joint1)
print()

print("Joint 2: ")
print(tableUR5[1][0], tableUR5[1][1], tableUR5[1][2], tableUR5[1][3])
print(joint2)
print()

print("Joint 3: ")
print(tableUR5[2][0], tableUR5[2][1], tableUR5[2][2], tableUR5[2][3])
print(joint3)
print()

print("Joint 4: ")
print(tableUR5[3][0], tableUR5[3][1], tableUR5[3][2], tableUR5[3][3])
print(joint4)
print()

print("Joint 5: ")
print(tableUR5[4][0], tableUR5[4][1], tableUR5[4][2], tableUR5[4][3])
print(joint5)
print()

print("Joint 6: ")
print(tableUR5[5][0], tableUR5[5][1], tableUR5[5][2], tableUR5[5][3])
print(joint6)

# computing the final Forward Kinematics of all 6 joints
print()
print("Forward Kinematics Matrix:  ")
FKM = joint1.dot(joint2).dot(joint3).dot(joint4).dot(joint5).dot(joint6)
print(FKM)
