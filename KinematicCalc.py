# Created by: Tom Zheng U98418371
# Simple python program that calculates the Euler and RPY for given theta1, theta2, and theta3

import numpy as np


# rotation matrix around X axis with provided theta
def FK(a, alpha, d, theta):
    A = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                  [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return A


def cTheta(theta):
    theta = np.radians(theta)
    return theta


def cAlpha(alpha):
    alpha = np.radians(alpha)
    return alpha


def storeDH():
    a_param = float(input("Enter value for a: "))
    alpha_param = float(input("Enter value for alpha: "))
    d_param = float(input("Enter value for d: "))
    theta_param = float(input("Enter value for theta: "))
    A = np.around(FK(a_param, cAlpha(alpha_param), d_param, cTheta(theta_param)))
    return A



print("Joint parameters are hardcoded. Please edit code to increase number of joints: ")
print("Current number of Joints: 3")
joint1 = storeDH()
joint2 = storeDH()
joint3 = storeDH()

FKM = joint1.dot(joint2).dot(joint3)
print("The Forward Kinematic Matrix is calculate to be: ")
print(FKM)



