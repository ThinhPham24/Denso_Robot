import math
import numpy as np
from numpy.linalg import inv
import cv2

# Calculates Rotation Matrix given euler angles.
def euler2Rotation(pose) :
    alpha = pose[3] * math.pi/180
    beta = pose[4] * math.pi/180
    gamma = pose[5] * math.pi/180
    R_x = np.array([[1,         0,                  0,                  0],
                    [0,         math.cos(alpha), -math.sin(alpha),      0],
                    [0,         math.sin(alpha), math.cos(alpha),       0],
                    [0,         0,               0,                     1]])

    R_y = np.array([[math.cos(beta),    0,      math.sin(beta),         0],
                    [0,                     1,      0,                  0],
                    [-math.sin(beta),   0,      math.cos(beta),         0],
                    [0,         0,               0,                     1]])

    R_z = np.array([[math.cos(gamma),    -math.sin(gamma),    0,        0],
                    [math.sin(gamma),    math.cos(gamma),     0,        0],
                    [0,                  0,                   1,        0],
                    [0,                  0,                   0,        1]])

    R = np.dot(R_z, np.dot( R_y, R_x ))
    R[0,3] = pose[0]
    R[1,3] = pose[1]
    R[2,3] = pose[2]

    return R

# of the euler angles ( x and z are swapped ).
def rotation2Euler(R) :

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6
    x = R[0,3]
    y = R[1,3]
    z = R[2,3]

    if  not singular :
        Rx = math.atan2(R[2,1] , R[2,2])
        Ry = math.atan2(-R[2,0], sy)
        Rz = math.atan2(R[1,0], R[0,0])
    else :
        Rx = math.atan2(-R[1,2], R[1,1])
        Ry = math.atan2(-R[2,0], sy)
        Rz = 0
    Rx = Rx*180/math.pi
    Ry = Ry*180/math.pi
    Rz = Rz*180/math.pi
    return np.array([round(x,2), round(y,2), round(z,2), round(Rx,2), round(Ry,2), round(Rz,2)])
def robot2StationCut(object_pos, angleZ): ## Cutting
    delX = 0
    delY = 0
    delZ = 0
    object_matrix = euler2Rotation(object_pos)
    robot_station = euler2Rotation([-178.12, -280.889, -125.98, 0, 0, 0])
    # robot_station = euler2Rotation([-178.12, -280.889, -125.98,0, 0, 0])
    rotate = euler2Rotation([0,0,0,0,0,angleZ])
    tempend1 = np.matmul(robot_station, object_matrix)
    tempend2 = np.matmul(rotate,tempend1)
    tempend3 = np.matmul(inv(robot_station),tempend2)
    robot_eul = rotation2Euler(tempend3)

    return robot_eul
def robot2StationHold(object_pos, angleZ): ## Cutting

    object_matrix = euler2Rotation(object_pos)
    robot_station = euler2Rotation([-424.93, 426.36, -314.04, 0, 0, 0])
    rotate = euler2Rotation([0,0,0,0,0,angleZ])
    tempend1 = np.matmul(robot_station, object_matrix)
    tempend2 = np.matmul(rotate,tempend1)
    tempend3 = np.matmul(inv(robot_station),tempend2)

    robot_eul = rotation2Euler(tempend3)

    return robot_eul
def object2BaseCut(object_pos): ## Cutting
    delX = 12.59
    delY = -11.65
    delZ = 4
    object_matrix = euler2Rotation(object_pos)
    camera2End =np.array([[0, -0.9939, -0.0349, 190.71 + delX], # Rx = 178, Ry = 0, Rz = -90
                          [-1, 0, 0, 321.01 + delY],
                          [0, 0.0349, -0.9994, 591.62 + delZ],
                          [ 0,  0,  0,  1]])
    # camera2EndTemp = rotation2Euler(camera2End)
    # camera2EndTemp[3]= 178
    # camera2EndTemp[4]= 0
    # camera2EndTemp[5]= -90
    # camera2End = euler2Rotation(camera2EndTemp)
    # print(camera2End)
    tempend1 = np.matmul(camera2End, object_matrix)
    robot_eul = rotation2Euler(tempend1)
    return robot_eul
def object2BaseHold(object_pos): ## holding
    delX = 10.7
    delY = -21.17
    delZ = -68.5
    object_matrix = euler2Rotation(object_pos)
    camera2End =np.array([[0, -0.9939, -0.0349, 439 + delX], # Rx = 178, Ry = 0, Rz = -90
                          [-1, 0, 0, -376.5 + delY],
                          [0, 0.0349, -0.9994, 775.68 + delZ],
                          [ 0,  0,  0,  1]])
    # camera2EndTemp = rotation2Euler(camera2End)
    # camera2EndTemp[3]= 178
    # camera2EndTemp[4]= 0
    # camera2EndTemp[5]= -90
    # camera2End = euler2Rotation(camera2EndTemp)
    # print(camera2End)
    tempend1 = np.matmul(camera2End, object_matrix)
    robot_eul = rotation2Euler(tempend1)
    return robot_eul
def objectNormal(robot_pos, x):
    
    robot_matrix = euler2Rotation(robot_pos)
   

    end2Tool =np.array([[1, 0, 0, x], 
                [0, 1, 0, 0],
                [0, 0, 1, 0], 
                [ 0,  0,  0,  1]])

    tempend1 = np.matmul(robot_matrix, end2Tool)
 
    robot_eul = rotation2Euler(tempend1)
    return robot_eul

