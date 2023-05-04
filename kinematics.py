"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

D2R = np.pi / 180.0
R2D = 180.0 / np.pi

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    # print("In FK_dh, dh_params = ", dh_params)
    H = np.identity(4);
    for i in range(link + 1):
        H = np.dot(H, get_transform_from_dh(dh_params[i, 0], dh_params[i, 1], dh_params[i, 2], dh_params[i, 3] + joint_angles[i]))
    return H


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """

    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                            [0, np.sin(alpha), np.cos(alpha), d],
                            [0, 0, 0, 1]])


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    euler = np.zeros((3, 1))
    R23 = T[1, 2]
    R13 = T[0, 2]
    R33 = T[2, 2]
    R32 = T[2, 1]
    R31 = T[2, 0]
    a = np.arctan2(R23, R13)
    b = np.arctan2((1 - R33 ** 2) ** 0.5, R33)
    y = np.arctan2(R32, -R31)
    #print(euler.shape)
    euler[0, 0] = a
    euler[1, 0] = b
    euler[2, 0] = y

    return euler


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    phi = get_euler_angles_from_T(T)[1, 0] * R2D
    pose = T[:, 3:4]
    #print("shape of pose = ", pose.shape)
    pose[3, 0] = phi


    return list(pose)


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """

    # R = np.array([[cos(phi), -sin(phi)][sin(phi), cos(phi)]])
    # l1 = 200
    # l2 = 250
    # l3 = 174.15

    # dx = pose[0]
    # dy = pose[1]
    # phi = pose[3]
    # phi = D2R * phi

    # x = dx - l3 * np.cos(phi)
    # y = dy - l3 * np.sin(phi)

    # r = x**2 + y**2
    # x2 = (r - l1**2 - l2**2)/(2*l1*l2)
    # y2 = np.sqrt(1 - x2**2)
    # theta2 = np.arctan2(y2,x2)

    # y1 = ((l1 + l2*x2)*dy - l2*y2*dx)/r
    # x1 = ((l1 + l2*x2)*dx + l2*y2*dy)/r
    # theta1 = np.arctan2(y1,x1)
    # theta3 = phi - theta1 - theta2

    # # dh_params[1][3] = theta1
    # # dh_params[2][3] = theta2
    # # dh_params[3][3] = theta3

    l6 = 174.15
    l2 = np.sqrt(200**2 + 50**2)
    l3 = 200
    d = 103.91
    l2_offset = np.arctan(1.0 / 4.0)
    # print('l2_offset = ', l2_offset)
    # print(l2_offset)

    xo = pose[0]
    yo = pose[1]
    zo = pose[2]
    phi = (pose[3] - 90) * D2R

    # Calculate R
    theta1 = np.arctan2(yo, xo) - np.pi / 2
    # rot_z = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
    # rot_x = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    R = np.array([[-np.sin(phi) * np.sin(theta1), np.cos(theta1), -np.cos(phi) * np.sin(theta1)], [np.sin(phi) * np.cos(theta1), np.sin(theta1), np.cos(phi) * np.cos(theta1)], [np.cos(phi), 0, -np.sin(phi)]], dtype=float)
    # print("R = ", R)
    r13 = R[0, 2]
    r23 = R[1, 2]
    r33 = R[2, 2]
    xc = xo - l6 * r13
    yc = yo - l6 * r23
    zc = zo - l6 * r33
    # print("xo, yo, zo = ", xo, yo, zo)
    # print("xc, yc, zc = ", xc, yc, zc)
    r = np.sqrt(xc ** 2 + yc ** 2)
    s = zc - d

    # print("r, s = ", r, " ", s)
    # print(r ** 2)
    # print(s ** 2)
    # print(l2 ** 2)
    # print(l3 ** 2)
    # print((r ** 2 + s ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3))
    sign_list = [-1, 1]
    IK_output_list = []
    IK_angle_list = []

    for sign in sign_list:


        theta3 = sign * np.arccos((r ** 2 + s ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3))
        theta2 = np.arctan2(s, r) - np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
        # print('ang 1 = ', np.arctan2(s, r))
        # print('ang 2 = ', np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3)))

        theta4 = - phi - (theta2 + theta3)
        # print("theta1 = ", theta1 * R2D, " theta2 = ", theta2 * R2D, " theta3 = ", theta3 * R2D, " theta4 = ", theta4 * R2D)
        # Transform from theta to joint angle
        angle1 = theta1
        angle2 = np.pi / 2.0 - theta2 - l2_offset
        angle3 = np.pi / 2.0 + theta3 - l2_offset
        angle4 = theta4
        # print(np.pi / 2)

        # print(angle1, angle2, angle3, angle4)

        # IK_output = np.array([[angle1 * R2D, angle2 * R2D, angle3 * R2D, angle4 * R2D]])
        # print(np.array([[angle1 * R2D, angle2 * R2D, angle3 * R2D, angle4 * R2D]]))


        IK_output = np.array([[clamp(angle1), clamp(angle2), clamp(angle3), clamp(angle4)]])
        IK_angle = np.array([[angle1 * R2D, angle2 * R2D, angle3 * R2D, angle4 * R2D]])

        IK_output_list.append(IK_output)
        IK_angle_list.append(IK_angle)

    # print("IK_angle_list = ", IK_angle_list)

    for output in IK_output_list:
        output_sum = np.sum(output)
        if (not np.isnan(output_sum)):
            return output

    return np.array([[float('NaN'), float('NaN'), float('NaN'), float('NaN')]])





    return IK_output
