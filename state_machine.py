"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import timeit
import numpy as np
import rospy
import cv2
import csv
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from kinematics import clamp

D2R = np.pi / 180.0
R2D = 180.0 / np.pi

class Mask():
    def __init__(self, x_center, y_center, radius):
        self.x_center = x_center
        self.y_center = y_center
        self.radius = radius

class WaypointRecording():
    def __init__(self, arm_coords=[], gripper_state=[]):
        self.arm_coords = arm_coords
        self.gripper_state = gripper_state

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.gripper_state = 1

        self.waypoints = WaypointRecording()

        self.phi_down_place = 0
        # self.waypoints.arm_coords = [
        #     [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
        #     [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
        #     [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
        #     [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
        #     [0.0,             0.0,      0.0,         0.0,     0.0],
        #     [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
        #     [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
        #     [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
        #     [np.pi/2,         0.5,     0.3,      0.0,     0.0],
        #     [0.0,             0.0,     0.0,      0.0,     0.0]]

        # self.waypoints.gripper_state = [1,1,1,1,1,1,1,1,1,1]



    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()


    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    # TODO: change move time back to 0.8
    def execute(self, k_move=1.0, k_accel=1.0/5, min_move_time=4.0, sleep_time=0.4):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        # #self.rxarm.set_moving_time(2.0)
        # k_move = 1.2
        # k_accel = 1.0/10
        # min_move_time = 0.5

        self.status_message = "State: Execute - Executing motion plan"
        # current_position = self.rxarm.get_positions()
        # print(current_position)
        # print(type(current_position))
        for i in range(len(self.waypoints.arm_coords)):
            current_position = self.rxarm.get_positions()# Get current position
            current_position = [clamp(deg) for deg in current_position]
            next_position = self.waypoints.arm_coords[i] # Get next position
            next_position = [clamp(deg) for deg in next_position]
            # print("next_position = ", next_position)
            difference = np.absolute(np.subtract(current_position[:-1], next_position[:-1])) # Difference between current and next position
            # print('Difference Position')
            # print(difference)
            max_angle_disp = np.amax(difference)# Find highest angle displacement
            # print('Max Displacement')
            # print(max_angle_disp)
            moving_time = k_move * max_angle_disp# Multiply the above by constant to get time
            # print("max_ang = ", max_angle_disp)
            # print("moving_time = ", moving_time)
            if (moving_time < min_move_time):
               moving_time = min_move_time
            #print('Moving Time')
            #print(moving_time)
            accel_time = k_accel * moving_time
            #print('Acceleration Time')
            #print(accel_time)
            self.rxarm.set_moving_time(moving_time)# Do set moving time
            self.rxarm.set_accel_time(accel_time)
            self.rxarm.set_positions(next_position)
            # TODO: Maybe uncomment this if no work
            rospy.sleep(moving_time)
            if(self.waypoints.gripper_state[i]):
                self.rxarm.open_gripper()
            else:
                self.rxarm.close_gripper()
            # TODO: Maybe uncomment this if no work
            rospy.sleep(sleep_time)
        self.next_state = "idle"
        self.clear_waypoints()

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        tag_position_c = np.zeros((4,3))
        print('Calibration')
        if len(self.camera.tag_detections.detections) < 4:
            self.status_message = "Not enough tags"
        else:
            for i in range(4):
                id1 = self.camera.tag_detections.detections[i].id[0] - 1
                tag_position_c[id1,0] = self.camera.tag_detections.detections[i].pose.pose.pose.position.x
                tag_position_c[id1,1] = self.camera.tag_detections.detections[i].pose.pose.pose.position.y
                tag_position_c[id1,2] = self.camera.tag_detections.detections[i].pose.pose.pose.position.z

        tag_position_c = np.transpose(tag_position_c).astype(np.float32)
        tag_position_c_scaled = tag_position_c * 1000
        for i in range(4):
            tag_position_c[:, i] /=  tag_position_c[2,i]

        tag_position_i = np.dot(self.camera.intrinsic_matrix,tag_position_c).astype(np.float32)

        #print("U V coordinates")
        #print(tag_position_i)
        #tag_position_i = tag_position_i
        self.status_message = "Calibration - Completed Calibration"


        (success, rot_vec, trans_vec) = cv2.solvePnP(self.camera.tag_locations.astype(np.float32), np.transpose(tag_position_i[:2, :]).astype(np.float32), self.camera.intrinsic_matrix,self.camera.dist_coefficient, flags = cv2.SOLVEPNP_ITERATIVE)


        dst = cv2.Rodrigues(rot_vec)
        dst = np.array(dst[0])

        trans_vec = np.squeeze(trans_vec)
        self.camera.extrinsic_matrix[:3, :3] = dst
        self.camera.extrinsic_matrix[:3, 3] = trans_vec


        # Tilted plane fix
        uv_coords = tag_position_i.astype(int)
        intrinsic_inv = np.linalg.inv(self.camera.intrinsic_matrix)
        c_coords =  np.matmul(intrinsic_inv, uv_coords)

        for i in range(4):
            tag_position_c[:, i] /=  tag_position_c[2,i]
            z = self.camera.DepthFrameRaw[uv_coords[1,i]][uv_coords[0,i]]
            c_coords[:,i] *= z

        #print(c_coords.shape)
        #print([float(1),float(1),float(1),float(1)])
        c_coords = np.append(c_coords, [[float(1),float(1),float(1),float(1)]], axis=0)
        w_coords = np.matmul(np.linalg.inv(self.camera.extrinsic_matrix), c_coords)
        #print(w_coords)

        # Cross product of tilted frame
        id1 = 1;
        id2 = 2;
        cross_w = np.cross(w_coords[:3,id1], w_coords[:3,id2])
        b=np.linalg.norm(cross_w)
        cross_w = cross_w / b

        w_points = np.append(np.expand_dims(w_coords[:3,id1], axis = 1),np.expand_dims(w_coords[:3,id2], axis = 1), axis = 1)
        w_points = np.append(w_points,np.expand_dims(cross_w, axis = 1), axis = 1)
        #print(w_points)

        # Cross product of true locations
        true_locations = np.transpose(self.camera.tag_locations)
        cross_t = np.cross(true_locations[:,id1], true_locations[:,id2])
        t=np.linalg.norm(cross_t)
        cross_t = cross_t / t

        t_points = np.append(np.expand_dims(true_locations[:,id1], axis = 1),np.expand_dims(true_locations[:,id2], axis = 1), axis = 1)
        t_points = np.append(t_points,np.expand_dims(cross_t, axis = 1), axis = 1)
        #print(t_points)

        # Cross product for rotation axis
        rot_axis = np.cross(cross_w, cross_t)
        mag_rot=np.linalg.norm(rot_axis)
        rot_axis = rot_axis / mag_rot
        #print(rot_axis)

        # Angle of rotation
        dot_product = np.dot(cross_w, cross_t)
        angle = -np.arccos(dot_product)/2

        # Quaternion rotation around the axis
        q_rotation = Quaternion(axis = rot_axis, angle = angle)
        #print(q_rotation)

        # From Quaternion to rotation
        #R = q_rotation.transformation_matrix
        #print(q_rotation.transformation_matrix)
        R = np.array([[1, 0, 0, 0],[0 ,math.cos(angle), - math.sin(angle), 0],[0, math.sin(angle), math.cos(angle), 10], [0, 0, 0, 1]])


        self.camera.extrinsic_matrix = np.matmul(self.camera.extrinsic_matrix, np.linalg.inv(R))
        #
        #print(angle)
        #print(R)
        self.camera.processDepthFrame()


        print(self.camera.extrinsic_matrix)


    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"

    def add_waypoint(self):

        self.waypoints.arm_coords.append(self.rxarm.get_positions())
        self.waypoints.gripper_state.append(self.gripper_state)
        print(self.waypoints.arm_coords)

    def add_gripper(self, state):
        self.gripper_state = state

    def save_waypoints(self):
        # Write out to file
        with open("teach_repeat.csv", "wb") as file:
            writer = csv.writer(file)
            for i in range(len(self.waypoints.arm_coords)):
                writer.writerow(self.waypoints.arm_coords[i][0])
                writer.writerow(self.waypoints.gripper_state[i][0])
                # print(type(self.waypoints.arm_coords))
                # print(self.waypoints.arm_coords[i])
                # print(self.waypoints.arm_coords[i][0])
    def load_waypoints(self):
        with open("teach_repeat.csv", "r") as file:
            reader = csv.reader(file, delimeter = '\t')
            for row in reader:
                print(row)

    def clear_waypoints(self):
        self.waypoints.arm_coords = []
        self.waypoints.gripper_state = []

    def pick_click(self):
        print('Camera Coordinates')
        print(self.camera.last_click)
        x = self.camera.last_click[0]
        y = self.camera.last_click[1]
        z = self.camera.DepthFrameRaw[y][x]
        # from click get world coordinates
        w_coords = self.camera.u_v_d_to_world(x,y,z)

        self.pick("big",w_coords)

    def pick(self, block_size, w_coords, block_theta=0, height=150,k_move=0.8, k_accel=1.0, min_move_time=0.5, phi_i=175):
        # Append phi angle to w_coords

        # phi_i = 175
        phi_up = phi_i
        phi_down = phi_i
        # w_coords = np.append(w_coords, phi_i)

        # Increase z by 30 mm (avoid hitting ground)
        w_coords_up = w_coords.copy()
        w_coords_up = np.append(w_coords_up, phi_up)
        w_coords_down = w_coords.copy()
        w_coords_down = np.append(w_coords_down, phi_down)

        # lower the down point for to the sky
        w_coords_down[2] -= 15;

        w_coords_up_z = w_coords[2] + height
        w_coords_up[2] = w_coords_up_z

        #         # TODO: Not this, manually adjust z for bad calibration in stack
        # if(w_coords_down[0] < -350 or w_coords_down[0] > 350 or w_coords_down[1] > 370):
        #     w_coords_down[2] = w_coords_down[2] + 10

        # IK
        # block_rot = 0;

#         while ((any(np.isnan(self.rxarm.world_to_joint(w_coords_up)[0])) or any(np.isnan(self.rxarm.world_to_joint(w_coords_down)[0]))) and phi >= 80):
# #            print("trying phi = ", phi)
#             phi -= 5
#             w_coords[3] = phi

#             # Increase z by 30 mm (avoid hitting ground)
#             w_coords_up = w_coords.copy()
#             w_coords_down = w_coords.copy()
#             w_coords_up[2] += height

#             if w_coords_down[2] < 5.0:
#                 w_coords_down[2] += 20.0

    # TODO: Commented this out to test, reverted to prior ik
        while (any(np.isnan(self.rxarm.world_to_joint(w_coords_up)[0])) and phi_up >= 5):
    #            print("trying phi = ", phi)
            phi_up -= 5
            w_coords_up[3] = phi_up

            # Increase z by 30 mm (avoid hitting ground)
            # w_coords_up = w_coords.copy()
            # w_coords_down = w_coords.copy()
            w_coords_up[2] = w_coords_up_z

        while (any(np.isnan(self.rxarm.world_to_joint(w_coords_down)[0])) and phi_down >= 5):
    #            print("trying phi = ", phi)
            phi_down -= 5
            w_coords_down[3] = phi_down

            # Increase z by 30 mm (avoid hitting ground)
            # w_coords_up = w_coords.copy()
            # w_coords_down = w_coords.copy()
            # w_coords_up[2] += height

            if w_coords_down[2] < 5.0:
                w_coords_down[2] += 20.0

            # # TODO: Not this, manually adjust z for bad calibration in stack
            # if(w_coords_down[0] < -350 or w_coords_down[0] > 350 or w_coords_down[1] > 370):
            #     w_coords_down[2] = w_coords_down[2] + 10

        # self.phi_down_place = phi_down


        # Adjust pick height for small blocks
#        if block_size == "small":
#                w_coords_down[2]





        joint_angles_up = self.rxarm.world_to_joint(w_coords_up)

        joint_angles_up.flatten()
        if phi_down == phi_i:
            block_rot = clamp(D2R * (block_theta + 90 + R2D * (joint_angles_up[0][0])))
        else:
            block_rot = 0

        # if block_rot < 0:
        #     block_rot += 180
        joint_angles_up = np.append(joint_angles_up, block_rot)
        self.waypoints.arm_coords.append(joint_angles_up)
        self.waypoints.gripper_state.append(1)
        # print(self.waypoints.arm_coords)

        joint_angles_down = self.rxarm.world_to_joint(w_coords_down)
        joint_angles_down.flatten()
        joint_angles_down = np.append(joint_angles_down, block_rot)

        self.waypoints.arm_coords.append(joint_angles_down)
        self.waypoints.gripper_state.append(0)

        self.waypoints.arm_coords.append(joint_angles_up)
        self.waypoints.gripper_state.append(0)

        self.execute(k_move, k_accel, min_move_time, sleep_time=0.3);
        # print(self.waypoints.arm_coords)

        # define trajectory based on click
        # use inverse kinematics to calculate joint position
        # Add to join positions waypoint to self.waypoints
        # make sure end effector closes at the end


    def place_click(self):
        print('Camera Coordinates')
        print(self.camera.last_click)
        x = self.camera.last_click[0]
        y = self.camera.last_click[1]
        z = self.camera.DepthFrameRaw[y][x]
        # from click get world coordinates
        c_coords = [x,y,z]
        w_coords = self.camera.u_v_d_to_world(x,y,z)
        self.place(c_coords, w_coords)
        # self.place(c_coords, w_coords, phi_i=85, not_rot=True)

    def place(self, c_coords, w_coords, block_theta=0, mask_placement=0, block_size="big", height=80.0, k_move=0.8, k_accel=1.0, min_move_time=1.0, phi_i=175, not_rot=False):
        # Append phi angle to w_coords
        # phi_i = 175
        phi_up = phi_i
        phi_down = phi_i

        w_coords_up = w_coords[::]
        w_coords_up = np.append(w_coords_up, phi_up)
        w_coords_down = w_coords[::]
        w_coords_down = np.append(w_coords_down, phi_down)

        w_coords_up_z = w_coords[2] + height
        w_coords_up[2] = w_coords_up_z

    # TODO: Uncomment if needed
        while (any(np.isnan(self.rxarm.world_to_joint(w_coords_up)[0])) and phi_up >= 5):
    #            print("trying phi = ", phi)
            phi_up -= 5
            w_coords_up[3] = phi_up

            # Increase z by 30 mm (avoid hitting ground)
            # w_coords_up = w_coords.copy()
            # w_coords_down = w_coords.copy()
            w_coords_up[2] = w_coords_up_z

        while (any(np.isnan(self.rxarm.world_to_joint(w_coords_down)[0])) and phi_down >= 5):
    #            print("trying phi = ", phi)
            phi_down -= 5
            w_coords_down[3] = phi_down

            # Increase z by 30 mm (avoid hitting ground)
            # w_coords_up = w_coords.copy()
            # w_coords_down = w_coords.copy()
            # w_coords_up[2] += height

            if w_coords_down[2] < 5.0:
                w_coords_down[2] += 20.0



        # # Append phi angle to w_coords
        # phi_i = 175
        # # phi_i = self.phi_down_place
        # phi = phi_i
        # w_coords = np.append(w_coords, phi)

        # # Increase z by 30 mm (avoid hitting ground)
        # w_coords_up = w_coords.copy()
        # w_coords_down = w_coords.copy()
        # # IK

        # w_coords_up[2] += height

        # while ((any(np.isnan(self.rxarm.world_to_joint(w_coords_up)[0])) or any(np.isnan(self.rxarm.world_to_joint(w_coords_down)[0]))) and phi_down >= 80):
        #     # print("trying phi = ", phi)
        #     phi_down -= 5

        #     # Increase z by 30 mm (avoid hitting ground)
        #     w_coords_up = w_coords.copy()
        #     w_coords_down = w_coords.copy()
        #     w_coords_up = w_coords[::]
        #     w_coords_up = np.append(w_coords_up, phi_up)
        #     w_coords_down = w_coords[::]
        #     w_coords_down = np.append(w_coords_down, phi_down)
        #     w_coords_down[3] = phi_down
        #     w_coords_up[2] += height

        #     if w_coords_down[2] < 5.0:
                # w_coords_down[2] += 20.0


        # # TODO: Not this, manually adjust z for bad calibration in stack
        # if(w_coords_down[0] < -350 or w_coords_down[0] > 350 or w_coords_down[1] > 370 or w_coords_down[1] < 0):
        #     w_coords_down[2] = w_coords_down[2] + 10


        if block_size == "small":
                w_coords_down[2] -= 5

         #200.0
        w_coords_down[2] += 30.0

        joint_angles_up = self.rxarm.world_to_joint(w_coords_up)
        # print("first = ", joint_angles_up)

        joint_angles_up.flatten()
        # print("second = ", joint_angles_up)

        if phi_down == phi_i:
            block_rot = clamp(D2R * (block_theta + 90 + R2D * (joint_angles_up[0][0])))
        else:
            block_rot = 0

        if not_rot:
            block_rot = 0
        # if block_rot < 0:
        #     block_rot += 180
        # print("block_rot = ", block_rot)
        joint_angles_up = np.append(joint_angles_up, block_rot)

        self.waypoints.arm_coords.append(joint_angles_up)
        self.waypoints.gripper_state.append(0)
        # print(self.waypoints.arm_coords)

        joint_angles_down = self.rxarm.world_to_joint(w_coords_down)

        joint_angles_down.flatten()
        joint_angles_down = np.append(joint_angles_down, block_rot)
        self.waypoints.arm_coords.append(joint_angles_down)
        self.waypoints.gripper_state.append(1)
        # same as pick but open end effector at the end

        self.waypoints.arm_coords.append(joint_angles_up)
        self.waypoints.gripper_state.append(1)

        # Create mask for placement
        # TODO: Convert world back to camera coords
        if(mask_placement):
            print("PLACING MASK !!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            if(w_coords[0] < 0):
                disp = 1.4
            else:
                disp = 1.4
            if(block_size == "small"):
                msk = Mask(int(c_coords[0]-disp), int(c_coords[1]), 50)
            else:
                msk = Mask(int(c_coords[0]-disp), int(c_coords[1]), 100)
            self.camera.mask_list.append(msk)

        self.execute(k_move, k_accel, min_move_time, sleep_time=0.3);

    # Figure out if far away
    # def is_far_away(self, coord1, coord2, 400):
    #     x_d = coord2[0] - coord1[0]
    #     y_d = coord2[1] - coord1[1]

    #     if(math.sqrt(x_d*x_d + y_d*y_d) > thresh):
    #         return 1
    #     return 0

    # Event 1
    def pick_sort(self):

        # Image coordinates
        destination_right_uv = [[780,590,970],[780,670,970],[860,590,970],[860,670,970],[940,590,970],[940,670,970],[1020,590,970],[1020,670,970],[1100,590,970]]
        destination_left_uv = [[540,590,970],[540,670,970],[500, 590,970],[500,670,970],[460,590,970],[460, 570,970],[420,590,970],[420,670,970],[380, 590,970]]

        destination_right = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in destination_right_uv]
        destination_left = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in destination_left_uv]

        print(destination_right_uv)
        print(destination_right)
        print(destination_left_uv)
        print(destination_left)
        #destination_right = [[155,50,0],[155,-30,0],[205,-110,0]]
        #destination_left = [[-155,50,0],[-155,-30,0],[-205,-110,0]]

        self.camera.blockDetector()

        i = 0
        j = 0

        last_coords = []
        while len(self.camera.block_detections) > 0:
            for block in self.camera.block_detections:
                w_coords = block.coord
                print(w_coords)
                self.pick(block.size, w_coords, block.theta)
                # if(is_far_away(w_coords, last_coords)):
                #     avg_waypoint = []
                #     avg_waypoint.append((w_coords[0] + last_coords[0]) / 2)
                #     avg_waypoint.append((w_coords[1] + last_coords[1]) / 2)
                #     avg_waypoint.append(150)
                #     self.execute(avg_waypoint)
                if(block.size == "big"):
                    self.place(destination_right_uv[i], destination_right[i], block_theta=90, mask_placement=1, block_size=block.size, height=150)
                    last_coords = destination_right[i]
                    i += 1
                else:
                    self.place(destination_left_uv[j], destination_left[j], block_theta=90, mask_placement=1, block_size=block.size, height=150)
                    last_coords = destination_left[j]
                    j += 1

            self.rxarm.sleep()
            rospy.sleep(2)
            self.camera.blockDetector()
            rospy.sleep(1)

        self.camera.mask_list = []

    # Event 2
    def pick_stack(self):

        # Image coordinates
        destination_bases_uv = [[800,620,979],[875,620,977],[950,620,975]]
        destination_buff_uv = [[540,590,970],[540,670,970],[460, 590,970],[460,670,970],[380,590,970],[380,670,970],[300,590,970],[300,670,970],[450, 550,970]]

        destination_bases = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in destination_bases_uv]
        destination_buff = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in destination_buff_uv]

        self.camera.blockDetector()
        i = 0
        j = 0
        state = "stack_bigs"
        block_detect_process = []
        max_height = 0
        while len(self.camera.block_detections) > 0:
            # Sort
            for block in self.camera.block_detections:
                if(block.size == "big"):
                    block_detect_process.append(block)
            for block in self.camera.block_detections:
                if(block.size == "small"):
                    block_detect_process.append(block)

            # Change state
            if(block_detect_process[0].size == "big"):
                state = "stack_bigs"
            print("HELLO !!!!!!!!!!!!!!!!!!!!")
            print(block_detect_process[0].coord[2])
            print(block_detect_process[0].size)
            if(block_detect_process[0].coord[2] < 40 and block_detect_process[0].size == "small"):
                print(block_detect_process[0].coord[2])
                print(block_detect_process[0].size)
                state = "stack_smalls"

            if(state == "stack_bigs"):
                for block in block_detect_process:
                    w_coords = block.coord
                    print(w_coords)
                    # Grab all big blocks first to form the stack bases

                    if(block.size == "big"):
                        self.pick(block.size, w_coords, block.theta, height=150, )
                        self.place(destination_bases_uv[i%3], destination_bases[i%3], 0, 1, block.size, height=max_height+100)
                        print("PLACING AT DESTINATION: ")
                        print(destination_bases[i%3])
                        destination_bases[i%3][2] = destination_bases[i%3][2] + 41
                        print("NEW DESTINATION: ")
                        print(destination_bases[i%3])
                        if(i%3 == 0):
                            max_height += 50
                        i += 1
                    elif(block.size == "small" and block.coord[1] > 120):
                        self.pick(block.size, w_coords, block.theta, height=150)
                        self.place(destination_buff_uv[j], destination_buff[j], 0, 0, block.size, height=150)
                        j += 1

            elif(state == "stack_smalls"):
                print("I AM IN STACK SMALL STATE")
                for block in block_detect_process:
                    w_coords = block.coord
                    print(w_coords)

                    self.pick(block.size, w_coords, block.theta, height=150)
                    self.place(destination_bases_uv[i%3], destination_bases[i%3], 0, 1, block.size, height=100+max_height)
                    destination_bases[i%3][2] += 25
                    if(i%3==0):
                        max_height += 35
                    i += 1

            block_detect_process = []


            self.rxarm.sleep()
            rospy.sleep(2)
            #
            self.camera.blockDetector(660, 1200, 400, 700)
            rospy.sleep(1)
        self.camera.mask_list = []

        # Event 3
    def line_up(self):
        # Image coordinates
        # Fixed waypoint for neat stacking



        # big_waypoints = [
        #     WaypointRecording([
        #         [-0.398835  ,  0.27304858,  0.44945639, -1.73339832, -0.37275735],
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968],
        #         [-0.398835  ,  0.27304858,  0.44945639, -1.73339832, -0.37275735]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968],
        #         [-0.56757289,  0.23623304,  0.40650493, -1.69351482, -0.51848555],
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.6304661 ,  0.29299033,  0.37582532, -1.64749539, -0.56910688],
        #         [-0.61359233,  0.44332045,  0.10277671, -1.29928172, -0.57217485],
        #         [-0.6304661 ,  0.29299033,  0.37582532, -1.64749539, -0.56910688]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.74704868,  0.08130098,  0.10277671, -1.59840798, -0.7056312 ],
        #         [-0.72864091,  0.23623304, -0.21168935, -1.20724297, -0.69642729],
        #         [-0.74704868,  0.08130098,  0.10277671, -1.59840798, -0.7056312 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.86056322, -0.02607767, -0.20401946, -1.44040799, -0.8620972 ],
        #         [-0.86516517,  0.11965051, -0.4555923 , -1.04003906, -0.86516517],
        #         [-0.86056322, -0.02607767, -0.20401946, -1.44040799, -0.8620972 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-1.03390312, -0.19174761, -0.50007772, -1.26553416, -1.0262332 ],
        #         [-1.04157293, -0.06596117, -0.67341757, -0.95567006, -1.02163124],
        #         [-1.03390312, -0.19174761, -0.50007772, -1.26553416, -1.0262332 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-1.30695164, -0.33133987, -0.5767768 , -1.29007792, -1.27013612],
        #         [-1.30695164, -0.14112623, -0.74091274, -0.9909516 , -1.28087401],
        #         [-1.30695164, -0.33133987, -0.5767768 , -1.29007792, -1.27013612]
        #     ], [0,1,1])
        # ]

        # small_waypoints = [
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        # ]

        #intermediate_uv = [[515,485,973],[515,545,973],[515, 585,973],[455,485,973],[455,545,973],[455, 585,973],[395,485,973],[395,545,973],[395, 585,973],
        #                    [515,425,973],[515,365,973],[515, 305,973],[455,425,973],[455,365,973],[455, 305,973],[395,425,973],[395,365,973],[395, 305,973]]
        x_init = 540
        x = x_init
        y = 230
        z = 968
        depthFrame = self.camera.DepthFrameRaw

        intermediate_uv = []
        for i in range(6):
            for j in range(3):
                # Check surrounding depth to see if location is clear
                depthCheck = 1
                for q in range(48):
                    for k in range(48):
                        q_fac = q-24
                        k_fac = k-24
                        print("Checking x,y,depth: ")
                        print("\tx: " + str(x+q_fac) + "y: " + str(y+k_fac) + "d: " + str(depthFrame[y+k_fac, x+q_fac]))
                        if(depthFrame[y+k_fac, x+q_fac] < 963):
                            depthCheck = 0
                # If good location, append
                if(depthCheck == 1):
                    intermediate_uv.append([x,y,z])
                x -= 80
            x = x_init
            y += 80

        intermediate = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in intermediate_uv]

        self.camera.blockDetector()

        i = 0
        j = 0

        # Process big and small blocks, unstack them and place them on layer 1 away from destination point
        while len(self.camera.block_detections) > 0:
            for block in self.camera.block_detections:
                #  TODO
                # If in first layer and already in buffer, skip it
                # if(block.coord[0] > <~> and block.coord[1] < <~> and block.coord[2] < 42)
                w_coords = block.coord
                print(w_coords)
                self.pick(block.size, w_coords, block.theta, k_move=1.4, k_accel=(1.0/6.0))
                self.place(intermediate_uv[i], intermediate[i], 0, 1, block.size, height=150, k_move=1.4, k_accel=(1.0/6.0))
                i += 1

            self.rxarm.sleep()
            rospy.sleep(2)
            self.camera.blockDetector()
            rospy.sleep(1)

        self.camera.mask_list = []
        self.camera.blockDetector()

        big_blocks = []
        small_blocks = []

        # Separate Big from small blocks
        for block in self.camera.block_detections:
            if(block.size == "big"):
                big_blocks.append(block)
        for block in self.camera.block_detections:
            if(block.size == "small"):
                small_blocks.append(block)

        # Sort by color
        big_sorted = []
        small_sorted = []

        red_big = []
        orange_big = []
        yellow_big = []
        green_big = []
        blue_big = []
        violet_big = []

        red_small = []
        orange_small = []
        yellow_small = []
        green_small = []
        blue_small = []
        violet_small = []

        for block in big_blocks:
            if (block.color == "red"):
                red_big.append(block)
            elif (block.color == "orange"):
                orange_big.append(block)
            elif (block.color == "yellow"):
                yellow_big.append(block)
            elif (block.color == "green"):
                green_big.append(block)
            elif (block.color == "blue"):
                blue_big.append(block)
            elif (block.color == "violet"):
                violet_big.append(block)

        big_sorted += [red for red in red_big]
        big_sorted += [orange for orange in orange_big]
        big_sorted += [yellow for yellow in yellow_big]
        big_sorted += [green for green in green_big]
        big_sorted += [blue for blue in blue_big]
        big_sorted += [violet for violet in violet_big]


        for block in small_blocks:
            if (block.color == "red"):
                red_small.append(block)
            elif (block.color == "orange"):
                orange_small.append(block)
            elif (block.color == "yellow"):
                yellow_small.append(block)
            elif (block.color == "green"):
                green_small.append(block)
            elif (block.color == "blue"):
                blue_small.append(block)
            elif (block.color == "violet"):
                violet_small.append(block)

        small_sorted += [red for red in red_small]
        small_sorted += [orange for orange in orange_small]
        small_sorted += [yellow for yellow in yellow_small]
        small_sorted += [green for green in green_small]
        small_sorted += [blue for blue in blue_small]
        small_sorted += [violet for violet in violet_small]

        # LINNING UP Blocks
        u_big = 635
        v_big = 355
        d_big = 970
        step_big = 48

        u_small = 635
        v_small = 265
        d_small = 973
        step_small = 33

        print("Starting Block Placement!")
        print(big_sorted)
        print(small_sorted)

        i = 0
        for block in big_sorted:
            if((abs(block.theta) < 5) or ((abs(block.theta) > 85) and (abs(block.theta) < 95))):
                block.theta = 0
            self.pick(block.size, block.coord, block.theta, k_move=2.0, k_accel=(1.0/6.0))
            # self.waypoints = big_waypoints[i]
            # self.execute()
            # self.clear_waypoints()
            self.place([u_big, v_big, d_big], self.camera.u_v_d_to_world(u_big, v_big, d_big), 0, 1, block.size, height=150, k_move=2.0, k_accel=(1.0/6.0))
            u_big += step_big

        for block in small_sorted:
            if((abs(block.theta) > 5) or ((abs(block.theta) > 85) and (abs(block.theta) < 95))):
                block.theta = 0
            self.pick(block.size, block.coord, block.theta, k_move=2.0, k_accel=(1.0/6.0))
            self.place([u_small, v_small, d_small], self.camera.u_v_d_to_world(u_small, v_small, d_small), 0, 1, block.size, height=150, k_move=2.0, k_accel=(1.0/6.0))
            u_small += step_small

        self.rxarm.sleep()
        self.camera.mask_list = []
        return



    # Event 4
    def stack_high(self):
        # Image coordinates
        # Fixed waypoint for neat stacking



        # big_waypoints = [
        #     WaypointRecording([
        #         [-0.398835  ,  0.27304858,  0.44945639, -1.73339832, -0.37275735],
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968],
        #         [-0.398835  ,  0.27304858,  0.44945639, -1.73339832, -0.37275735]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968],
        #         [-0.56757289,  0.23623304,  0.40650493, -1.69351482, -0.51848555],
        #         [-0.53229135,  0.73937875,  0.57370883, -1.51710701, -0.50467968]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.6304661 ,  0.29299033,  0.37582532, -1.64749539, -0.56910688],
        #         [-0.61359233,  0.44332045,  0.10277671, -1.29928172, -0.57217485],
        #         [-0.6304661 ,  0.29299033,  0.37582532, -1.64749539, -0.56910688]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.74704868,  0.08130098,  0.10277671, -1.59840798, -0.7056312 ],
        #         [-0.72864091,  0.23623304, -0.21168935, -1.20724297, -0.69642729],
        #         [-0.74704868,  0.08130098,  0.10277671, -1.59840798, -0.7056312 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-0.86056322, -0.02607767, -0.20401946, -1.44040799, -0.8620972 ],
        #         [-0.86516517,  0.11965051, -0.4555923 , -1.04003906, -0.86516517],
        #         [-0.86056322, -0.02607767, -0.20401946, -1.44040799, -0.8620972 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-1.03390312, -0.19174761, -0.50007772, -1.26553416, -1.0262332 ],
        #         [-1.04157293, -0.06596117, -0.67341757, -0.95567006, -1.02163124],
        #         [-1.03390312, -0.19174761, -0.50007772, -1.26553416, -1.0262332 ]
        #     ], [0,1,1]),
        #     WaypointRecording([
        #         [-1.30695164, -0.33133987, -0.5767768 , -1.29007792, -1.27013612],
        #         [-1.30695164, -0.14112623, -0.74091274, -0.9909516 , -1.28087401],
        #         [-1.30695164, -0.33133987, -0.5767768 , -1.29007792, -1.27013612]
        #     ], [0,1,1])
        # ]

        # small_waypoints = [
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        #     [
        #         [],
        #         [],
        #         []
        #     ],
        # ]

        #intermediate_uv = [[515,485,973],[515,545,973],[515, 585,973],[455,485,973],[455,545,973],[455, 585,973],[395,485,973],[395,545,973],[395, 585,973],
        #                    [515,425,973],[515,365,973],[515, 305,973],[455,425,973],[455,365,973],[455, 305,973],[395,425,973],[395,365,973],[395, 305,973]]
        x_init = 540
        x = x_init
        y = 290
        z = 968
        depthFrame = self.camera.DepthFrameRaw

        intermediate_uv = []
        for i in range(6):
            for j in range(3):
                # Check surrounding depth to see if location is clear
                depthCheck = 1
                for q in range(48):
                    for k in range(48):
                        q_fac = q-24
                        k_fac = k-24
                        print("Checking x,y,depth: ")
                        print("\tx: " + str(x+q_fac) + "y: " + str(y+k_fac) + "d: " + str(depthFrame[y+k_fac, x+q_fac]))
                        if(depthFrame[y+k_fac, x+q_fac] < 963):
                            depthCheck = 0
                # If good location, append
                if(depthCheck == 1):
                    intermediate_uv.append([x,y,z])
                x -= 80
            x = x_init
            y += 80

        intermediate = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in intermediate_uv]

        self.camera.blockDetector()

        i = 0
        j = 0

        # Process big and small blocks, unstack them and place them on layer 1 away from destination point
        while len(self.camera.block_detections) > 0:
            for block in self.camera.block_detections:
                # TODO
                # If in first layer and already in buffer, skip it
                # if(block.coord[0] > <~> and block.coord[1] < <~> and block.coord[2] < 42)
                w_coords = block.coord
                print(w_coords)
                self.pick(block.size, w_coords, block.theta, k_move=1.4, k_accel=(1.0/6.0))
                self.place(intermediate_uv[i], intermediate[i], 0, 1, block.size, height=150, k_move=1.4, k_accel=(1.0/6.0))
                i += 1

            self.rxarm.sleep()
            rospy.sleep(2)
            self.camera.blockDetector()
            rospy.sleep(1)

        self.camera.mask_list = []
        self.camera.blockDetector()

        big_blocks = []
        small_blocks = []

        # Separate Big from small blocks
        for block in self.camera.block_detections:
            if(block.size == "big"):
                big_blocks.append(block)
        for block in self.camera.block_detections:
            if(block.size == "small"):
                small_blocks.append(block)

        # Sort by color
        big_sorted = []
        small_sorted = []

        red_big = []
        orange_big = []
        yellow_big = []
        green_big = []
        blue_big = []
        violet_big = []

        red_small = []
        orange_small = []
        yellow_small = []
        green_small = []
        blue_small = []
        violet_small = []

        for block in big_blocks:
            if (block.color == "red"):
                red_big.append(block)
            elif (block.color == "orange"):
                orange_big.append(block)
            elif (block.color == "yellow"):
                yellow_big.append(block)
            elif (block.color == "green"):
                green_big.append(block)
            elif (block.color == "blue"):
                blue_big.append(block)
            elif (block.color == "violet"):
                violet_big.append(block)

        big_sorted += [red for red in red_big]
        big_sorted += [orange for orange in orange_big]
        big_sorted += [yellow for yellow in yellow_big]
        big_sorted += [green for green in green_big]
        big_sorted += [blue for blue in blue_big]
        big_sorted += [violet for violet in violet_big]


        for block in small_blocks:
            if (block.color == "red"):
                red_small.append(block)
            elif (block.color == "orange"):
                orange_small.append(block)
            elif (block.color == "yellow"):
                yellow_small.append(block)
            elif (block.color == "green"):
                green_small.append(block)
            elif (block.color == "blue"):
                blue_small.append(block)
            elif (block.color == "violet"):
                violet_small.append(block)

        small_sorted += [red for red in red_small]
        small_sorted += [orange for orange in orange_small]
        small_sorted += [yellow for yellow in yellow_small]
        small_sorted += [green for green in green_small]
        small_sorted += [blue for blue in blue_small]
        small_sorted += [violet for violet in violet_small]

        # LINNING UP Blocks
        u_big = 780
        v_big = 410
        d_big = 965
        step_big = 40
        u_step_big = 4
        v_step_big = 2

        u_small = 775
        v_small = 265
        d_small = 968
        step_small = 24
        u_step_small = 3
        v_step_small = -2

        print("Starting Block Placement!")
        print(big_sorted)
        print(small_sorted)

        i = 0
        for block in big_sorted:
            if((abs(block.theta) <5) or ((abs(block.theta) > 85) and (abs(block.theta) < 95))):
                block.theta = 0

            self.pick(block.size, block.coord, block.theta, k_move=2.0, k_accel=(1.0/6.0))
            # self.waypoints = big_waypoints[i]
            # self.execute()
            # self.clear_waypoints()
            self.place([u_big, v_big, d_big], self.camera.u_v_d_to_world(u_big, v_big, d_big), 0, 1, block.size, height=150, k_move=2.0, k_accel=(1.0/6.0))
            d_big -= step_big
            u_big += u_step_big
            v_big += v_step_big

        for block in small_sorted:
            self.pick(block.size, block.coord, block.theta, k_move=2.0, k_accel=(1.0/6.0))
            self.place([u_small, v_small, d_small], self.camera.u_v_d_to_world(u_small, v_small, d_small), 0, 1, block.size, height=150, k_move=1.4, k_accel=(1.0/6.0))
            d_small -= step_small
            u_small += u_step_small
            v_small += v_step_small

        self.rxarm.sleep()
        self.camera.mask_list = []

    # Event 5 helper
    def tnr_place(self):
        # reads from stored joint angles to place large block at specified index
        pass

    # Event 5
    def to_sky(self):
            # self.waypoints = big_waypoints[i]
            # self.execute()
            # self.clear_waypoints()
        '''
        x_init = 525
        x = x_init
        y = 315
        z = 968
        depthFrame = self.camera.DepthFrameRaw

        intermediate_uv = []
        for i in range(4):
            for j in range(3):
                # Check surrounding depth to see if location is clear
                depthCheck = 1
                for q in range(48):
                    for k in range(48):
                        q_fac = q-24
                        k_fac = k-24
                        print("Checking x,y,depth: ")
                        print("\tx: " + str(x+q_fac) + "y: " + str(y+k_fac) + "d: " + str(depthFrame[y+k_fac, x+q_fac]))
                        if(depthFrame[y+k_fac, x+q_fac] < 963):
                            depthCheck = 0
                # If good location, append
                if(depthCheck == 1):
                    intermediate_uv.append([x,y,z])
                x -= 80
            x = x_init
            y += 80


        x_init = 930
        x = x_init
        y = 315
        z = 968
        depthFrame = self.camera.DepthFrameRaw

        # intermediate_uv = []
        for i in range(4):
            for j in range(3):
                # Check surrounding depth to see if location is clear
                depthCheck = 1
                for q in range(48):
                    for k in range(48):
                        q_fac = q-24
                        k_fac = k-24
                        print("Checking x,y,depth: ")
                        print("\tx: " + str(x+q_fac) + "y: " + str(y+k_fac) + "d: " + str(depthFrame[y+k_fac, x+q_fac]))
                        if(depthFrame[y+k_fac, x+q_fac] < 963):
                            depthCheck = 0
                # If good location, append
                if(depthCheck == 1):
                    intermediate_uv.append([x,y,z])
                x -= 80
            x = x_init
            y += 80

        intermediate = [list(self.camera.u_v_d_to_world(dest[0], dest[1], dest[2])) for dest in intermediate_uv]


        self.camera.blockDetector()

        i = 0
        j = 0

        # Process big and small blocks, unstack them and place them on layer 1 away from destination point
        while len(self.camera.block_detections) > 0:
            for block in self.camera.block_detections:
                # TODO
                # If in first layer and already in buffer, skip it
                # if(block.coord[0] > <~> and block.coord[1] < <~> and block.coord[2] < 42)
                w_coords = block.coord
                print(w_coords)
                self.pick(block.size, w_coords, block.theta, k_move=0.8, k_accel=(1.0))
                self.place(intermediate_uv[i], intermediate[i], 0, 1, block.size, height=150, k_move=0.6, k_accel=(1.0/4.0))
                i += 1

            self.rxarm.sleep()
            rospy.sleep(2)
            self.camera.blockDetector()
            rospy.sleep(1)


        '''
        self.camera.mask_list = []
        self.camera.blockDetector()

        print("done sorting ~~~~~~~")

        # Begin the stacking

        # LINNING UP Blocks
        u_big = 650
        v_big = 300
        d_big = 972
        step_big = 38
        u_step_big = 4
        v_step_big = -8

        # Naively stack blocks a la event 4
        i = 0
        for h, block in enumerate(self.camera.block_detections):
            # self.pick(block.size, block.coord, block.theta, k_move=2.0, k_accel=(1.0/6.0))
            sleep_angle = -0.8 if h < 8 else -1.2
            self.pick(block.size, block.coord, block.theta)
            self.waypoints.arm_coords = [[0.0, sleep_angle, 0.0, 0.0, 0.0]]
            self.waypoints.gripper_state = [0]
            self.execute(k_move=0.7, k_accel=3.0, min_move_time=1.0)
            if h < 6:
                self.place([u_big, v_big, d_big], self.camera.u_v_d_to_world(u_big, v_big, d_big), 0, 1, block.size, height=70, k_move=2.0, k_accel=(1.0/6.0), phi_i=174)
            else:
                self.place([u_big, v_big, d_big], self.camera.u_v_d_to_world(u_big, v_big, d_big), 0, 1, block.size, height=70, k_move=2.0, k_accel=(1.0/6.0), phi_i=90, not_rot=True)
            self.waypoints.arm_coords = [[0.0, sleep_angle, 0.0, 0.0, 0.0]]
            self.waypoints.gripper_state = [1]
            self.execute(k_move=0.5, k_accel=3.0, min_move_time=0.8)

            d_big -= step_big
            u_big += u_step_big
            v_big += v_step_big

            if h >= 6:
                v_big += v_step_big

        # self.rxarm.sleep()

        self.waypoints.arm_coords = [[0.0, -103.0 * D2R, -89.0 * D2R, -45.0 * D2R, 0.0]]
        self.waypoints.gripper_state = [1]
        self.execute(k_move=1.5, k_accel=3.0, min_move_time=0.8)
        self.camera.mask_list = []
        print("done working")


class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)

    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)
