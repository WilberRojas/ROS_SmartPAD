#!/usr/bin/env python3

""" from __future__ import print_function
from six.moves import input """

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from math import tau, dist, fabs, cos

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterface(object):    

    def __init__(self, group):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_kr6", anonymous=True)
        robot = moveit_commander.RobotCommander()        
        scene = moveit_commander.PlanningSceneInterface()
       
        group_name = group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)       
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )        
       
        planning_frame = self.move_group.get_planning_frame()       
        eef_link = self.move_group.get_end_effector_link()        
        group_names = robot.get_group_names()       
        current_state = robot.get_current_state()    

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = self.move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.scale_ROT = tau/16
        self.scale_LIN = 0.1

        N_points = 100
        self.Joint_points = [None] * N_points
        self.Cartesian_points = [None] * N_points
        self.mov_type = [None] *N_points
        
        #self.add_table_fun()
        self.joints_pos1 = [0.4599, -0.5522, 1.9368, -0.0001, 0.1855, 0.4601]
        self.joints_pos2 = [-0.4499, -0.5549, 1.9478, -0.0, 0.177, -0.45]
        self.joints_pos3 = [0.3, -0.2421, 1.0413, -0.0, 0.7808, 0.3]
        self.joints_pos4 = [-0.2899, -0.2391, 1.0336, -0.0, 0.7655, -0.29]
        self.joints_pos5 = [0.0094, -0.4869, 1.6946, -0.0094, 0.3526, 0.018]


    def get_position(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values() #joints
        wpose = move_group.get_current_pose().pose #cartesian
        joint_list2 = []
        for index, data in enumerate(joint_goal):
            #print("joint_goal["+str(index)+"] = " + str(round(data,2)))  
            joint_list2.append(round(data,4))
        print("in list:", joint_list2)
        print("X:", wpose.position.x, "  Y:", wpose.position.y)

    def go_to_pos(self, pos):
        move_group = self.move_group       
        joint_goal = move_group.get_current_joint_values()
        joins_pos = []
        if pos == 1: joins_pos = self.joints_pos1
        elif pos == 2: joins_pos = self.joints_pos2
        elif pos == 3: joins_pos = self.joints_pos3
        elif pos == 4: joins_pos = self.joints_pos4
        elif pos == 5: joins_pos = self.joints_pos5

        joint_goal[0] = joins_pos[0]
        joint_goal[1] = joins_pos[1]
        joint_goal[2] = joins_pos[2]
        joint_goal[3] = joins_pos[3]
        joint_goal[4] = joins_pos[4]
        joint_goal[5] = joins_pos[5]

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_draw(self):
        move_group = self.move_group       
        joint_goal = move_group.get_current_joint_values()        

        joint_goal[0] = 0
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = 0
        joint_goal[4] = tau/4
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_home(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = 0
        joint_goal[4] = 0 #tau/4
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def joints_move(self, joint, direction):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        scale_ROT = self.scale_ROT

        if joint == "j1":
            if direction == "forward":
                joint_goal[0] += scale_ROT 
            elif direction == "reverse":
                joint_goal[0] -= scale_ROT
        elif joint == "j2":
            if direction == "forward":
                joint_goal[1] += scale_ROT 
            elif direction == "reverse":
                joint_goal[1] -= scale_ROT
        elif joint == "j3":
            if direction == "forward":
                joint_goal[2] += scale_ROT
            elif direction == "reverse":
                joint_goal[2] -= scale_ROT
        elif joint == "j4":
            if direction == "forward":
                joint_goal[3] += scale_ROT 
            elif direction == "reverse":
                joint_goal[3] -= scale_ROT
        elif joint == "j5":
            if direction == "forward":
                joint_goal[4] += scale_ROT 
            elif direction == "reverse":
                joint_goal[4] -= scale_ROT
        elif joint == "j6":
            if direction == "forward":
                joint_goal[5] += scale_ROT 
            elif direction == "reverse":
                joint_goal[5] -= scale_ROT

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
   
    def cartesian_pos(self, axis, direction):
        move_group = self.move_group
        waypoints = []
        scale_LIN = self.scale_LIN
        scale_ROT = self.scale_ROT
        wpose = move_group.get_current_pose().pose

        if axis == "x":
            if direction == "forward":
                wpose.position.x += scale_LIN
            elif direction == "reverse":
                wpose.position.x -= scale_LIN
        elif axis == "y":
            if direction == "forward":
                wpose.position.y += scale_LIN
            elif direction == "reverse":
                wpose.position.y -= scale_LIN
        elif axis == "z":
            if direction == "forward":
                wpose.position.z += scale_LIN
            elif direction == "reverse":
                wpose.position.z -= scale_LIN

        elif axis == "R":
            print("roll")
            if direction == "forward":
                wpose.orientation.x += scale_ROT
            elif direction == "reverse":
                wpose.orientation.x -= scale_ROT
        elif axis == "P":
            print("pitch")
            if direction == "forward":
                wpose.orientation.y += scale_ROT
            elif direction == "reverse":
                wpose.orientation.y -= scale_ROT
        elif axis == "Y":
            print("yaw")
            if direction == "forward":
                wpose.orientation.z += scale_ROT
            elif direction == "reverse":
                wpose.orientation.z -= scale_ROT


        waypoints.append(copy.deepcopy(wpose))  
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

        return plan, fraction

    def joints_move_sequence(self, n):
        move_group = self.move_group       
        joint_goal = self.Joint_points[n][0]
        move_group.go(joint_goal, wait=True)        
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def cartesian_sequence(self, n):
        move_group = self.move_group
        waypoints = []
        wpose = self.Cartesian_points[n][0]
        waypoints.append(copy.deepcopy(wpose))  
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

        return plan, fraction
    
    def change_vel(self, operation, percentage):
        if operation == "upscale":
            self.scale_LIN = self.scale_LIN*2
            self.scale_ROT = self.scale_ROT*2
            print(percentage)
            self.move_group.set_max_velocity_scaling_factor(percentage/100)

        elif operation == "downscale":
            self.scale_LIN = self.scale_LIN/2
            self.scale_ROT = self.scale_ROT/2
            print(percentage)
            self.move_group.set_max_velocity_scaling_factor(percentage/100)

    def add_table_fun(self):
        scene = self.scene
        self.table_dimensions = [1.005, 1.505, 0.05]
        self.table_position = [0.22, 0.0, 0.26]

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = self.table_position[0] + self.table_dimensions[0]/2 # For centering
        box_pose.pose.position.y = self.table_position[1]
        box_pose.pose.position.z = self.table_position[2]
        self.box_name = "box"
        scene.add_box(self.box_name, box_pose, size=(self.table_dimensions[0], self.table_dimensions[1], self.table_dimensions[2]))
          
    def remove_table_fun(self):        
        box_name = self.box_name
        scene = self.scene        
        scene.remove_world_object(box_name)        
        
    #END CLASS

