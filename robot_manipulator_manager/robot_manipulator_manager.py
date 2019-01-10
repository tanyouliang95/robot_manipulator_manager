#!/usr/bin/env python3

"""
=========================================================
Creator: Tan You Liang
Date: Jan 2019
Description:  Robot Manipulator Manager, Working
              Edit `eef_pose.yaml` for define of eef pose
Gripper:      Open: 0, close: 1

*Note: if ros2 gripper is enable, eef command will be pub after arm motion is fully done
==========================================================
"""

import rclpy
from rclpy.node import Node
from termcolor import colored
import os
import yaml

from atm_msgs.msg import ArmManipulatorCommand
from atm_msgs.msg import ArmManipulatorStatus

from robot_manipulator_manager.msg import ManipulatorState

from std_msgs.msg import String
from std_msgs.msg import Int8


class RobotManipulatorManager(Node):
    def __init__(self, config_path):
        # start node & pub sub
        super().__init__('robot_manipulator_manager')  # run rclpy obj 'Node'
        self.atm_pub = self.create_publisher(ArmManipulatorStatus, '/arm_manipulator/status')
        self.atm_sub = self.create_subscription(ArmManipulatorCommand, '/arm_manipulator/command', self.atm_command_callback)
        self.timer = self.create_timer(1.0, self.atm_status_update_callback)

        self.control_pub = self.create_publisher(String, '/ur10/motion_group_id')
        self.control_sub = self.create_subscription(ManipulatorState, '/ur10/manipulator_state', self.manipulator_state_callback)


        self.current_arm_pose_id = 'Nan'
        self.request_arm_pose_id = 'Nan'
        self.current_eef_state = -1
        self.request_eef_state = -1
        self.request_motion_group_id = 'Nan'
        self.current_motion_group_id = 'Nan'
        self.eef_pose = [0,0,0,0,0,0] # xyz and rpy
        self.motion_progress = 0
        self.arm_newly_completed = False

        # YAML CONFIG OBJ
        self.yaml_obj = []
        self.load_pose_config(path=config_path)
        if (self.enable_ros2_gripper == True):
            # ros2 dynamixel gripper if enable, PENDING.........
            
            from dynamixel_gripper_ros2_msgs.msg import GripState # ros2 gripper
            self.gripper_pub = self.create_publisher(String, '/gripper/command')  
            self.gripper_sub = self.create_subscription(GripState, '/gripper/grip_state', self.gripper_state_callback) 

        print(colored(" Successfully being initialized!!", "white", 'on_green'))

            

    # Read Yaml file
    def load_pose_config(self, path):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        full_path = dir_path + "/" + path
        with open(full_path, 'r') as stream:
            try:
                self.yaml_obj = yaml.load(stream)
            except yaml.YAMLError as exc:
                print("Error in loading Yaml" + exc)
                exit(0)

        self.enable_ros2_gripper =  self.yaml_obj['enable_ros2_gripper']    # bool
        self.pos_tolerance =  self.yaml_obj['pos_tolerance']              # float
        self.rot_tolerance =  self.yaml_obj['rot_tolerance']              # float
        self.manipulator_id =  self.yaml_obj['arm_id']    


    # **Manipulator state from ros1 2 bridge
    def manipulator_state_callback(self, msg):
        self.motion_progress =  msg.arm_motion_progress
        self.eef_pose = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw]

        # check if motion is completed
        if ( msg.arm_motion_progress == 1.0):
            # check whether eef reaches pose
            if (self.check_eef_pose()):
                self.current_arm_pose_id = self.request_arm_pose_id
                self.arm_newly_completed = True
            else:
                self.current_arm_pose_id = 'error'
        else:
            self.current_arm_pose_id = 'running'
        
        # publish ros2 gripper request after the arm motion
        if (self.enable_ros2_gripper == True):
            if (self.arm_newly_completed):
                msg = Int8()
                msg.data = self.request_eef_state
                self.gripper_pub.publish(msg)
                self.arm_newly_completed = False
        else:  
            # use ros1 gripper by update the class state
            self.current_eef_state = msg.gripper_state


    # **Timer callback to update ATM on manipulator state
    def atm_status_update_callback(self):
        msg = ArmManipulatorStatus()
        msg.manipulator_id = self.manipulator_id
        msg.task_id = 'Nan' # TBC
        msg.arm_pose_id = self.current_arm_pose_id
        msg.eef_state = self.current_eef_state
        self.atm_pub.publish(msg)


    # **ATM command Callback
    def atm_command_callback(self, msg):
        if (self.manipulator_id == msg.manipulator_id):
            
            self.request_arm_pose_id = msg.arm_pose_id
            self.request_eef_state = msg.eef_command

            # Map arm_pos and eef_state to motion_id
            if (self.request_arm_pose_id == 'P1' and self.current_arm_pos == 'P0'):
                self.request_motion_group_id = 'G2'
            if (self.request_arm_pos == 'P2' and self.current_arm_pos == 'P1'):
                self.request_motion_group_id = 'G3'
            # TODO: Add MORE GROUPS!!!
            else:
                print(colored(" Invalid command!!!!", "red"))

            group_id_msg = String()
            group_id_msg.data = self.request_motion_group_id
            self.control_pub.publish(group_id_msg)


    # Check eef pose according to yaml, 
    # @return: True if all within tolerance
    def check_eef_pose(self):
        # check xyz position tolerance
        for i in range(0,3):
            if (abs(self.eef_pose[i] - self.yaml_obj['pose_id'][self.request_arm_pose_id])  < self.pos_tolerance):
                return False
        # check rpy rotation tolerance
        for i in range(3,6):
            if (abs(self.eef_pose[i] - self.yaml_obj['pose_id'][self.request_arm_pose_id])  < self.rot_tolerance):
                return False
        return True


    # TODO: or change to wait completion, maybe will use
    def check_completion(self, motion_group_id):
        if (motion_group_id == self.current_motion_group_id):
            if (self.motion_progress == 1.0):
                return True
        return False


    # Init placement
    def init_robot_pose(self):
        group_id_msg = String()
        group_id_msg.data = 'G1'
        self.control_pub.publish(group_id_msg)


    # *Gripper Callback, TBC
    def gripper_state_callback(self, msg):
        self.current_eef_state = msg.gripper_state
        

############################################################################################
############################################################################################


if __name__ == '__main__':
    rclpy.init(args=None)
    ur10_robot = RobotManipulatorManager(config_path="../config/eef_pose.yaml")
    rclpy.spin(ur10_robot)
