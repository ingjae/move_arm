#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import rospy
from std_msgs.msg import String,Bool
import math
from math import sin, cos, pi
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)
  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True

## move client 
class MoveClient(object):
    def __init__(self):
        # xarm
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_client')
        target_listener = tf.TransformListener()
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("xarm6") # xarm6
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        pose_target = geometry_msgs.msg.Pose()
        rospy.set_param("execute",-1)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher

        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0
        self.OFFSET_Z = 0.0
        
        # client
        self.tcp_sub = rospy.Subscriber('tcptopic', String, self.tcpCallback)
        self.tcp_msg = String()
        self.finish_pub = rospy.Publisher('is_finished',Bool,queue_size=10)
        self.rate = rospy.Rate(1.0)
        self.once = False
        self.is_finished = False
    
    # xarm
    def move_pose(self,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w):  # move
        group = self.group
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        plan1 = group.plan()
        pose_target.position.x = pos_x
        pose_target.position.y = pos_y
        pose_target.position.z = pos_z
        pose_target.orientation.x = ori_x  
        pose_target.orientation.y = ori_y 
        pose_target.orientation.w = ori_w 
        pose_target.orientation.z = ori_z  
        if (pos_z > 0.1):
            group.set_pose_target(pose_target)
            plan = group.go(wait=True)
        else:
            pass
            # print("wrong pose")
        rospy.sleep(0.5)
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)

    def move_xyz(self,pos_x,pos_y,pos_z):  # move
        group = self.group
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        plan1 = group.plan()
        pose_target.position.x = pos_x  
        pose_target.position.y = pos_y
        pose_target.position.z = pos_z
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0
        group.set_pose_target(pose_target)
        plan = group.go(wait=True)
        rospy.sleep(5)
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)
    
    # first movement : Coming out of the robot.
    def move_initial_pose_a(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = -1.8116518      # -103.8
        joint_goal[1] = 0.715585        # 41
        joint_goal[2] = -0.1797689      # -10.3
        joint_goal[3] = 0.000           # 0
        joint_goal[4] = -0.5742133      # -32.9
        joint_goal[5] = 0.0698132       # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_initial_pose_b(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = -1.85005        # -106
        joint_goal[1] = 0.4852015       # 27.8
        joint_goal[2] = -0.13439        # -7.7
        joint_goal[3] = -0.010472       # -0.6
        joint_goal[4] = -0.3228859      # -18.5
        joint_goal[5] = -0.13439        # -7.7
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_initial_pose_c(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = -1.682497       # -96.4
        joint_goal[1] = -0.0418879      # -2.4
        joint_goal[2] = 0.101229        # 5.8
        joint_goal[3] = -0.146608       # -8.4
        joint_goal[4] = -0.0663225      # -3.8
        joint_goal[5] = 0.0698132       # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(3)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_camera_pose(self,degree):
        if degree == 0: # 0(360) degree
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = -1.81514       # -104
            joint_goal[1] = -0.5794
            joint_goal[2] = -0.3334
            joint_goal[3] = 0.000
            joint_goal[4] = 0.9424
            joint_goal[5] = 0.000
            group.go(joint_goal, wait=True)
            rospy.sleep(2)  
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 90:  # 90 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = -0.366519      # -21
            joint_goal[1] = -0.5794
            joint_goal[2] = -0.3334
            joint_goal[3] = 0.000
            joint_goal[4] = 0.9424
            joint_goal[5] = 0.000
            group.go(joint_goal, wait=True)
            rospy.sleep(2)     
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 180:   # 180 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = 1.309          # 75
            joint_goal[1] = -0.5794
            joint_goal[2] = -0.3334
            joint_goal[3] = 0.000
            joint_goal[4] = 0.9424
            joint_goal[5] = 0.000
            group.go(joint_goal, wait=True)
            rospy.sleep(2)            
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 270:  # 270 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = 2.79253        # 160
            joint_goal[1] = -0.5794
            joint_goal[2] = -0.3334
            joint_goal[3] = 0.000
            joint_goal[4] = 0.9424
            joint_goal[5] = 0.000
            group.go(joint_goal, wait=True)
            rospy.sleep(3)            
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)
        else:
            pass
            # print("you select the wrong pose")

# marker(fidicial static_tansform) lookup 
def lookup_trans_tar():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (tar_trans,tar_rot) = target_listener.lookupTransform('/world','/marker', rospy.Time(0))
            rospy.logwarn("marker checkout")
            return (tar_trans, tar_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

# eef lookup
def lookup_trans_eef():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (eef_trans,eef_rot) = target_listener.lookupTransform('/world','/link_eef', rospy.Time(0))
            rospy.logwarn("link_eef checkout")

            return (eef_trans, eef_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

# qurternion transform (only 'yaw' value)
def quternion_rotation(tar1,tar2,tar3,tar4):
    # quaternion transform
    q_eul = euler_from_quaternion([tar1,tar2,tar3,tar4])
    q_eul = [3.14,0,q_eul[2]]
    # euler transform    
    new_rot = quaternion_from_euler(q_eul[0],q_eul[1],q_eul[2])    
    return new_rot

def charging_up_pose(tar1,tar2,tar3,rot1,rot2,rot3,rot4):
    client = MoveClient()
    client.move_pose(tar1,tar2,tar3+0.18,rot1,rot2,rot3,rot4)
    rospy.sleep(0.5)
    
def charging_down_pose():
    client = MoveClient()
    # repeat 2
    for i in range(2):
        print("%d iter start" %(i+1))
        # lookup 1
        (tar_trans, tar_rot) = lookup_trans_tar()
        (eef_trans, eef_rot) = lookup_trans_eef()
        new_rot = quternion_rotation(tar_rot[0],tar_rot[1],tar_rot[2],tar_rot[3])
        print("%d rotate pose" %(i+1))
        client.move_pose(eef_trans[0],eef_trans[1],eef_trans[2],new_rot[0],new_rot[1],new_rot[2],new_rot[3])
        # lookup 2
        (tar_trans, tar_rot) = lookup_trans_tar()

        print("%d center pose" %(i+1))
        client.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.25,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    
    # sponge down
    client.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.18,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    client.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.114,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    
    rospy.logwarn("go to charge")

    rospy.sleep(1.5)
    return(tar_trans[0],tar_trans[1],tar_trans[2],new_rot[0],new_rot[1],new_rot[2],new_rot[3])

if __name__=="__main__":    
    try:
        rospy.init_node('move_client')
        rate = rospy.Rate(10)
        client = MoveClient()
        while not rospy.is_shutdown():
            # print(tcp.tcp_msg)
            if client.tcp_msg.data == "00000001":
                rospy.logwarn("Initial_setting_a")

                client.checkPublisher()
            elif client.tcp_msg.data == "00000002":
                rospy.logwarn("Initial_setting_b")

                client.checkPublisher()
            elif client.tcp_msg.data == "00000003":
                rospy.logwarn("Initial_setting_c")

                client.checkPublisher()
            elif client.tcp_msg.data == "00000004":
                rospy.logwarn("Initial_pose")

                client.move_camera_pose()
                client.checkPublisher()
            elif client.tcp_msg.data == "00000005":
                rospy.logwarn("Charging pose")

                client.checkPublisher()
            elif client.tcp_msg.data == "00000006":
                rospy.logwarn("UnCharging pose")

                client.checkPublisher()
            elif client.tcp_msg.data == "":
                pass
            else:
                rospy.logwarn("Error")
            client.tcp_msg.data ="" 
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass