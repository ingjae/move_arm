#!/usr/bin/env python
#-*- coding:utf-8 -*-

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
# from gb_detection_3d_msgs.msg import BoundingBoxes3d
# from gb_detection_3d_msgs.msg import BoundingBox3d
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from gb_visual_detection_3d_msgs.msg import BoundingBox3d

global go_state

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
        
        # rospy.init_node('move_client')
        target_listener = tf.TransformListener()

        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("james_arm_v2") # xarm6
        
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
        # self.tcp_sub = rospy.Subscriber('tcptopic', String, self.tcpCallback)
        # self.tcp_msg = String()
        # self.finish_pub = rospy.Publisher('is_finished',Bool,queue_size=10)
        self.rate = rospy.Rate(1.0)
        # self.once = False
        # self.is_finished = False
        bounding_boxes_3d_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, cb_bounding_boxes_3d)

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
            print("wrong pose")
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
        pose_target.orientation.x = 0.707
        pose_target.orientation.y =  0.006
        pose_target.orientation.z = -0.006
        pose_target.orientation.w =   0.707
        group.set_pose_target(pose_target)
        plan = group.go(wait=True)
        # rospy.sleep(5)

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)
    def move_joint(self,joint1,joint2,joint3,joint4,joint5,joint6):
        group = self.group
        joint_goal = group.get_current_joint_values()
        
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6
        group.go(joint_goal, wait=True)
        rospy.sleep(5)
        
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)  
    
# marker(fidicial static_tansform) lookup 
def lookup_trans_tar():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (tar_trans,tar_rot) = target_listener.lookupTransform('/base_link','/target_link', rospy.Time(0))
            print("button checkout")
            return (tar_trans, tar_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def cb_bounding_boxes_3d(data):
    global go_state
    box_data = data
    boxes_count = len(box_data.bounding_boxes)
    go_state = 0
    for count in range (boxes_count):
        temp_class = box_data.bounding_boxes[count].Class
        button_state = rospy.get_param('button')
        # print(temp_class)
        if (temp_class == button_state):
            # print(temp_class,button_state)
            go_state = 1 # agree to go target
    # print(go_state)


if __name__=="__main__":
    rospy.init_node('push_button_client')
    rate = rospy.Rate(20.0)
    rospy.set_param("status",-1)
    OFFSET_X = -0.13 
    OFFSET_Y = +0.00
    OFFSET_Z = +0.00
    # bounding_boxes_3d_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, cb_bounding_boxes_3d)

    try:
        client = MoveClient()
        while not rospy.is_shutdown():
            global go_state

            if (rospy.get_param('status')== -1): # 명령 대기 //아무것도 하지 않음
                pass
            
            elif (rospy.get_param('status')== "check"):
                rospy.logwarn(go_state)

            elif (rospy.get_param('status')== 1): # 수행 완료 후 홈포즈 : (누르기 준비 자세)
                rospy.logwarn("button pushed!")
                rospy.logwarn("waiting for command, home pose")
                client.move_joint(0,0,0,0,0,0) # 홈포즈     
                rospy.set_param("status",-1) 

            elif (rospy.get_param('status')== 2): # 수행 실패
                rospy.logwarn("button push fail")
                rospy.logwarn("waiting for command, home pose")
                rospy.set_param("status",-1)

            elif (rospy.get_param('status')== 0 and go_state == 1): # 6 floor
                # client.move_joint(0,0,0,0,0,0) # 홈포즈 
                rospy.logwarn("Press button")
                print("executing command")
                (tar_trans, tar_rot)= lookup_trans_tar()        
                tar_trans[0]=tar_trans[0] - 0.12403
                print("move")
                client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                print("push")
                # tar_trans[1]=tar_trans[1] - 0.02
                # client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                # tar_trans[1]=tar_trans[1] + 0.02
                # client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                rospy.set_param("status",-1)
            
            elif (rospy.get_param('status')== 4):
                j=14
                for i in range(7):
                    rospy.set_param('button', j) # 5 floor
                    (tar_trans, tar_rot)= lookup_trans_tar()    
                    tar_trans[1]=tar_trans[1] + 0.05 
                    print("move")
                    client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                    print("push")
                    tar_trans[1]=tar_trans[1] - 0.02
                    client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                    tar_trans[1]=tar_trans[1] + 0.02
                    client.move_xyz(tar_trans[0],tar_trans[1], tar_trans[2])
                    client.move_pose(-0.024, -0.215, 0.432, 0.043, 0.709, 0.027, 0.703) # 홈포즈     
                    rospy.sleep(0.5)
                    j=j-2

                rospy.set_param("status",-1)
            else:
                rospy.logwarn("malfunction, check the TF")
                rospy.set_param("status",-1)
            ##############################################################################################

            
        rate.sleep            
          
            
                
    except rospy.ROSInterruptException:
        pass