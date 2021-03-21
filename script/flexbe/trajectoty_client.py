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
# import pandas as pd

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
        rospy.init_node('trajectory_client')
        moveit_commander.roscpp_initialize(sys.argv)
 
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("james_arm_v2") # xarm6
        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher

        
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
    # trajectory = []


if __name__=="__main__":
    rate = rospy.Rate(20.0)
    rospy.set_param("trajectory_status",0)
    try:
        client = MoveClient()
        joint_list = []
        line_count =0
        while 1:
            with open('/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/test.csv') as traj:
                while 1:
                    data = traj.readline().replace("\n","") # 줄 바꿈 제거 
                    # print (data)
                    if not data: break
                    if line_count == 0 :
                        header = data.split(",")
                    else : 
                        joint_list.append(data.split(","))
                    line_count += 1

        while not rospy.is_shutdown():
            if rospy.get_param("trajectory_status") == 1:
                for joint in joint_list:
                    clint.move_joint(joint[0],joint[1],joint[2],joint[3],joint[4],joint[5])
            else:
                pass
        # print (joint_list[0][5])  
          
            
                
    except rospy.ROSInterruptException:
        pass