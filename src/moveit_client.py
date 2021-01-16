#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
from math import sin, cos, pi

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from moveit_commander.conversions import pose_to_list
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
def preset_joint():
        # joint goal
        if 3 < rospy.get_param("button") < 10: # B1 ~ 3 
            return 0.000, 0.121, 1.959, 0.000, -0.660, 0.000
        elif 11 < rospy.get_param("button") < 18: # 4 ~ 6 
            print("4~6")
            return -1.055, 0.293, 1.124, -1.525, -0.979, 1.614
            return 1.364, -1.505, 1.319, -1.358, 1.509, -0.241

        else:
            return -0.054, -1.537, 1.086, 0.000, 2.000, 0.000 

class simple_move():
    def __init__(self):
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

    def move_joint(self):
        group = self.group
        # joint goal
        group.clear_pose_targets()
        joint_goal = group.get_current_joint_values()
        joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3], joint_goal[4], joint_goal[5] = set_joint()
        rospy.logwarn("move joint")
        group.go(joint_goal, wait=True)
        rospy.sleep(10)
        
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

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
        pose_target.orientation.z = ori_z  
        pose_target.orientation.w = ori_w 

        group.set_pose_target(pose_target)
        rospy.logwarn("move to pose")
        plan = group.go(wait=True)
        rospy.sleep(10)

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
        rospy.logwarn("move to pose")
        plan = group.go(wait=True)
        rospy.sleep(10)

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)

    def move_home(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        
        joint_goal[0] = 0.000
        joint_goal[1] = -1.736
        joint_goal[2] = 1.181
        joint_goal[3] = 0.000
        joint_goal[4] = 2.079
        joint_goal[5] = 0.000
        # rospy.logwarn("move to Home")
        group.go(joint_goal, wait=True)
        rospy.sleep(7)

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    def move_camera_pose(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        
        joint_goal[0] = 0.000
        joint_goal[1] = -0.5794
        joint_goal[2] = -0.3334
        joint_goal[3] = 0.000
        joint_goal[4] = 0.9424
        joint_goal[5] = 0.000
        # rospy.logwarn("move to Home")
        group.go(joint_goal, wait=True)
        rospy.sleep(7)

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def get_end_effector_pose(self):
        group = self.group
        end_effector_pose = group.get_current_pose("link_eef")
        print(end_effector_pose)

    def plan_cartesian_path(self, scale=1):
        # group.get_current_pose().posedirectly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
   

def main(): # 1. 타겟 20cm 앞에서 정지 2. 20cm 앞으로 이동
    Xarm6 = simple_move()
    Xarm6.move_camera_pose() # 카메라로 보고
    target_listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    
    while not rospy.is_shutdown():
        rospy.logwarn("Start Xarm Control!")
        try:
            (trans,rot) = target_listener.lookupTransform('/world','/object_13', rospy.Time(0)) # object
            print (trans[2])
            rospy.logwarn("Move to Z+0.20!")
            # if trans[2]+0.11 < 0.12:
            #     trans[2] = 0.12

            if Xarm6.move_xyz(trans[0]-0.005,trans[1]+0.022,trans[2]+0.12) == True :
                rospy.logwarn("Now Move to Target!")    
                break
                # if james_ear.push_action(trans[0],trans[1],trans[2]) == True :
                # else:
                #     pass
            else:
                pass     

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Look up Failed")
         
        rate.sleep()
    moveit_commander.roscpp_shutdown()



if __name__=="__main__":    
    try:
        main()      
    except rospy.ROSInterruptException:
        pass