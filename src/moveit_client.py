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
    def move_initial_pose(self):
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
        
        joint_goal[0] = -1.85005        # -106
        joint_goal[1] = 0.4852015       # 27.8
        joint_goal[2] = -0.13439        # -7.7
        joint_goal[3] = -0.010472       # -0.6
        joint_goal[4] = -0.3228859      # -18.5
        joint_goal[5] = -0.13439        # -7.7
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)

        joint_goal[0] = -1.682497       # -96.4
        joint_goal[1] = -0.0418879      # -2.4
        joint_goal[2] = 0.101229        # 5.8
        joint_goal[3] = -0.146608       # -8.4
        joint_goal[4] = -0.0663225      # -3.8
        joint_goal[5] = 0.0698132       # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)

        joint_goal[0] = -0.820305      # -47
        joint_goal[1] = -0.383972      # -22
        joint_goal[2] = -0.471239      # -27
        joint_goal[3] = -0.0174533     # -1
        joint_goal[4] = 0.820305       # 47
        joint_goal[5] = 0.767945       # 44
        group.go(joint_goal, wait=True)
        rospy.sleep(0.5)

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    # second movement : View with Camera
    def move_camera_pose(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        
        joint_goal[0] = 0.000
        joint_goal[1] = -0.5794
        joint_goal[2] = -0.3334
        joint_goal[3] = 0.000
        joint_goal[4] = 0.9424
        joint_goal[5] = 0.000
        group.go(joint_goal, wait=True)
        rospy.sleep(2)

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)      

    # fifth movement : Entering the robot.
    def move_final_pose(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = -0.261799      # -15
        joint_goal[1] = 0.226893       # 13
        joint_goal[2] = -0.314159      # -18
        joint_goal[3] = -0.0174533     # -1
        joint_goal[4] = 0.10472        # 6
        joint_goal[5] = -0.488692      # -28   
        group.go(joint_goal, wait=True)
        rospy.sleep(0.5)

        joint_goal[0] = -0.820305      # -47
        joint_goal[1] = -0.383972      # -22
        joint_goal[2] = -0.471239      # -27
        joint_goal[3] = -0.0174533     # -1
        joint_goal[4] = 0.820305       # 47
        joint_goal[5] = 0.767945       # 44
        group.go(joint_goal, wait=True)
        rospy.sleep(0.5)

        joint_goal[0] = -1.682497      # -96.4
        joint_goal[1] = -0.0418879     # -2.4
        joint_goal[2] = 0.101229       # 5.8
        joint_goal[3] = -0.146608      # -8.4
        joint_goal[4] = -0.0663225     # -3.8
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)

        joint_goal[0] = -1.85005        # -106
        joint_goal[1] = 0.4852015       # 27.8
        joint_goal[2] = -0.13439        # -7.7
        joint_goal[3] = -0.010472       # -0.6
        joint_goal[4] = -0.3228859      # -18.5
        joint_goal[5] = -0.13439        # -7.7
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)

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

    # 0(360) degree 
    def zero_quarter_high(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = -1.81514       # -104
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = -1.81514       # -104
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -1.36136       # -78
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 1.29154        # 74
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)        

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)

    # 90 degree 
    def one_quarter_high(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = -0.366519      # -21
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = -0.366519      # -21
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -1.36136       # -78
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 1.29154        # 74
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)        

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)

    # 180 degree 
    def two_quarter_high(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = -0.366519      # -21
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = 1.309          # 75
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = 1.309          # -75
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -1.36136       # -78
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 1.29154        # 74
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)        

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)

    # 270 degree 
    def three_quarter_high(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = -0.366519      # -21
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = 2.79253        # 160
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -0.610865      # -35
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 0.541052       # 31
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        joint_goal[0] = 2.79253        # 160
        joint_goal[1] = 0.0349066      # 2
        joint_goal[2] = -1.36136       # -78
        joint_goal[3] = 0.000          # 0
        joint_goal[4] = 1.29154        # 74
        joint_goal[5] = 0.0698132      # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)        

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)

# marker(fidicial static_tansform) lookup 
def lookup_trans_tar():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (tar_trans,tar_rot) = target_listener.lookupTransform('/world','/marker', rospy.Time(0))
            print("marker checkout")
            return (tar_trans, tar_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

# eef lookup
def lookup_trans_eef():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (eef_trans,eef_rot) = target_listener.lookupTransform('/world','/link_eef', rospy.Time(0))
            print("link_eef checkout")
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

# third movement : Go to charge
def move_charging_pose():
    Xarm6 = simple_move()
    # repeat 2
    for i in range(2):
        print("%d iter start" %(i+1))
        # lookup 1
        (tar_trans, tar_rot) = lookup_trans_tar()
        (eef_trans, eef_rot) = lookup_trans_eef()
        new_rot = quternion_rotation(tar_rot[0],tar_rot[1],tar_rot[2],tar_rot[3])
        
        print("%d rotate" %(i+1))
        Xarm6.move_pose(eef_trans[0],eef_trans[1],eef_trans[2],new_rot[0],new_rot[1],new_rot[2],new_rot[3])

        # lookup 2
        (tar_trans, tar_rot) = lookup_trans_tar()

        print("%d go to center" %(i+1))
        Xarm6.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.25,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    
    # sponge down
    Xarm6.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.18,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    Xarm6.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.114,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
    rospy.sleep(1)
    # Xarm6.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.18,new_rot[0],new_rot[1],new_rot[2],new_rot[3])    

def main():
    Xarm6 = simple_move()
    
    # Xarm6.move_final_pose()

    # print("---start---")
    # Xarm6.move_initial_pose()
    
    # print("---second---")
    # Xarm6.move_camera_pose() # camera pose
    
    # print("---third---")
    # move_charging_pose()
    
    # print("---final---")
    # Xarm6.move_final_pose()
    
    
    # Xarm6.zero_quarter_high()
    # Xarm6.one_quarter_high()
    # Xarm6.two_quarter_high()
    # Xarm6.three_quarter_high()


if __name__=="__main__":
    try:
        main()    
        # simple_move().move_camera_pose() # camera pose

    except rospy.ROSInterruptException:
        pass