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
        pose_target.orientation.w = ori_w 
        pose_target.orientation.z = ori_z  
        if (pos_z > 0.12):
            group.set_pose_target(pose_target)
            plan = group.go(wait=True)
        else:
            print("wrong pose")
        rospy.sleep(5)

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
        rospy.sleep(5)

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

def lookup_trans_tar():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (tar_trans,tar_rot) = target_listener.lookupTransform('/world','/fiducial_100', rospy.Time(0))

            print("detect target_pos")
            return (tar_trans, tar_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def lookup_trans_cam():
    target_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (cam_trans,cam_rot) = target_listener.lookupTransform('/world','/camera_link', rospy.Time(0))

            print("detect cam_pose")
            return (cam_trans, cam_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def quternion_rotation(tar1,tar2,tar3,tar4):
    # quaternion transform
    q_eul = euler_from_quaternion([tar1,tar2,tar3,tar4])
    q_eul = [3.14,0,q_eul[2]]
    # euler transform    
    new_rot = quaternion_from_euler(q_eul[0],q_eul[1],q_eul[2])    
    return new_rot


def main():
    Xarm6 = simple_move()
    Xarm6.move_camera_pose() # 카메라로 보는 위치로 이동
    (tar_trans, tar_rot) = lookup_trans_tar()
    (cam_trans, cam_rot) = lookup_trans_cam()
    print(tar_trans)
    print("")
    print(cam_trans)
    if abs(tar_trans[0])-abs(cam_trans[0])>0.003 or abs(tar_trans[1])-abs(cam_trans[1])>0.003:
        # go to center pose
        Xarm6.move_xyz(tar_trans[0]-0.05,tar_trans[1],tar_trans[2]+0.25)
        # lookup 1 more
        (tar_trans, tar_rot) = lookup_trans_tar()
        # new_rot = quternion_rotation(tar_rot[0],tar_rot[1],tar_rot[2],tar_rot[3])

        # move down(default center)
        Xarm6.move_xyz(tar_trans[0]-0.085,tar_trans[1]+0.017,tar_trans[2]+0.09)

    else:
        # move to revised position  
        # Xarm6.move_pose(tar_trans[0]+0.005,tar_trans[1],tar_trans[2]+0.02,new_rot[0],new_rot[1],new_rot[2],new_rot[3])
        print("move down")

if __name__=="__main__":
    try:
        main()    
        # simple_move().move_camera_pose() # 카메라로 보고

    except rospy.ROSInterruptException:
        pass


# in Quaternion [0.996, 0.092, -0.004, 0.007]
#               [ 0.99998616  0.0. 0.00526112]