#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import rospy
from std_msgs.msg import String,Bool,Int8
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

def compare_joint_values(current_values_list,values_list,tolerance):
    if (abs(current_values_list[0]-values_list[0]) < tolerance) and (abs(current_values_list[1]-values_list[1]) < tolerance) and \
    (abs(current_values_list[2]-values_list[2]) < tolerance) and (abs(current_values_list[3]-values_list[3]) < tolerance) and \
    (abs(current_values_list[4]-values_list[4]) < tolerance) and (abs(current_values_list[5]-values_list[5]) < tolerance) == True:
        return True
    else:
        return False
    
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

        # Setting Joint Values [YOU SHUOLD CHANGE HERE]#############################################
        self.init_a_list = [2.4207311, 0.610855, -0.1151898, 0.1064633, -0.3158993, 0.2722668 ]
        self.init_b_list = [2.4765807, -0.0017453, 0.0785385, 0.1064633, 0.1640582, -0.9284996 ]
        self.init_c_list = [2.7523381, -0.7923662, 0.0296701, 0.104718, 0.8115645, 0.2722668 ]   
        self.ready_0_list = [0.0750479, -0.3438241, -0.6649593, -0.0069812, 1.021005, -0.0226889 ]
        self.ready_90_list = [0.0750479, -0.3438241, -0.6649593, -0.0069812, 1.021005, -0.0226889 ]
        self.ready_180_list = [0.0750479, -0.3438241, -0.6649593, -0.0069812, 1.021005, -0.0226889 ]
        self.ready_270_list = [0.0750479, -0.3438241, -0.6649593, -0.0069812, 1.021005, -0.0226889 ]
        #############################################################################################

        # Client
        self.all_0_list = [0,0,0,0,0,0]
        self.tcp_sub = rospy.Subscriber('tcptopic', String, self.tcpCallback)
        self.tcp_msg = String()
        self.finish_pub = rospy.Publisher('is_finished',Bool,queue_size=10) # 
        self.check_pub = rospy.Publisher('check',Int8,queue_size=10) # 
        self.rate = rospy.Rate(1.0)
        self.once = False
        self.is_finished = False
        self.checked = 0

    def tcpCallback(self, msg):
        self.tcp_msg = msg

    def finishPublisher(self):
        self.is_finished = True
        self.finish_pub.publish(self.is_finished)
        self.is_finished = False

    def checkPublisher(self,checked_list): 
        if checked_list == self.init_a_list :
            self.checked = 1
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.init_b_list : 
            self.checked = 2
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.init_c_list : 
            self.checked = 3
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.ready_0_list : 
            self.checked = 4
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.ready_90_list : 
            self.checked = 5
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.ready_180_list : 
            self.checked = 6
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.ready_270_list : 
            self.checked = 7
            self.check_pub.publish(self.checked)
            self.checked = 0
        elif checked_list == self.all_0_list :
            self.checked = 0
            self.check_pub.publish(self.checked) 
        else :
            self.checked = 0
            self.check_pub.publish(self.checked) 
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
        joint_goal[0] = self.init_a_list[0]      # -103.8
        joint_goal[1] = self.init_a_list[1]        # 41
        joint_goal[2] = self.init_a_list[2]      # -10.3
        joint_goal[3] = self.init_a_list[3]           # 0
        joint_goal[4] = self.init_a_list[4]      # -32.9
        joint_goal[5] = self.init_a_list[5]       # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(1)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_initial_pose_b(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = self.init_b_list[0]        # -106
        joint_goal[1] = self.init_b_list[1]       # 27.8
        joint_goal[2] = self.init_b_list[2]        # -7.7
        joint_goal[3] = self.init_b_list[3]       # -0.6
        joint_goal[4] = self.init_b_list[4]      # -18.5
        joint_goal[5] = self.init_b_list[5]        # -7.7
        group.go(joint_goal, wait=True)
        rospy.sleep(0.05)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_initial_pose_c(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = self.init_c_list[0]       # -96.4
        joint_goal[1] = self.init_c_list[1]      # -2.4
        joint_goal[2] = self.init_c_list[2]        # 5.8
        joint_goal[3] = self.init_c_list[3]       # -8.4
        joint_goal[4] = self.init_c_list[4]      # -3.8
        joint_goal[5] = self.init_c_list[5]       # 4
        group.go(joint_goal, wait=True)
        rospy.sleep(3)
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 3)  

    def move_camera_pose(self,degree):
        if degree == 0: # 0(360) degree
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = self.ready_0_list[0]       # -104
            joint_goal[1] = self.ready_0_list[1]
            joint_goal[2] = self.ready_0_list[2]
            joint_goal[3] = self.ready_0_list[3]
            joint_goal[4] = self.ready_0_list[4]
            joint_goal[5] = self.ready_0_list[5]
            group.go(joint_goal, wait=True)
            rospy.sleep(2)  
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 90:  # 90 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = self.ready_90_list[0]      # -21
            joint_goal[1] = self.ready_90_list[1]
            joint_goal[2] = self.ready_90_list[2]
            joint_goal[3] = self.ready_90_list[3]
            joint_goal[4] = self.ready_90_list[4]
            joint_goal[5] = self.ready_90_list[5]
            group.go(joint_goal, wait=True)
            rospy.sleep(2)     
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 180:   # 180 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = self.ready_180_list[0]        # 75
            joint_goal[1] = self.ready_180_list[1]
            joint_goal[2] = self.ready_180_list[2]
            joint_goal[3] = self.ready_180_list[3]
            joint_goal[4] = self.ready_180_list[4]
            joint_goal[5] = self.ready_180_list[5]
            group.go(joint_goal, wait=True)
            rospy.sleep(2)            
            current_joints = self.group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 3)

        elif degree == 270:  # 270 degree 
            group = self.group
            joint_goal = group.get_current_joint_values()
            joint_goal[0] = self.ready_270_list[0]        # 160
            joint_goal[1] = self.ready_270_list[1]
            joint_goal[2] = self.ready_270_list[2]
            joint_goal[3] = self.ready_270_list[3]
            joint_goal[4] = self.ready_270_list[4]
            joint_goal[5] = self.ready_270_list[5]
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


    # charging down
    client.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.170,new_rot[0],new_rot[1],new_rot[2],new_rot[3]) # before charging
    client.move_pose(tar_trans[0],tar_trans[1],tar_trans[2]+0.150,new_rot[0],new_rot[1],new_rot[2],new_rot[3]) # charging
    
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
            if client.tcp_msg.data == "0010":
                print("Initial_setting_a")
                client.move_initial_pose_a()
                client.finishPublisher()
            elif client.tcp_msg.data == "0020":
                print("Initial_setting_b")
                client.move_initial_pose_b()
                client.finishPublisher()
            elif client.tcp_msg.data == "0030":
                print("Initial_setting_c")
                client.move_initial_pose_c()
                client.finishPublisher()
            elif client.tcp_msg.data == "0110":
                print("Initial_camera_pose")
                client.move_camera_pose(0)
                client.finishPublisher()
            elif client.tcp_msg.data == "0120":
                print("Initial_camera_pose")
                client.move_camera_pose(90)
                client.finishPublisher()
            elif client.tcp_msg.data == "0130":
                print("Initial_camera_pose")
                client.move_camera_pose(180)
                client.finishPublisher()
            elif client.tcp_msg.data == "0140":
                print("Initial_camera_pose")
                client.move_camera_pose(270)
                client.finishPublisher()
            elif client.tcp_msg.data == "0210":
                print("Charging pose")
                charging_point = charging_down_pose()
                client.finishPublisher()
            elif client.tcp_msg.data == "0220":
                print("UnCharging pose")
                charging_up_pose(charging_point[0],charging_point[1],charging_point[2],charging_point[3],charging_point[4],charging_point[5],charging_point[6])
                client.finishPublisher()
            elif client.tcp_msg.data == "1300": #checking joint state
                print("Checking Joint state")
                current_joints = client.group.get_current_joint_values()
                if compare_joint_values(current_joints,client.init_a_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.init_a_list)
                    client.finishPublisher()
                    rospy.logwarn("init a")
                elif compare_joint_values(current_joints,client.init_b_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.init_b_list)
                    client.finishPublisher()
                    rospy.logwarn("init b")
                elif compare_joint_values(current_joints,client.init_c_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.init_c_list)
                    client.finishPublisher()
                    rospy.logwarn("init c")
                elif compare_joint_values(current_joints,client.ready_0_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.ready_0_list)
                    client.finishPublisher()
                    rospy.logwarn("ready 0")
                elif compare_joint_values(current_joints,client.ready_90_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.ready_90_list)
                    client.finishPublisher()
                    rospy.logwarn("ready 90")
                elif compare_joint_values(current_joints,client.ready_180_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.ready_180_list)
                    client.finishPublisher()
                    rospy.logwarn("ready 180")
                elif compare_joint_values(current_joints,client.ready_270_list,0.01) == True: # tolerance 0.01 
                    client.checkPublisher(client.ready_270_list)
                    client.finishPublisher()
                    rospy.logwarn("ready 270")
                else:
                    client.checkPublisher(client.all_0_list)
                    client.finishPublisher()
                    rospy.logwarn("unkown pose detected")
            elif client.tcp_msg.data == "":
                client.finishPublisher()
            elif client.tcp_msg.data == "1100": #checking joint state
                client.finishPublisher()
                print("server helth check")
            elif client.tcp_msg.data == "1200": #checking joint state
                client.finishPublisher()
                print("camera helth check")
            # elif client.tcp_msg.data == "1300": #checking joint state
            #     client.finishPublisher()
            #     print("robot Joint state")
            #     # pass
            else:
                rospy.logwarn("Protocol Error")
                client.finishPublisher()

            client.tcp_msg.data ="" 
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass
