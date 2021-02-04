#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
from math import sin, cos, pi


import copy

from std_msgs.msg import Header
from gb_detection_3d_msgs.msg import BoundingBoxes3d
from gb_detection_3d_msgs.msg import BoundingBox3d

import threading
import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
from enum import Enum
from std_msgs.msg import UInt8, Float32MultiArray
from tf.transformations import *
import tf
# from PySide import QtCore, QtGui, QtOpenGL
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point
from math import pow, atan2, sqrt

# Manipulator 
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.srv import GetJointPosition
from open_manipulator_msgs.srv import GetKinematicsPose
from open_manipulator_msgs.srv import SetActuatorState


def is_reached(x,y,z, c_x,c_y,c_z,tolerance): # 미완성
    if (abs(x - c_x) > tolerance and abs(y- c_y) > tolerance and abs(z-c_z) > tolerance):
        return False
    else:
        return True
def is_over_limit(joint_state_effort):
    total_effort = 0
    for effort in joint_state_effort:
        total_effort += effort
    
    # print (total_effort)
    return total_effort
class SimpleMove():
    def __init__(self):   
        self.set_joint_position = rospy.ServiceProxy('open_manipulator_6dof/goal_joint_space_path', SetJointPosition)
        self.set_kinematics_position = rospy.ServiceProxy('open_manipulator_6dof/goal_task_space_path_position_only', SetKinematicsPose)
        self.set_kinematics_position_from_present = rospy.ServiceProxy('open_manipulator_6dof/goal_task_space_path_from_present_position_only', SetKinematicsPose)
        self.set_joint_position_from_present = rospy.ServiceProxy('open_manipulator_6dof/goal_joint_space_path_from_present', SetJointPosition)
        self.set_actuator_state = rospy.ServiceProxy('open_manipulator_6dof/set_actuator_state', SetActuatorState)
        self.set_gripper_control = rospy.ServiceProxy('open_manipulator_6dof/goal_tool_control', SetJointPosition)

        self.open_manipulator_joint_states_sub_ = rospy.Subscriber('open_manipulator_6dof/joint_states', JointState, self.jointStatesCallback)
        self.open_manipulator_kinematics_pose_sub_ = rospy.Subscriber('open_manipulator_6dof/gripper/kinematics_pose', KinematicsPose, self.kinematicsPoseCallback)

        self.listener = tf.TransformListener()
        self.objectPose = PoseStamped()

        self.pannelPose = {}

        self.pickTargetPose = PoseStamped()
        self.currentToolPose = Pose()

        self.is_triggered = False
        self.first_pose_trigger = False
        self.jointStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.effort = []
        self.over_limit = False
        # (joint 1,2,3,4 / gripper ) (joint 1,2,3,4 velocity / gripper velocity )

        
    def jointStatesCallback(self, msg):
        self.over_limit = False #정상
        self.effort = msg.effort 
        for i, pose in enumerate(msg.position):
            self.jointStates[i] = pose
        total_effort = 0
        for effort in self.effort:
            total_effort += effort
        if (total_effort > 1200):
            self.over_limit = True

    def kinematicsPoseCallback(self, msg):
        self.currentToolPose.position.x = msg.pose.position.x
        self.currentToolPose.position.y = msg.pose.position.y
        self.currentToolPose.position.z = msg.pose.position.z

    def moveToHomePose(self):
        rospy.logwarn("move to Home")
        self.moveByJoint(0.000, -1.654, -0.216, 0.000, 1.864, 0.018) # 누르기 준비 자세


    def getCurrentPose(self):
        x = self.currentToolPose.position.x
        y = self.currentToolPose.position.y
        z = self.currentToolPose.position.z
        return x,y,z

    def getCurrentJoint(self):
        states = self.jointStates
        return states
    

    def moveByJoint(self, joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=0.0, joint6=0.0):
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4','joint5','joint6']  
        joint_position.position = [joint1, joint2, joint3, joint4, joint5, joint6]  

        operating_time = 5

        try:
            resp = self.set_joint_position("",joint_position, operating_time)
            
            rospy.sleep(operating_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return resp

    def moveByXYZ(self, x=0.287, y=0.10, z=0.2, operating_time=5):
        planning_group = "gripper"
        end_effector_name = "gripper"
        kinematics_pose = KinematicsPose()  

        kinematics_pose.pose.position.x = x
        kinematics_pose.pose.position.y = y
        kinematics_pose.pose.position.z = z
        kinematics_pose.pose.orientation.x = 0.00531
        kinematics_pose.pose.orientation.y = 0.00850
        kinematics_pose.pose.orientation.z = 0.01270
        kinematics_pose.pose.orientation.w = 0.99986
            
        try:
            resp = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
            rospy.logdebug("Move to Task Space %f / %f / %f" %(x, y, z))
            control_count = (operating_time * 2) 

            for i in range(control_count): #0,1,2,3,4 ... 9
                if self.over_limit == False:
                    rospy.sleep(0.5)
                else:
                    rospy.logwarn("over current limitation ")
                    pass
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return resp

    def getJointEffort(self,i=10):
        if (i ==10):
            return self.effort
        else:
            return self.effort[i]
    def moveEndEffector(self,triggered = True):
        if triggered == True:
            
            joint_position = JointPosition()
            joint_position.joint_name = ['gripper']  
            joint_position.position = [0.017]    

            operating_time = 3
            print("true")

            try:
                resp = self.set_gripper_control("",joint_position, operating_time)
                rospy.sleep(operating_time)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return False
            
        elif triggered == False:
            joint_position = JointPosition()
            joint_position.joint_name = ['gripper']  
            joint_position.position = [-0.017]    
            
            operating_time = 3
            print("false")

            try:
                resp = self.set_gripper_control("",joint_position, operating_time)
                rospy.sleep(operating_time)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return False



if __name__=="__main__":    
    rospy.init_node('push_button_client')
    rate = rospy.Rate(20.0)
    rospy.set_param("status",-1)
    target_listener = tf.TransformListener()
    OFFSET_X = -0.13 
    OFFSET_Y = +0.00
    OFFSET_Z = +0.00
    
    try:
        simplemove = SimpleMove()
        #simplemove.moveByJoint(-1.5,0.227, 0.313, -0.012, 1.109, -0.035) # opencr 켜지면 (이동 중 로봇 팔 위치 )     
        
           
        while not rospy.is_shutdown():
            if (rospy.get_param('status')== -1): # 명령 대기 //아무것도 하지 않음
                pass
                # print("waiting for command, home pose")
            elif (rospy.get_param('status')== 1): # 수행 완료 후 홈포즈 : (누르기 준비 자세)
                rospy.logwarn("button pushed!")
                rospy.logwarn("waiting for command, home pose")
                simplemove.moveToHomePose()    
                #simplemove.moveByJoint(2.8, 0.071, 0.578, 0.017, 1.039, -0.051) # opencr 켜지면 (이동 중 로봇 팔 위치 )     
                
                rospy.set_param("status",-1) 
            elif (rospy.get_param('status')== 2): # 수행 실패
                rospy.logwarn("button push fail")
                rospy.logwarn("waiting for command, home pose")
                simplemove.moveToHomePose()
                rospy.set_param("status",-1)
            elif (rospy.get_param('status')== 3): ################################################### 4층 6층 누르기
                simplemove.moveToHomePose()
                # simplemove.moveByJoint(0.0,-0.893, 0.224, -0.140, 0.522,0.063) #  4층
                #simplemove.moveByJoint(0.0,-0.893, 0.224, -0.140, 0.522,0.063) #  6층
                simplemove.moveByJoint(1.454,-1.300, -0.092, -1.477, 1.397,-0.017) #  4층

                simplemove.moveEndEffector(False)
                simplemove.moveEndEffector(True) #누르기 
                simplemove.moveToHomePose()
                rospy.set_param("status",-1)
            elif (rospy.get_param('status')== 4): ##################################################### UP 버튼 누르기
                simplemove.moveToHomePose()
                simplemove.moveByJoint(0.058, -1.029, 0.158, -0.025, 0.695, -0.023) # 누르기 준비 자세
                
                simplemove.moveEndEffector(False)
                simplemove.moveEndEffector(True) #누르기 
                simplemove.moveToHomePose()
                
                rospy.set_param("status",-1)
            elif (rospy.get_param('status')== 5): # 현재 상태 
                rospy.logwarn("info")
                joint = simplemove.getCurrentJoint()
                print(joint)
                rospy.set_param("status",-1)  
            elif (rospy.get_param('status')== 6): 
                simplemove.moveByJoint(1.563,-1.342, 0.189, -0.005, 1.328, +0.035) # opencr 켜지면 (이동 중 로봇 팔 위치 )     
                simplemove.moveByJoint(-1.5,-1.227, 0.313, -0.012, 1.109, -0.035) # opencr 켜지면 (이동 중 로봇 팔 위치 )  
                simplemove.moveByJoint(-1.5,0.227, 0.313, -0.012, 1.109, -0.035)
                
                rospy.set_param("status",-1)
                            
            ####################################[Scenario]#######################################
            elif (rospy.get_param('status')== 0): # 6 floor
                simplemove.moveByJoint(-1.5,-1.227, 0.313, -0.012, 1.109, -0.035) # opencr 켜지면 (이동 중 로봇 팔 위치 )     
                simplemove.moveByJoint(0.0,-1.227, 0.313, -0.012, 1.109, -0.035) # opencr 켜지면 (이동 중 로봇 팔 위치 )     
                simplemove.moveToHomePose() # 누르기 준비 
                rospy.logwarn("Press button")
                print("executing command")
                try:
                    (trans,rot) = target_listener.lookupTransform('/world','/target_link', rospy.Time(0)) 
                    trans[0]=trans[0] + OFFSET_X
                    trans[1]=trans[1] + OFFSET_Y
                    trans[2]=trans[2] + OFFSET_Z
                    if (0<=rospy.get_param("button")<=17): # 6F button! 
                        trans[0]=trans[0] - 0.02
                        trans[1]=trans[1] - 0.00
                        trans[2]=trans[2] + 0.08          

                        simplemove.moveByJoint(-0.03221359848976135, -0.14726215600967407, 0.9004467725753784, 0.006135923322290182, -0.9142525792121887, 0.0015339808305725455)
                    else:  # UP button!   
                        
                        trans[0]=trans[0] - 0.00
                        trans[1]=trans[1] + 0.02
                        trans[2]=trans[2] + 0.08   
                        simplemove.moveByJoint(-1.0814565420150757, -0.026077674701809883, -0.07363107800483704, -1.494097352027893, -1.072252631187439, 1.5370488166809082)
                
                        
                    
                    print("move")
                    simplemove.moveByXYZ(trans[0],trans[1], trans[2],5) #움직여라
                    x,y,z = simplemove.getCurrentPose()

                    if is_reached(trans[0],trans[1], trans[2],x,y,z,0.005) == True  :  
                        simplemove.moveEndEffector(False)
                        rospy.sleep(1)
                        simplemove.moveEndEffector(True) #누르기 
                        rospy.set_param("status",1) # 수행완료
                    else :
                        rospy.set_param("status",2) # 수행실패  

                        print ("fail")
        
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("fail") 

            ##############################################################################################

            
        rate.sleep            
          
            
                
    except rospy.ROSInterruptException:
        pass