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
from gb_detection_3d_msgs.msg import BoundingBoxes3d
from gb_detection_3d_msgs.msg import BoundingBox3d
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
            
        # if init_pose == 0:  # 중립포즈 [카메라로 위치 파악]
        #     return -0.054, -1.537, 1.086, 0.000, 2.000, 0.000
        # elif init_pose == 1: #밑 의 층 누르기 포즈
        #     return 0.000, 0.121, 1.959, 0.000, -0.660, 0.000
        # elif init_pose == 2:
        #     pass
        # elif init_pose == 3:
        #     pass
# def is_reached(pose_target):
#         group = self.group

#         pose_current = group.get_current_pose()
#         if (abs(pose_target.position.x - pose_current.pose.position.x) <0.1 and abs(pose_target.position.y- pose_current.pose.position.y) <0.1 and abs(pose_target.position.z- pose_current.pose.position.z) <0.1):
#             group.clear_pose_targets()
#             return True
#         else :
#             return False
class simple_move():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('push_button_client')
        target_listener = tf.TransformListener()

        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm")
        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        pose_target = geometry_msgs.msg.Pose()

        rospy.set_param("execute",-1)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.OFFSET_X = - 0.14
        self.OFFSET_Y = + 0.033
        self.OFFSET_Z = + 0.04
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

    def move_xyz(self,x,y,z):
        group = self.group
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        plan1 = group.plan()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = 0.00070281789782  
        pose_target.orientation.y = 0.706563080537  
        pose_target.orientation.z = -0.00315379182291  
        pose_target.orientation.w = 0.707642687284 

        group.set_pose_target(pose_target)
        rospy.logwarn("move to xyz")
        plan = group.go(wait=True)
        rospy.sleep(7)

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
    
    def get_end_effector_pose(self):
        group = self.group
        end_effector_pose = group.get_current_pose("arm")
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
    def push_action(self,x,y,z):
        group = self.group
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        plan1 = group.plan()
        pose_target.position.x = x + 0.02
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = 0.00070281789782  
        pose_target.orientation.y = 0.706563080537  
        pose_target.orientation.z = -0.00315379182291  
        pose_target.orientation.w = 0.707642687284 

        group.set_pose_target(pose_target)
        rospy.logwarn("move to xyz")
        plan = group.go(wait=True)
        rospy.sleep(3)

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)

    def back_action(self,x,y,z):
        group = self.group
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        plan1 = group.plan()
        pose_target.position.x = x 
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = 0.00070281789782  
        pose_target.orientation.y = 0.706563080537  
        pose_target.orientation.z = -0.00315379182291  
        pose_target.orientation.w = 0.707642687284 

        group.set_pose_target(pose_target)
        rospy.logwarn("move to xyz")
        plan = group.go(wait=True)
        rospy.sleep(3)

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_target, current_pose, 0.01)

def main():
    james_ear = simple_move()
    
    
   
    # tf listener

    target_listener = tf.TransformListener()
    
   
    rate = rospy.Rate(1.0)
    james_ear.move_home()
    while not rospy.is_shutdown():
        if (rospy.get_param('execute')== -1): #do nothing
            print("waiting for command")       
        elif (rospy.get_param('execute')== 1):
            rospy.logwarn("button pushed!")
        elif (rospy.get_param('execute')== 3):
            print("executing command")
            try:
                (trans,rot) = target_listener.lookupTransform('/world','/target_link', rospy.Time(0)) # 계산
                #offset
         
                trans[0] = trans[0]  + self.OFFSET_X #0.03 덜 가도록
                trans[1] = trans[1]  + self.OFFSET_Y
                trans[2] = trans[2]  + self.OFFSET_Z
                # print (trans[0],trans[1],trans[2])
                
                # james_ear.move_joint() ## init pose
                
                 # 가라
                print (trans[0],trans[1],trans[2])
        
            
                if james_ear.move_xyz(trans[0],trans[1],trans[2]) == True :
                    rospy.logwarn("now push!")
                    
                    if james_ear.push_action(trans[0],trans[1],trans[2]) == True :
                        if james_ear.back_action(trans[0],trans[1],trans[2]) == True :
                            rospy.set_param("execute",1)# 도착
                            james_ear.move_home()
                            rospy.set_param("execute",-1) # 대기 상태
                    else:
                        rospy.set_param("execute",2)
                else:
                    rospy.set_param("execute",2)
                    

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("fail")


        elif (rospy.get_param('execute')== 0):
                if james_ear.move_home() == True:
                    rospy.set_param("execute",-1) # 대기 상태
                else :
                    pass
        elif (rospy.get_param('execute')== 2):
            rospy.logwarn("goal failed!")
            rospy.set_param("execute",0)

        # test pose  0.16288592571804772, -0.14714986603188876, 0.3095673581235922
        elif (rospy.get_param('execute')== 4): #test move
            if james_ear.move_xyz(0.16288592571804772,-0.14714986603188876,0.3095673581235922) == True :

                rospy.set_param("execute",1)# 도착
                james_ear.move_home()
                rospy.set_param("execute",-1) # 대기 상태
            else:
                rospy.set_param("execute",2)
        elif (rospy.get_param('execute')== 5): #test move
            if james_ear.move_xyz(0.228,-0.121,0.280) == True :

                rospy.set_param("execute",1)# 도착
                james_ear.move_home()
                rospy.set_param("execute",-1) # 대기 상태
            else:
                rospy.set_param("execute",2)

            
            
        rate.sleep()


     
    #없어도 되나?

    # moveit_commander.roscpp_shutdown()



if __name__=="__main__":    
    try:
        main()
        
        
            
    except rospy.ROSInterruptException:
        pass


#0.6353041962439351, -0.19837263155652396, 0.14632660184374774)