#!/usr/bin/env python

# exmple code from (https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf)

import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3 
from gb_detection_3d_msgs.msg import BoundingBoxes3d
from gb_detection_3d_msgs.msg import BoundingBox3d


class ObjectFrame(object):
    def __init__(self):
        self.button_point_sub = rospy.Subscriber('/button_target_point', Vector3, self.cb_button_target_point)
        self.frame_pub = rospy.Publisher("target_object_frame", PoseStamped, queue_size=50)
        self.frame_broadcaster = tf.TransformBroadcaster()
        self.camera_link_listener = tf.TransformListener()


        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.target_point = Vector3()

        self.header = Header()
        self.pose = PoseStamped()

        self.frame_position = Point()
        self.frame_orientation = Quaternion()
        
        

        self.rate = rospy.Rate(10.0)
        self.trans = Point()
        self.rot = Quaternion()

    def cb_button_target_point(self, target):
        # pub = rospy.Publisher('/box_data', Box_data, queue_size=10)
        
        self.target_point = target

    def pub_frame(self):

        self.current_time = rospy.Time.now()
    
        try:

            (trans,rot) = self.camera_link_listener.lookupTransform('/world','/camera_link', rospy.Time(0))
            print('lookup')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 0
        
       
        self.frame_position.x = self.target_point.x  #+ trans[0]
        self.frame_position.y = self.target_point.y  #+ trans[1]
        self.frame_position.z = self.target_point.z  #+ trans[2]


        self.frame_orientation.x = 0#rot[0]
        self.frame_orientation.y = 0#rot[1]
        self.frame_orientation.z = 0#rot[2]
        self.frame_orientation.w = 1#rot[3]

        self.frame_broadcaster.sendTransform(
            (self.frame_position.x, self.frame_position.y, self.frame_position.z),
            (self.frame_orientation.x,self.frame_orientation.y,self.frame_orientation.z,self.frame_orientation.w),
            self.current_time,
            "target_link",
            "camera_link"
        )

       
        self.pose.header.stamp = self.current_time
        self.pose.header.frame_id = "target_link"
        self.pose.pose.position = self.frame_position
        self.pose.pose.orientation = self.frame_orientation


        
        self.frame_pub.publish(self.pose)


if __name__=="__main__":    
    try:
        print ("frame_pub")
        rospy.init_node('target_frame_publisher')
        rate = rospy.Rate(50.0)

        target_frame = ObjectFrame()

        while not rospy.is_shutdown():
            print ("frame_pub1")

            target_frame.pub_frame()
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass
        