#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String
 

class TCPListener(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber('tcptopic', String, self.tcpCallback)
        self.tcp_msg = String()

        self.rate = rospy.Rate(1.0)
        self.once = False

    def tcpCallback(self, msg):
        self.tcp_msg = msg


if __name__=="__main__":    
    try:
        rospy.init_node('tcp_listener')
        rate = rospy.Rate(1)
        tcp = TCPListener()
        while not rospy.is_shutdown():
            # print(tcp.tcp_msg)
            if tcp.tcp_msg.data == "00000001":
                print("Moving Out Position")
            elif tcp.tcp_msg.data == "00000002":
                print("QR Position and Calculate TF")
            elif tcp.tcp_msg.data == "00000004":
                print("Charging Position")
            elif tcp.tcp_msg.data == "00000003":
                print("Waiting Position")
            elif tcp.tcp_msg.data == "00000005":
                print("Movig In Position")
            elif tcp.tcp_msg.data == "":
                print("Waiting Command")
            else:
                print("Error")
            tcp.tcp_msg.data ="" 
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass