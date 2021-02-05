#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import rospy
from std_msgs.msg import String,Bool
 
## move client 
class TCPListener(object):
    def __init__(self):
        self.tcp_sub = rospy.Subscriber('tcptopic', String, self.tcpCallback)
        self.tcp_msg = String()
        self.finish_pub = rospy.Publisher('is_finished',Bool,queue_size=10)
        self.rate = rospy.Rate(1.0)
        self.once = False
        self.is_finished = False

    def tcpCallback(self, msg):
        self.tcp_msg = msg.data

    def checkPublisher(self):
        self.is_finished = True
        self.finish_pub.publish(self.is_finished)
        self.is_finished = False

if __name__=="__main__":    
    try:
        rospy.init_node('tcp_listener')
        rate = rospy.Rate(10)
        tcp = TCPListener()
        while not rospy.is_shutdown():
            # print(tcp.tcp_msg)
            if tcp.tcp_msg == "0001":
                print("Moving Out Position")
                time.sleep(5)# robot arm moving
                tcp.checkPublisher()
            elif tcp.tcp_msg == "0002":
                print("QR Position and Calculate TF")
                time.sleep(5)# robot arm moving
                tcp.checkPublisher()
            elif tcp.tcp_msg == "0004":
                print("Charging Position")
                time.sleep(5)# robot arm moving
                tcp.checkPublisher()
            elif tcp.tcp_msg == "0003":
                print("Waiting Position")
                time.sleep(5)# robot arm moving
                tcp.checkPublisher()
            elif tcp.tcp_msg == "0005":
                print("Movig In Position")
                time.sleep(5)# robot arm moving
                tcp.checkPublisher()
            elif tcp.tcp_msg == "":
                pass
            else:
                print("Error")
            tcp.tcp_msg.data ="" 
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass