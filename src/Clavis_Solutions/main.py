#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String,Bool,Int8
from MoveClient import MoveClient
from TCPServer import TCPServer
import socket

if __name__=="__main__":    
    try:
        rospy.init_node('tcpserver',anonymous=0)
        tcp_server = TCPServer()
        serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        serverSock.bind((tcp_server.HOST,tcp_server.PORT))
        serverSock.listen(5)
        
        rate = rospy.Rate(5)
        print 'TCP Server Activate'

        connectionSock,addr=serverSock.accept() #waiting connect  # always server connection 
        print 'TCP Server Connected'
        while not rospy.is_shutdown():
            try :
                # connectionSock,addr=serverSock.accept() #waiting connect every time 
                # print 'TCP Server Connected'
                # connectionSock.settimeout(5)
                buffer=connectionSock.recv(tcp_server.BUFFER)
                # print buf
                tcp_server.pub.publish(buffer)
            except socket.timeout:
                print 'time out'
            
            
            rate.sleep()
       
    except Exception as e:
        print "exit"
        # connectionSock.close()
        rospy.logerr(e)
    finally:
        pass