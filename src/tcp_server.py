#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String
 
######################tcp begining
HOST='192.168.1.100'
# HOST = socket.gethostbyname("localhost")
print HOST
PORT=8008
 
BUFFER=4096
 
serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
 
serverSock.bind((HOST,PORT))
 
serverSock.listen(5)
 
################ros begining
rospy.init_node('tcptalker',anonymous=0)
pub=rospy.Publisher('tcptopic',String,queue_size=10)
 
print 'i am listening'
 
connectionSock,addr=serverSock.accept() #waiting connect
while not rospy.is_shutdown():
    
    try :
        connectionSock.settimeout(5)
        buf=connectionSock.recv(BUFFER)
        # print buf
        pub.publish(buf)
        time.sleep(1)
    except socket.timeout:
        print 'time out'
    connectionSock.send('yes i recv')
print "exit"
connectionSock.close()
