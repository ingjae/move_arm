#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String,Bool

class TCPServer(object):
    def __init__(self):
        self.HOST='192.168.1.100'
        self.PORT=8008
        self.BUFFER=4096
        self.RATE = rospy.Rate(10)
        self.pub=rospy.Publisher('tcptopic',String,queue_size=10)
        self.sub = rospy.Subscriber('is_finished', Bool,self.checkCallback)
        self.check = False

    def checkCallback(self, msg):
        self.check = msg.data


if __name__=="__main__":    
    try:
        rospy.init_node('tcpserver',anonymous=0)
        tcp_server = TCPServer()
        serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        serverSock.bind((tcp_server.HOST,tcp_server.PORT))
        serverSock.listen(5)
        print 'TCP Server Activate'

        connectionSock,addr=serverSock.accept() #waiting connect
        print 'TCP Server Connected'
        
        while not rospy.is_shutdown():
            try :
                # connectionSock.settimeout(5)
                buffer=connectionSock.recv(tcp_server.BUFFER)
                # print buf
                tcp_server.pub.publish(buffer)
                time.sleep(6)
                if tcp_server.check == True:
                    connectionSock.send('00000010')
                    tcp_server.check = False
                else:
                    connectionSock.send('00000011')
                    
            except socket.timeout:
                print 'time out'
            
            tcp_server.RATE.sleep()
        print "exit"
        connectionSock.close()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass




 
