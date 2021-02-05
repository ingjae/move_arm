#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String,Bool

class TCPServer(object):
    def __init__(self):
        self.HOST='192.168.1.200'
        self.PORT=8080
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
                tcp_server.check = False
                while True :
                    if tcp_server.check == True:
                        break
                    else:
                        pass
                if buffer =="0001":
                    
                    if tcp_server.check == True:
                        connectionSock.send('0011')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0001')
                elif buffer == "0002":
                   
                    if tcp_server.check == True:
                        connectionSock.send('0012')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0002')
                    
                elif buffer == "0003":
                 
                    if tcp_server.check == True:
                        connectionSock.send('0013')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0003')
                elif buffer == "0004":
                    
                    if tcp_server.check == True:
                        connectionSock.send('0014')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0004')
                elif buffer == "0005":
                    
                    if tcp_server.check == True:
                        connectionSock.send('0015')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0005')
                elif buffer == "0006":
                    
                    if tcp_server.check == True:
                        connectionSock.send('0016')
                        tcp_server.check = False
                    else:
                        connectionSock.send('0006')
                    
                else:
                    pass
                    
            except socket.timeout:
                print 'time out'
            
            tcp_server.RATE.sleep()
        print "exit"
        connectionSock.close()
    except Exception as e:
        rospy.logerr(e)
    finally:
        pass