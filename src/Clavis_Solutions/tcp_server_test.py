#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String,Bool,Int8,Header,Time,UInt32
from sensor_msgs.msg import Image,CameraInfo
import os


class TCPServer(object):
    def __init__(self):
        self.HOST='192.168.1.200'
        self.PORT=8080
        self.BUFFER=1024 #한번에 수신되는 데이터의 크기 
        self.RATE = rospy.Rate(20)              
        self.pub=rospy.Publisher('tcptopic',String,queue_size=10)
   

if __name__=="__main__": 
    while True: # re connection 을 위한 loop
        try:
            rospy.init_node('tcpserver',anonymous=0)
            tcp_server = TCPServer()
            serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            serverSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)# WinError 10048 에러 해결를 위해 필요합니다. 
            serverSock.bind((tcp_server.HOST,tcp_server.PORT))
            serverSock.listen(5) # 5개의 클라이언트를 기다림
            print 'TCP Server Activate'

            connectionSock,addr=serverSock.accept() #waiting connect # return tuple 
            print (str(addr),'TCP Server Connected') 
            
            while not rospy.is_shutdown(): #받은 데이터를 퍼블리시 하는  loop
                try :
                    buffer=connectionSock.recv(tcp_server.BUFFER)
                    if (buffer == ""):
                        print("no connection")
                        break
                    else: # 정상 상태
                        tcp_server.pub.publish(buffer)
                        # pass
                
                except socket.timeout:
                    print ('time out')
                    pass
                except socket.error:
                    print ("socket error")
                    pass
                    # break 를 해도 될거 가음
                tcp_server.RATE.sleep()
            print "exit"
            connectionSock.close()
        except Exception as e:
            rospy.logerr(e)
        finally:
            pass