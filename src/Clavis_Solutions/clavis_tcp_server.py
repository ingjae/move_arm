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
        self.BUFFER=1024
        self.RATE = rospy.Rate(10)              
        self.pub=rospy.Publisher('tcptopic',String,queue_size=10)
        self.sub = rospy.Subscriber('is_finished', Bool ,self.finishCallback) 
        self.sub = rospy.Subscriber('check', Int8 ,self.checkCallback) 
        self.sub = rospy.Subscriber('camera/color/camera_info', CameraInfo ,self.cameraCallback) 
        
        
        self.finish = False # is_finished?
        self.check = 0 # new!! checking joint state
        self.camera_check = False # new!! checking camera topic state
        self.camera_info = UInt32() # new!! checking camera topic state



    def checkCallback(self, msg):
        self.check = msg.data

    def finishCallback(self, msg):
        self.finish = msg.data

    def cameraCallback(self, msg):
        self.camera_info = msg.header.stamp.secs
       
        # print self.camera_info
        # print time
        # print "---------"
        # self.camera_check = True

    def camera_health_check(self) :
        time = rospy.get_rostime()
        if abs(self.camera_info - time.secs) > 1:
            # print "lost conection"
            return False
        else : 
            return True
            # print "connected"



if __name__=="__main__":
    while True:
        try:
            rospy.init_node('tcpserver',anonymous=0)
            tcp_server = TCPServer()
            serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            serverSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)# To Fix WinError 10048 error
            serverSock.bind((tcp_server.HOST,tcp_server.PORT))
            serverSock.listen(5)
            print 'TCP Server Activate'

            connectionSock,addr=serverSock.accept() #waiting connect
            print 'TCP Server Connected'
            
            while not rospy.is_shutdown():
                try :
                    # tcp_server.camera_check = False
                    buffer=connectionSock.recv(tcp_server.BUFFER)
                    # print buf
                    if (buffer == ""):
                        print("no connection")
                        break
                    else:
                        tcp_server.pub.publish(buffer)

                    while True :
                        if tcp_server.finish == True:
                            break
                        else:
                            pass
                    if buffer =="0010": 
                        if tcp_server.finish == True:
                            connectionSock.send('0011')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0012')
                    elif buffer == "0020":
                    
                        if tcp_server.finish == True:
                            connectionSock.send('0021')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0022')
                        
                    elif buffer == "0030":
                    
                        if tcp_server.finish == True:
                            connectionSock.send('0031')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0032')
                    elif buffer == "0110":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0111')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0112')
                    elif buffer == "0120":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0121')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0122')
                    elif buffer == "0130":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0131')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0132')
                    elif buffer == "0140":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0141')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0142')

                    elif buffer == "0210":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0211')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0212')
                    elif buffer == "0220":
                        
                        if tcp_server.finish == True:
                            connectionSock.send('0221')
                            tcp_server.finish = False
                        else:
                            connectionSock.send('0222')
                    # Add checking joint state Protocol
                    elif buffer == "1300":  
                        # sleep(1) #sleep when code is not working  
                        if tcp_server.check == 0: # error code 
                            connectionSock.send('1320')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 1: # init pose a
                            connectionSock.send('1301')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 2: # init pose b
                            connectionSock.send('1302')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 3: # init pose c
                            connectionSock.send('1303')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 4: # ready pose 0
                            connectionSock.send('1311')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 5: # ready pose 90
                            connectionSock.send('1312')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 6: # ready pose 180
                            connectionSock.send('1313')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        elif tcp_server.check == 7: # ready pose 270
                            connectionSock.send('1314')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                        else: # protocol error 
                            connectionSock.send('1321')
                            tcp_server.finish = 0
                            tcp_server.finish = False
                    elif buffer =="1100": # server health check 
                        connectionSock.send('1101')
                        tcp_server.finish = False
                    elif buffer =="1200": # camera health check 
                        if tcp_server.camera_health_check() == True:
                            connectionSock.send('1201')
                        else:
                            connectionSock.send('1202')

                        tcp_server.finish = False
                    elif buffer =="1000": # reset all
                        connectionSock.send('1001')
                        os.system("sudo systemctl restart bringup_ros.service")

                        tcp_server.finish = False
                    else:
                        # connectionSock.send('0321')

                        pass
                        
                except socket.timeout:
                    print 'time out'
                except socket.error:
                    print ("socket error")
                    break
                
                tcp_server.RATE.sleep()
            print "exit"
            connectionSock.close()
        except Exception as e:
            rospy.logerr(e)
        finally:
            pass