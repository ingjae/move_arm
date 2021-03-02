#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import socket
import rospy
from std_msgs.msg import String,Bool,Int8,Header,Time,UInt32
from sensor_msgs.msg import Image,CameraInfo

class TCPServer(object):
    def __init__(self):
        self.HOST='192.168.1.200'
        self.PORT=8080
        self.BUFFER=4096
        self.RATE = rospy.Rate(5)              
        self.pub=rospy.Publisher('tcptopic',String,queue_size=10)
        self.finish_sub = rospy.Subscriber('is_finished', Bool ,self.finishCallback) 
        self.check_sub = rospy.Subscriber('check', Int8 ,self.checkCallback) 
        self.error_sub = rospy.Subscriber('error', Bool ,self.errorCallback) 

        self.sub = rospy.Subscriber('camera/color/camera_info', CameraInfo ,self.cameraCallback) 
        
        
        self.finish = False # is_finished?
        self.check = 0 # new!! checking joint state
        self.camera_check = False # new!! checking camera topic state
        self.camera_info = UInt32() # new!! checking camera topic state
        self.error = False



    def checkCallback(self, msg):
        self.check = msg.data

    def finishCallback(self, msg):
        self.finish = msg.data

    def errorCallback(self, msg):
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
    try:
        rospy.init_node('tcpserver',anonymous=0)
        tcp_server = TCPServer()
        serverSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        serverSock.bind((tcp_server.HOST,tcp_server.PORT))
        serverSock.listen(5)
        print 'TCP Server Activate'

        # connectionSock,addr=serverSock.accept() #waiting connect  # always server connection 
        # print 'TCP Server Connected'
        
        while not rospy.is_shutdown():
            try :
                connectionSock,addr=serverSock.accept() #waiting connect every time 
                print 'TCP Server Connected'
                # connectionSock.settimeout(5)
                buffer=connectionSock.recv(tcp_server.BUFFER)
                # print buf
                tcp_server.pub.publish(buffer)
                # tcp_server.finish = True
                while True :
                    # print connectionSock
                    if tcp_server.finish == True:
                        
                        if buffer =="0010": 
                            connectionSock.send('0011')
                            break
                        elif buffer == "0020":
                            connectionSock.send('0021')
                            break
                        elif buffer == "0030":
                            connectionSock.send('0031')
                            break
                        elif buffer == "0110":
                            connectionSock.send('0111')
                            break
                        elif buffer == "0120":
                            connectionSock.send('0121')
                            break
                        elif buffer == "0130":
                            connectionSock.send('0131')
                            break
                        elif buffer == "0140":
                            connectionSock.send('0141')
                            break
                        elif buffer == "0210":
                            connectionSock.send('0211')
                            break
                        elif buffer == "0220":
                            connectionSock.send('0221')
                            break
                           
                        # Add checking joint state Protocol
                        elif buffer == "1300":  
                            # sleep(1) #sleep when code is not working  
                            if tcp_server.check == 0: # error code 
                                connectionSock.send('1320')
                                break
                            elif tcp_server.check == 1: # init pose a
                                connectionSock.send('1301')
                                break
                            elif tcp_server.check == 2: # init pose b
                                connectionSock.send('1302')
                                break
                            elif tcp_server.check == 3: # init pose c
                                connectionSock.send('1303')
                                break
                            elif tcp_server.check == 4: # ready pose 0
                                connectionSock.send('1311')
                                break
                            elif tcp_server.check == 5: # ready pose 90
                                connectionSock.send('1312')
                                break
                            elif tcp_server.check == 6: # ready pose 180
                                connectionSock.send('1313')
                                break
                            elif tcp_server.check == 7: # ready pose 270
                                connectionSock.send('1314')
                                break
                            else: # protocol error 
                                connectionSock.send('1321')
                                break
                        elif buffer =="1100": # server health check 
                            connectionSock.send('1101')
                            break
                        elif buffer =="1200": # camera health check 
                            if tcp_server.camera_health_check() == True:
                                connectionSock.send('1201')
                                break
                            else:
                                connectionSock.send('1202')
                                break

                        else:
                            # connectionSock.send('0321')

                            pass
                    
                    else:
                        pass
            except socket.timeout:
                print 'time out'
            
            
            tcp_server.RATE.sleep()
       
    except Exception as e:
        print "exit"
        # connectionSock.close()
        rospy.logerr(e)
    finally:
        pass