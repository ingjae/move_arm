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
        self.sub = rospy.Subscriber('is_finished', Bool ,self.finishCallback) 
        self.sub = rospy.Subscriber('check', Int ,self.checkCallback) 
        self.finish = False # is_finished?
        self.check = 0 # new!! checking joint state


    def checkCallback(self, msg):
        self.check = msg.data

    def finishCallback(self, msg):
        self.finish = msg.data
        


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
                tcp_server.finish = False
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
                elif buffer == "0300":  
                    # sleep(1) #sleep when code is not working  
                    if tcp_server.check == 0: # error code 
                        connectionSock.send('0320')
                        tcp_server.finish = 0
                    elif tcp_server.check == 1: # init pose a
                        connectionSock.send('0301')
                        tcp_server.finish = 0
                    elif tcp_server.check == 2: # init pose b
                        connectionSock.send('0302')
                        tcp_server.finish = 0
                    elif tcp_server.check == 3: # init pose c
                        connectionSock.send('0303')
                        tcp_server.finish = 0
                    elif tcp_server.check == 4: # ready pose 0
                        connectionSock.send('0311')
                        tcp_server.finish = 0
                    elif tcp_server.check == 5: # ready pose 90
                        connectionSock.send('0312')
                        tcp_server.finish = 0
                    elif tcp_server.check == 6: # ready pose 180
                        connectionSock.send('0313')
                        tcp_server.finish = 0
                    elif tcp_server.check == 7: # ready pose 270
                        connectionSock.send('0314')
                        tcp_server.finish = 0
                    else: # protocol error 
                        connectionSock.send('0321')
                        tcp_server.finish = 0
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