#!/usr/bin/env python
#-*- coding:utf-8 -*-


# subscribe
# 카메라 링크의 위치 (camera optical frame)
# 3d boundig box의 정보
 
# publish
# end point 가 이동할 object의 위치 좌표 

# parameter
# 파라미터로 몇층인지 받아오기

# 일단 1층을 타겟으로 설정
import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# from gb_detection_3d_msgs.msg import BoundingBoxes3d
# from gb_detection_3d_msgs.msg import BoundingBox3d
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


target_point = Vector3()
target_point.x = 0.0
target_point.y = 0.0
target_point.z = 0.0
target_button = -1

def init_node():
    rospy.init_node('button_point_publisher')
    bounding_boxes_3d_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, cb_bounding_boxes_3d)
    
    rospy.set_param('button',-1) # 아무것도 계산하지 않음
    # rospy.set_param('button',12) # 테스트
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        rate.sleep


def cb_bounding_boxes_3d(data):
    global target_button
    global target_point

    button_point_pub = rospy.Publisher("/button_target_point", Vector3, queue_size=20)
    box_data = data
    # print (bounding_boxes_3d)
    boxes_count = len(box_data.bounding_boxes)
    # print (boxes_count)

    for count in range (boxes_count):
        temp_class = box_data.bounding_boxes[count].Class
        # print(temp_class)
        selected_button = rospy.get_param('button')
        if (selected_button == "open" and temp_class == "open"):
            target_button = count
            break
        elif (selected_button == "close" and temp_class == "close"):
            target_button = count
            break
        elif (selected_button == "b1F" and temp_class == "b1"):
            target_button = count
            break   
        elif (selected_button == "1f" and temp_class == "1f"):
            target_button = count
            break
        elif (selected_button == "2f" and temp_class == "2f"):
            target_button = count
            break
        elif (selected_button ==  "3f" and temp_class == "3f"):
            target_button = count
            break
        elif (selected_button ==  "4f" and temp_class == "4f"):
            target_button = count
            break
        elif (selected_button ==  "5f" and temp_class == "5f"):
            target_button = count
            break
        elif (selected_button ==  "6f" and temp_class == "6f"):
            target_button = count
            break
        elif (selected_button ==  "up" and temp_class == "up"):
            target_button = count
            break
        elif (selected_button ==  "down" and temp_class == "down"):
            target_button = count
            break
        else:   
            target_button = -1

    # print(target_button)
    if((target_button != -1) and (len(box_data.bounding_boxes) >= target_button)):
        # print (box_data.bounding_boxes,target_button)
        target_x_min = box_data.bounding_boxes[target_button].xmin
        target_x_max = box_data.bounding_boxes[target_button].xmax
        target_y_min = box_data.bounding_boxes[target_button].ymin
        target_y_max = box_data.bounding_boxes[target_button].ymax
        target_z_min = box_data.bounding_boxes[target_button].zmin
        target_z_max = box_data.bounding_boxes[target_button].zmax

        float(target_x_min) 
        float(target_x_max) 
        float(target_y_min) 
        float(target_y_max) 
        float(target_z_min) 
        float(target_z_max) 
        target_center_x = (target_x_min+target_x_max)/2
        target_center_y = (target_y_min+target_y_max)/2
        target_center_z = (target_z_min+target_z_max)/2

        target_point.x = target_center_x
        target_point.y = target_center_y
        target_point.z = target_center_z
        # print(target_point.x,target_point.y,target_point.z)
        button_point_pub.publish(target_point)
       
    else:
        pass    
    


if __name__=="__main__":    
    try:
        # print("point init")

        target_button = -1 
        init_node()
        
        
            
    except rospy.ROSInterruptException:
        pass

# 0 OPEN
# 1 OPEN_On
# 2 CLOSE
# 3 CLOSE_On
# 4 B1F
# 5 B1F_On
# 6 1F
# 7 1F_On
# 8 2F
# 9 2F_On
# 10 3F
# 11 3F_On
# 12 4F
# 13 4F_On
# 14 5F
# 15 5F_On
# 16 6F
# 17 6F_On
# 18 UP
# 19 UP_On
# 20 DOWN
# 21 DOWN_On
