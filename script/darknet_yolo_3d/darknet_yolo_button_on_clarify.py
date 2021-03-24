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
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# from gb_detection_3d_msgs.msg import BoundingBoxes3d
# from gb_detection_3d_msgs.msg import BoundingBox3d
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


target_point = Bool()
target_point.data = False
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

    button_point_pub = rospy.Publisher("/button_on_target_point", Bool, queue_size=20)
    box_data = data
    # print (bounding_boxes_3d)
    boxes_count = len(box_data.bounding_boxes)
    # print (boxes_count)

    for count in range (boxes_count):
        temp_class = box_data.bounding_boxes[count].Class
        selected_button = str(rospy.get_param('button')) + "_on"
        if (selected_button == temp_class) :
            target_button = count
            break
        else:   
            target_button = -1

    if((target_button != -1) and (len(box_data.bounding_boxes) >= target_button)):
        target_point.data = True
        button_point_pub.publish(target_point)

    else:
        target_point.data = False
        button_point_pub.publish(target_point)
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