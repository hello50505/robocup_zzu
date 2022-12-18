#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy

import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

####ros msg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo,Imu
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from robocup.msg import goal_distance,goal_xy_in_picture,BoundingBox
pub_msg=goal_distance()

def color_depth_xy_goal_callback(data1,data2,data3):
    global pub_msg
    bridge = CvBridge()
    #color_image = bridge.imgmsg_to_cv2(data1, 'bgr8')
    #depth_image = bridge.imgmsg_to_cv2(data2, '16UC1')
    depth_image = bridge.imgmsg_to_cv2(data2, '32FC1')
    depth_image = np.nan_to_num(depth_image)
    # cv2.imshow('color_image',color_image)
    # cv2.waitKey(1000)
    c_x = data3.x
    c_y = data3.y
    #real_z = depth_image[c_y, c_x]  
    real_z_cache=[]
    

    real_z_1 = depth_image[c_y+1, c_x+1]
    real_z_2 = depth_image[c_y, c_x+1]
    real_z_3= depth_image[c_y-1, c_x+1]
    real_z_4= depth_image[c_y, c_x+1]
    real_z_5=depth_image[c_y-1, c_x]
    real_z_6=depth_image[c_y-1, c_x-1]
    real_z_7=depth_image[c_y, c_x-1]
    real_z_8=depth_image[c_y+1, c_x-1]
    real_z_9=depth_image[c_y, c_x]
   
    real_z_cache.append(real_z_1)
    real_z_cache.append(real_z_2)
    real_z_cache.append(real_z_3)
    real_z_cache.append(real_z_4)
    real_z_cache.append(real_z_5)
    real_z_cache.append(real_z_6)
    real_z_cache.append(real_z_7)
    real_z_cache.append(real_z_8)
    real_z_cache.append(real_z_9)
    avg_z=  sum(real_z_cache) /len(real_z_cache)

    remove_num=[]
    for i in range( len(real_z_cache)):
        if abs(avg_z-real_z_cache[i])>1.5:
            remove_num.append(real_z_cache[i])
    for i in range (len(remove_num)):
        real_z_cache.remove(remove_num[i])
    if (len(real_z_cache)>0):
        avg_z=sum(real_z_cache)/len(real_z_cache)
    real_z=avg_z
    

    real_x = (c_x- ppx) / fx * real_z*1000   #
    real_y = (c_y - ppy) / fy * real_z*1000 # real_x 和real_y的数据不知道准不准确
    # rospy.loginfo("potion:x=%f,y=%f,z=%f",real_x,real_y,real_z) #输出图像中心点在相机坐标系下的x,y,z
    rospy.loginfo("z=%f",real_z)
    pub_msg.distance=real_z
    pub_msg.goal_id = data3.goal_Id
    pub_msg.x_image_frame= data3.x
    pub_msg.y_image_frame= data3.y
    pub_msg.xmin=data3.xmin
    pub_msg.xmax= data3.xmax
    pub_msg.ymax= data3.ymax
    pub_msg.ymin= data3.ymin
    goal_dis_pub.publish(pub_msg)
    rate.sleep()

if __name__ == '__main__':
    global fx, fy, ppx, ppy #相机内参
    fx = 554.254691191187
    fy = 554.254691191187
    ppx = 320.5
    ppy = 240.5

    rospy.init_node("get_image_depth",anonymous=True)
    # vehicel_name = rospy.get_param("drone_name")
    # vehicel_id = str(rospy.get_param("drone_id"))
    color_sub = message_filters.Subscriber("realsense/depth_camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("realsense/depth_camera/depth/image_raw", Image)
    xy_sub = message_filters.Subscriber("bounding_box", BoundingBox)
    color_depth = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub,xy_sub], 1, 1, allow_headerless=True)#时间辍同步  
    #color_depth = message_filters.ApproximateTimeSynchronizer([xy_sub], 1, 1, allow_headerless=True)#时间辍同步
    color_depth.registerCallback(color_depth_xy_goal_callback)  
    #同时订阅三个话题，并利用message_filters实现话题同步，共同调用callback
    goal_dis_pub= rospy.Publisher("max_goal_distance",goal_distance,queue_size=1)
    rate = rospy.Rate(200)
    
    rospy.spin()

    # while not rospy.is_shutdown():
         
        
