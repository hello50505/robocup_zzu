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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
### self msg
from robocup.msg import goal_distance,goal_xy_in_picture
### ros pkg
import tf
import numpy as np #numpy 比 c++ 的eigen库还要快！！！！！！！！
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import tf2_ros
from  geometry_msgs.msg  import PointStamped,Pose2D


goal_id=0
actor_info_msg=ActorInfo()
last_actor_info_msg=ActorInfo()

actor_id_dict = {0:'green', 1:'blue', 2:'brown', 3:'white', 4:'red'}




#no use
# def point_camera_sub_callback(data):
#     global goal_id
#     listener = tf.TransformListener()
#     listener.waitForTransform(drone_name+"_"+drone_id+"/camera_link", drone_name+"_"+drone_id+"/gazebo", rospy.Time(0),rospy.Duration(10.0))
    
#     #实际点
#     actual_point=PointStamped()
#     actual_point.header.frame_id = drone_name+"_"+drone_id+"/camera_link"
#     actual_point.header.stamp =rospy.Time(0)
    
#     actual_point.point.x=data.point.x
#     actual_point.point.y=data.point.y
#     actual_point.point.z=data.point.z
#     actor_point=listener.transformPoint(drone_name+"_"+drone_id+"/gazebo",actual_point)
    
#     #无人机跟踪目标点，比实际点离无人机更近
#     sim_point=PointStamped()
#     sim_point.header.frame_id = drone_name+"_"+drone_id+"/camera_link"
#     sim_point.header.stamp =rospy.Time(0)
    
#     sim_point.point.x=data.point.x
#     sim_point.point.y=data.point.y
#     sim_point.point.z=data.point.z-2.5
#     track_point=listener.transformPoint(drone_name+"_"+drone_id+"/gazebo",sim_point)

#     print(actor_id_dict[int(goal_id)] )
#     actor_info_msg.cls=actor_id_dict[int(goal_id)] 
#     actor_info_msg.x=actor_point.point.x
#     actor_info_msg.y=actor_point.point.y
#     actor_pub= rospy.Publisher("/actor_"+actor_info_msg.cls+"_info",ActorInfo,queue_size=1)
#     if(   (last_actor_info_msg.x-actor_info_msg.x )**2  +   (last_actor_info_msg.y-actor_info_msg.y )**2  )< 5.0 :
#         actor_pub.publish(actor_info_msg)

    
    # last_actor_info_msg=actor_info_msg

    # track_point_pub=rospy.Publisher("track_point_xy",PoseStamped,queue_size=1)
    # track_point_pub.publish(track_point)

    # rate.sleep()




#use now
def track_back_callback(point_gazebo_camera_msg):
    global goal_id
    listener = tf.TransformListener()
    print("wait")
    listener.waitForTransform(drone_name+"_"+drone_id+"/gazebo_camera_link", drone_name+"_"+drone_id+"/gazebo", rospy.Time(0),rospy.Duration(10.0))
    
    #实际点
    actual_point=PointStamped()
    actual_point.header.frame_id = drone_name+"_"+drone_id+"/gazebo_camera_link"
    actual_point.header.stamp =rospy.Time(0)
    
    actual_point.point.x=point_gazebo_camera_msg.point.x
    actual_point.point.y=point_gazebo_camera_msg.point.y
    actual_point.point.z=point_gazebo_camera_msg.point.z
    actor_point=listener.transformPoint(drone_name+"_"+drone_id+"/gazebo",actual_point)
    
    #无人机跟踪目标点，比实际点离无人机更近
    sim_point=PointStamped()
    sim_point.header.frame_id = drone_name+"_"+drone_id+"/gazebo_camera_link"
    sim_point.header.stamp =rospy.Time(0)
    
    sim_point.point.x=point_gazebo_camera_msg.point.x 
    sim_point.point.y=point_gazebo_camera_msg.point.y
    sim_point.point.z=point_gazebo_camera_msg.point.z
    track_point=listener.transformPoint(drone_name+"_"+drone_id+"/gazebo",sim_point)

    print(actor_id_dict[int(goal_id)] )
    actor_info_msg.cls=actor_id_dict[int(goal_id)] 
    actor_info_msg.x=actor_point.point.x
    actor_info_msg.y=actor_point.point.y
    actor_pub= rospy.Publisher("/actor_"+actor_info_msg.cls+"_info",ActorInfo,queue_size=1)

    actor_pub.publish(actor_info_msg)

    
    track_point_pub=rospy.Publisher("track_point_xy",PointStamped,queue_size=1)
    track_point.header.stamp=rospy.Time.now()
    track_point_pub.publish(track_point)

    rate.sleep()
    



def max_goal_distance_sub_callback(msg):
    global goal_id
    goal_id=msg.goal_id

if __name__ == '__main__':
    rospy.init_node("transfer_back_tf")
    drone_name=rospy.get_param("~drone_name")
    drone_id= str(rospy.get_param("~drone_id"))
    
    print("transfer_back_tf")
    max_goal_distance_sub=rospy.Subscriber("max_goal_distance",goal_distance,queue_size=1,callback=max_goal_distance_sub_callback)
    
   
    # point_camera_sub=rospy.Subscriber("point_camera_frame",PointStamped,queue_size=1,callback=point_camera_sub_callback)

   
    # camera_pose_sub = message_filters.Subscriber("/xtdrone/"+drone_name+"_"+drone_id+"/cam_pose", PoseStamped)
    # goal_distance_sub = message_filters.Subscriber("max_goal_distance", goal_distance)
    # point_gazebo_camera_sub=message_filters.Subscriber("point_gazebo_camera_frame",PointStamped)
    # track_back = message_filters.ApproximateTimeSynchronizer([point_gazebo_camera_sub], 1, 50, allow_headerless=True)#时间辍同步
    # track_back.registerCallback(track_back_callback)  
   
    point_gazebo_camera_sub=rospy.Subscriber("point_gazebo_camera_frame",PointStamped,callback=track_back_callback,queue_size=1)

 
    
    rate = rospy.Rate(200)
    rospy.spin()


