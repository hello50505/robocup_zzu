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
from geometry_msgs.msg  import PointStamped
import sys

# point_camera_frame_pub= rospy.Publisher("point_camera_frame",PointStamped,queue_size=1) 

point_gazebo_camera_frame_pub= rospy.Publisher("point_gazebo_camera_frame",PointStamped,queue_size=1)    

rospy.init_node("pub_tf")

def ID_distance_odom_callback(data1,gazebo_camera_pose_msg):
    broadcaster = tf2_ros.TransformBroadcaster()
     #         4-2.创建 广播的数据(通过 data 设置)
    tfs6 = TransformStamped()
    tfs6.header.frame_id = drone_name+"_"+drone_id+"/gazebo"
 
    tfs6.header.stamp = rospy.Time.now()
    tfs6.child_frame_id = drone_name+"_"+drone_id+"/gazebo_camera_link"

    tfs6.transform.translation.x = gazebo_camera_pose_msg.pose.position.x
    tfs6.transform.translation.y = gazebo_camera_pose_msg.pose.position.y
    tfs6.transform.translation.z = gazebo_camera_pose_msg.pose.position.z

    tfs6.transform.rotation.x = gazebo_camera_pose_msg.pose.orientation.x
    tfs6.transform.rotation.y = gazebo_camera_pose_msg.pose.orientation.y
    tfs6.transform.rotation.z =gazebo_camera_pose_msg.pose.orientation.z
    tfs6.transform.rotation.w = gazebo_camera_pose_msg.pose.orientation.w
   #         4-3.广播器发布数据
    broadcaster.sendTransform(tfs6)





    #         4-2.创建 广播的数据(通过 data 设置)
#     tfs0 = TransformStamped()
#     tfs0.header.frame_id = drone_name+"_"+drone_id+"/gazebo"
#     tfs0.header.stamp = rospy.Time.now()
#     tfs0.child_frame_id = drone_name+"_"+drone_id+"/odom"

#     tfs0.transform.translation.x = 3.0
#     tfs0.transform.translation.y = -3.0
#     tfs0.transform.translation.z = 0.0 #data.pose.position.z #data.pose.position.z
#     tfs0.transform.rotation.x = 0.0
#     tfs0.transform.rotation.y = 0.0
#     tfs0.transform.rotation.z =0.0
#     tfs0.transform.rotation.w = 1.0
#    #         4-3.广播器发布数据
#     broadcaster.sendTransform(tfs0)

#     broadcaster = tf2_ros.TransformBroadcaster()
#     #         4-2.创建 广播的数据(通过 data 设置)
#     tfs = TransformStamped()
#     tfs.header.frame_id = drone_name+"_"+drone_id+"/odom"
#     tfs.header.stamp = rospy.Time.now()
#     tfs.child_frame_id = drone_name+"_"+drone_id+"/base_link"

#     tfs.transform.translation.x = data2.pose.pose.position.x
#     tfs.transform.translation.y = data2.pose.pose.position.y
#     tfs.transform.translation.z = data2.pose.pose.position.z#data.pose.position.z #data.pose.position.z
#     tfs.transform.rotation.x = data2.pose.pose.orientation.x
#     tfs.transform.rotation.y = data2.pose.pose.orientation.y
#     tfs.transform.rotation.z =data2.pose.pose.orientation.z
#     tfs.transform.rotation.w = data2.pose.pose.orientation.w
#    #         4-3.广播器发布数据
#     broadcaster.sendTransform(tfs)
    


    
#     quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)

#     tfs2 = TransformStamped()
#     tfs2.header.frame_id = drone_name+"_"+drone_id+"/base_link"
#     tfs2.header.stamp = rospy.Time.now()
#     tfs2.child_frame_id = drone_name+"_"+drone_id+"/camera_link"

#     tfs2.transform.translation.x = 0
#     tfs2.transform.translation.y = 0
#     tfs2.transform.translation.z = 0#data.pose.position.z #data.pose.position.z
#     tfs2.transform.rotation.x = quaternion[0]
#     tfs2.transform.rotation.y = quaternion[1]
#     tfs2.transform.rotation.z =quaternion[2]
#     tfs2.transform.rotation.w = quaternion[3]
    
#    #         4-3.广播器发布数据

#     broadcaster.sendTransform(tfs2)


  

    
    

    # #camera_link
    # point_camera_frame=PointStamped()
    # point_camera_frame.header.stamp=rospy.Time.now()
    # point_camera_frame.point.x=x_camera_frame
    # point_camera_frame.point.y=y_camera_frame
    # point_camera_frame.point.z=data1.distance
    # if(data1.distance>0.02):
    #     print("x_camera_frame:",x_camera_frame)
    #     print("y_camera_frame:",y_camera_frame)
    #     print("distance:",data1.distance)
    #     point_camera_frame_pub.publish(point_camera_frame)



    fx=554.254691191187
    fy=554.254691191187
    cx=320.5
    cy=240.5
    
    x_camera_frame = data1.distance*(data1.x_image_frame-320.5)/fx
    y_camera_frame =  data1.distance*(data1.y_image_frame-240.5)/fy

    # gazebo_camera_link
    point_gazebo_camera_frame=PointStamped()
    point_gazebo_camera_frame.header.stamp=rospy.Time.now()
    point_gazebo_camera_frame.point.x=data1.distance
    point_gazebo_camera_frame.point.y=-x_camera_frame
    point_gazebo_camera_frame.point.z=-y_camera_frame
    if(data1.distance>0.02):
        print("point_gazebo_camera_frame.point.x:",point_gazebo_camera_frame.point.x)
        print("point_gazebo_camera_frame.point.y:",point_gazebo_camera_frame.point.y)
        print("point_gazebo_camera_frame.point.z:",point_gazebo_camera_frame.point.z)
        point_gazebo_camera_frame_pub.publish(point_gazebo_camera_frame)
    rate.sleep()
    





broadcaster = tf2_ros.TransformBroadcaster()
def gazebo_camera_callback (gazebo_camera_pose_msg):
 
     #         4-2.创建 广播的数据(通过 data 设置)
    tfs6 = TransformStamped()
    tfs6.header.frame_id = drone_name+"_"+drone_id+"/gazebo"
 
    tfs6.header.stamp = rospy.Time.now()
    tfs6.child_frame_id = drone_name+"_"+drone_id+"/gazebo_camera_link"

    tfs6.transform.translation.x = gazebo_camera_pose_msg.pose.position.x
    tfs6.transform.translation.y = gazebo_camera_pose_msg.pose.position.y
    tfs6.transform.translation.z = gazebo_camera_pose_msg.pose.position.z

    tfs6.transform.rotation.x = gazebo_camera_pose_msg.pose.orientation.x
    tfs6.transform.rotation.y = gazebo_camera_pose_msg.pose.orientation.y
    tfs6.transform.rotation.z =gazebo_camera_pose_msg.pose.orientation.z
    tfs6.transform.rotation.w = gazebo_camera_pose_msg.pose.orientation.w
   #         4-3.广播器发布数据
    broadcaster.sendTransform(tfs6)




def  ID_distance_callback(data1):
    fx=554.254691191187
    fy=554.254691191187
    cx=320.5
    cy=240.5
    
    x_camera_frame = data1.distance*(data1.x_image_frame-320.5)/fx
    y_camera_frame =  data1.distance*(data1.y_image_frame-240.5)/fy

    # gazebo_camera_link
    point_gazebo_camera_frame=PointStamped()
    point_gazebo_camera_frame.header.stamp=rospy.Time.now()
    point_gazebo_camera_frame.point.x=data1.distance
    point_gazebo_camera_frame.point.y=-x_camera_frame
    point_gazebo_camera_frame.point.z=-y_camera_frame
    if(data1.distance>0.02):
        print("point_gazebo_camera_frame.point.x:",point_gazebo_camera_frame.point.x)
        print("point_gazebo_camera_frame.point.y:",point_gazebo_camera_frame.point.y)
        print("point_gazebo_camera_frame.point.z:",point_gazebo_camera_frame.point.z)
        point_gazebo_camera_frame_pub.publish(point_gazebo_camera_frame)
    rate.sleep()

 







    
if __name__ == '__main__':

    drone_name=rospy.get_param("~drone_name")
    drone_id= str(rospy.get_param("~drone_id"))
    # drone_name=sys.argv[0]
    print(drone_name)
    
    
    rate = rospy.Rate(200) # 100hz

    # ID_distance_sub = message_filters.Subscriber("max_goal_distance", goal_distance)
    # gazebo_camera_sub = message_filters.Subscriber( "/"+drone_name+"_"+drone_id+"/cam_pose", PoseStamped)
    #a = message_filters.ApproximateTimeSynchronizer([ID_distance_sub,gazebo_camera_sub], 1, 1, allow_headerless=True)#允许接收没有时间辍的消息 allow_headerless=True
   # a.registerCallback(ID_distance_odom_callback)  


    ID_distance_sub=rospy.Subscriber("max_goal_distance", goal_distance,ID_distance_callback,queue_size=1)
    gazebo_camera_sub=rospy.Subscriber("/"+drone_name+"_"+drone_id+"/cam_pose", PoseStamped,gazebo_camera_callback,queue_size=1)
    rospy.spin()

    # while not rospy.is_shutdown():
         
        
