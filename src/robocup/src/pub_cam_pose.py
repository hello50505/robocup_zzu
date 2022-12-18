#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import sys
import std_msgs.msg
import tf2_ros



if __name__ == "__main__":

    rospy.init_node('pub_cam_pose')
    rate=rospy.Rate(30)

    drone_name=rospy.get_param("~drone_name")
    drone_id= str(rospy.get_param("~drone_id"))

    vehicle_type =drone_name
    vehicle_id = drone_id

    
    cam_pose_pub = rospy.Publisher('cam_pose', PoseStamped, queue_size=1)
    cam_pose = PoseStamped()
    gazeboLinkstate = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
  

    while not rospy.is_shutdown():
        try:
            response = gazeboLinkstate(vehicle_type+'_'+vehicle_id+'::cgo3_camera_link','ground_plane::link')
        except:
            print("Gazebo model state service call failed")
        cam_pose.header.frame_id="map"
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.pose = response.link_state.pose
        cam_pose_pub.publish(cam_pose)
           



        rate.sleep()



















