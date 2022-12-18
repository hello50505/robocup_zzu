#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import sys
import std_msgs.msg

from robocup.msg import goal_distance


gimbal_pitch_ = -45 #-15 #-60
gimbal_yaw_ = 0 #0.0
gimbal_roll_ = 0.0



def  xy_callback(xy_msg):
    print("gimbal   tracking")
    global gimbal_pitch_,gimbal_roll_,gimbal_yaw_,msg
    width_err=xy_msg.x_image_frame-320
    height_err=240-xy_msg.y_image_frame
    print(width_err)
    if abs(width_err)>100:
        gimbal_yaw_=gimbal_yaw_+0.01*width_err
    if abs(height_err)>100:
        gimbal_pitch_=gimbal_pitch_+0.01*height_err
    print(gimbal_pitch_)
    
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    msg.pitch = gimbal_pitch_
    msg.roll = gimbal_roll_
    msg.yaw = gimbal_yaw_
    mountCnt.publish(msg)

    rate.sleep()








if __name__ == "__main__":

    
    rospy.init_node('gimbal_track')



    xy_sub=rospy.Subscriber("max_goal_distance",goal_distance,xy_callback,queue_size=1)
    drone_name=rospy.get_param("~drone_name")
    drone_id= str(rospy.get_param("~drone_id"))

    vehicle_type =drone_name
    vehicle_id = drone_id

    mountCnt = rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=1)
    
    rate=rospy.Rate(50)
  

    cam_pose_pub = rospy.Publisher('cam_pose', PoseStamped, queue_size=1)
    cam_pose = PoseStamped()


    print(vehicle_type+'_'+vehicle_id+': Gimbal control')
    # while not rospy.is_shutdown():
    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    msg.pitch = gimbal_pitch_
    msg.roll = gimbal_roll_
    msg.yaw = gimbal_yaw_
    mountCnt.publish(msg)
    rospy.spin()
