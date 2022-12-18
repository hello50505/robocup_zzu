
#! /usr/bin/env python
import rospy

import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

####ros msg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo,Imu
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from robocup.msg import goal_distance,goal_xy_in_picture
from nav_msgs.msg import Odometry

##
import tf
import math
a=Odometry()
a.pose.pose
def odom_callback(data2):
    



    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data2.pose.pose.orientation.x, data2.pose.pose.orientation.y, data2.pose.pose.orientation.z, data2.pose.pose.orientation.w])
    # print("roll:%.2f  picth::%.2f  yaw::%.2f \r\n",roll,picth,yaw)

     #参考：https://zhuanlan.zhihu.com/p/50514528#:~:text=%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%AD%A6%E4%B8%AD%EF%BC%8C%E7%94%A8%E9%BD%90%E6%AC%A1%E7%9F%A9%E9%98%B5%EF%BC%884x4%EF%BC%89%E6%9D%A5%E7%BB%9F%E4%B8%80%E6%8F%8F%E8%BF%B0%E5%88%9A%E4%BD%93%E7%9A%84%E4%BD%8D%E7%BD%AE%E5%92%8C%E5%A7%BF%E6%80%81%EF%BC%8C%E5%A6%82%E4%B8%8B%E5%9B%BE%E3%80%82%20%E9%80%9A%E8%BF%87%E7%9F%A9%E9%98%B5%E7%9A%84%E6%AD%A3%E9%80%86%E5%8F%98%E6%8D%A2%E5%92%8C%E7%9F%A9%E9%98%B5%E7%9B%B8%E4%B9%98%E6%93%8D%E4%BD%9C%EF%BC%8C%E5%AE%9E%E7%8E%B0%E4%BD%8D%E5%A7%BF%E7%9A%84%E5%8F%98%E6%8D%A2%E3%80%82,%E5%85%B6%E4%B8%AD%EF%BC%8C%20%E5%89%8D%E9%9D%A2%E7%9A%843x3%E7%9F%A9%E9%98%B5%E4%BB%A3%E8%A1%A8%E5%88%9A%E4%BD%93%E7%9A%84%E5%A7%BF%E6%80%81%EF%BC%8C%E5%90%8E%E9%9D%A2%E7%9A%843x1%E7%9F%A9%E9%98%B5%E4%BB%A3%E8%A1%A8%E5%88%9A%E4%BD%93%E7%9A%84%E4%BD%8D%E7%BD%AE%20%E3%80%82
    #适用范围：右手系：https://zhuanlan.zhihu.com/p/369299767
    #         逆时针为正
    #world—>body只考虑yaw
    rotate_only_yaw_matrix=np.array(
        [[math.cos(yaw),math.cos(yaw+math.pi/2),0],
        [math.sin(yaw),math.sin(yaw+math.pi/2),0],
        [0            ,             0         ,1]]
    )
    #world—>body只考虑picth
    rotate_only_picth_matrix=np.array([
        [math.cos(pitch), 0         , math.sin(pitch)],
        [  0              , 1       , 0],
        [-math.sin(pitch)  ,    0   ,math.cos(pitch)]
    ])
    #world—>body只考虑roll
    rotate__only_roll_matrix=np.array([
        [1,      0         ,         0],
        [ 0,  math.cos(roll)   , - math.sin(roll)],
        [ 0 ,   math.sin(roll)  , math.cos(roll)]
    ])

    #world—>body考虑yaw，picth，roll
    rotate_all_matrix=np.matmul(rotate__only_roll_matrix,np.matmul(rotate_only_picth_matrix,rotate_only_yaw_matrix))
    rotate_all_matrix_reverse=rotate_all_matrix.T
    # print(rotate_all_matrix)
    # print(rotate_all_matrix[0,0])
    uav_position_matrix=np.array([data2.pose.pose.position.x,data2.pose.pose.position.y,data2.pose.pose.position.z]) #无人机位置
    # print(uav_position_matrix)
    trans_matrix_reverse=-np.matmul(rotate_all_matrix.T,uav_position_matrix) #平移逆矩阵，见参考网址，见参考网址
    # print(trans_matrix_reverse)
    # print(trans_matrix_reverse[0])
    rotate_trans_reverse=np.array([
        [rotate_all_matrix_reverse[0,0],rotate_all_matrix_reverse[0,1],rotate_all_matrix_reverse[0,2],trans_matrix_reverse[0]],
        [rotate_all_matrix_reverse[1,0],rotate_all_matrix_reverse[1,1],rotate_all_matrix_reverse[1,2],trans_matrix_reverse[1]],
        [rotate_all_matrix_reverse[2,0],rotate_all_matrix_reverse[2,1],rotate_all_matrix_reverse[2,2],trans_matrix_reverse[2]],
        [                             0,                            0,                             0,                      1]

    ]) #逆齐次变换矩阵

    print(rotate_trans_reverse)
if __name__ == '__main__':
    rospy.init_node("cal_negetive_or_posive")
    odom=rospy.Subscriber("mavros/local_position/odom",Odometry,callback=odom_callback,queue_size=1)
    rospy.spin()

    # while not rospy.is_shutdown():
         
        
