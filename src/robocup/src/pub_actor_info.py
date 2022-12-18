
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
from nav_msgs.msg import Odometry
### self msg
from robocup.msg import goal_distance,goal_xy_in_picture
### ros pkg
import tf
import numpy as np #numpy 比 c++ 的eigen库还要快！！！！！！！！
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
actor_info_msg=ActorInfo()
actor_id_dict = {0:'green', 1:'blue', 2:'brown', 3:'white', 4:'red'}

# a = np.array([[1,2,3,4],[5,6,7,8]])
# c = np.matmul(a, b)
def cal_actor_info(data1,data2):
    quat=[data2.pose.pose.orientation.x,data2.pose.pose.orientation.y,data2.pose.pose.orientation.z,data2.pose.pose.orientation.w]

    rotate=R.from_quat(quat)
    rotate_matrix=rotate.as_matrix()

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data2.pose.pose.orientation.x, data2.pose.pose.orientation.y, data2.pose.pose.orientation.z, data2.pose.pose.orientation.w])
    print("roll:%.2f  picth::%.2f  yaw::%.2f \r\n",roll,pitch,yaw)

    uav_position_matrix=np.array([data2.pose.pose.position.x,data2.pose.pose.position.y,data2.pose.pose.position.z,1]) #无人机位置

 ####   ##############1.无人机旋转矩阵
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
    # print(rotate_all_matrix)
    
    
    ######齐次矩阵求逆矩阵，参考：https://blog.csdn.net/weixin_37835423/article/details/112553339
 #####   #############1.2.world—>body的逆其次变换矩阵 也就是body->world的其次变换矩阵
    rotate_all_matrix_reverse=rotate_all_matrix.T #转置
    rotate_trans_matrix=np.array([
        [rotate_all_matrix[0,0],rotate_all_matrix[0,1],rotate_all_matrix[0,2],uav_position_matrix[0]],
        [rotate_all_matrix[1,0],rotate_all_matrix[1,1],rotate_all_matrix[1,2],uav_position_matrix[1]],
        [rotate_all_matrix[2,0],rotate_all_matrix[2,1],rotate_all_matrix[2,2],uav_position_matrix[2]],
        [                             0,                            0,                             0,                      1]
    ]) #齐次变换矩阵 
    # trans_matrix_reverse=np.matmul(rotate_all_matrix_reverse,uav_position_matrix) #平移逆矩阵，见参考网址
    # rotate_trans_reverse=np.array([
    #     [rotate_all_matrix_reverse[0,0],rotate_all_matrix_reverse[0,1],rotate_all_matrix_reverse[0,2],trans_matrix_reverse[0]],
    #     [rotate_all_matrix_reverse[1,0],rotate_all_matrix_reverse[1,1],rotate_all_matrix_reverse[1,2],trans_matrix_reverse[1]],
    #     [rotate_all_matrix_reverse[2,0],rotate_all_matrix_reverse[2,1],rotate_all_matrix_reverse[2,2],trans_matrix_reverse[2]],
    #     [                             0,                            0,                             0,                      1]
    # ]) #逆齐次变换矩阵
    # print("math.cos(yaw)-rotate_trans_matrix",math.cos(yaw)-rotate_trans_matrix[0,0])
    # print("rotate_trans_matrix\n",rotate_trans_matrix)
    # print("rotate_matrix-rotate_only_yaw_matrix",rotate_all_matrix-rotate_only_yaw_matrix)
    # print(np.matmul(rotate_trans_reverse,rotate_trans_matrix))

    # print("trans_matrix_reverse\n",trans_matrix_reverse)
    rotate_trans_reverse=np.linalg.inv(rotate_trans_matrix)
    print("np.matmul(rotate_trans_reverse,rotate_trans_matrix)\n",np.matmul(rotate_trans_reverse,rotate_trans_matrix))
### 像素坐标 ！= 相机坐标  
### 转换关系，参考资料:https://blog.csdn.net/qq_45088942/article/details/127030079?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-127030079-blog-99207102.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-127030079-blog-99207102.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=5
### 3. [像素坐标]=1/z[相机内参矩阵k][无人机到相机旋转矩阵q][无人机齐次变换矩阵][像素世界坐标]  #参考《slam十四讲》
    ##所以  [像素世界坐标]=z[齐次逆][q逆][相机内参矩阵k的逆矩阵][像素坐标] 

    k=np.array([
        [554.254691191187,0.0,              320.5],
        [0.0,           554.254691191187,240.5],
        [0.0,           0.0,                1.0]
    ])
    k_reverse=np.linalg.inv(k)
    # print(np.matmul(k_reverse,k))
    quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, math.pi/2)
    aa = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    # q=aa.rotation_matrix
    # q_reverse=np.linalg.inv(q)
    q_reverse=aa.rotation_matrix

    actor_postion_image_frame=np.array([data1.x_image_frame,data1.y_image_frame,1])#像素坐标系
    # print("image_frame:",actor_postion_image_frame)
    # print("distance",data1.distance)
####4.[相机坐标]=depth*[k_reverse][像素坐标]
    actor_position_camera_frame=data1.distance*np.matmul(k_reverse,actor_postion_image_frame)
    # print("camera_frame:",actor_position_camera_frame)
    actor_position_uav_frame=np.matmul(q_reverse,actor_position_camera_frame)
    qici_actor_position_uav_frame=np.array([
        [-actor_position_uav_frame[0],-actor_position_uav_frame[1],actor_position_uav_frame[2],1],
    ])
    # print("qici_actor_position_uav_frame:",qici_actor_position_uav_frame)

    # z=data1.distance
    # x=(data1.x_image_frame-320.5)/554.254*z
    # y=(data1.y_image_frame-240.5)/554.254*z
    # print("x:",x)
    # print("y",y)
 ####  5.[世界坐标]=[齐次逆变换矩阵][相机坐标] 
    
    print("rotate_trans_reverse\n",rotate_trans_reverse)
    print("qici_actor_position_uav_frame.T\n",qici_actor_position_uav_frame.T)
    actor_postion_world_frame=np.matmul(rotate_trans_reverse,qici_actor_position_uav_frame.T)
    
    print("world_frames\n",actor_postion_world_frame)

    return (actor_postion_world_frame[0],actor_postion_world_frame[1])




def ID_distance_odom_callback(data1,data2):

    
    

    actor_info_msg.cls=actor_id_dict[int(data1.goal_id)]
    
    
    (actor_info_msg.x,actor_info_msg.y) = cal_actor_info(data1,data2)#计算行人位置 

    actor_pub= rospy.Publisher("/actor_"+actor_info_msg.cls+"_info",ActorInfo,queue_size=1)
    actor_pub.publish(actor_info_msg)
    rate.sleep()

if __name__ == '__main__':
    


    rospy.init_node("pub_actor_info",anonymous=True)
    ID_distance_sub = message_filters.Subscriber("max_goal_distance", goal_distance)
    odom_sub = message_filters.Subscriber("mavros/local_position/odom", Odometry)

    a = message_filters.ApproximateTimeSynchronizer([ID_distance_sub, odom_sub], 1, 1, allow_headerless=True)#允许接收没有时间辍的消息 allow_headerless=True
    a.registerCallback(ID_distance_odom_callback)  
    #同订阅两个话题，并利用message_filters实现话题同步，共同调用callback
    
    rate = rospy.Rate(200)
    # while not rospy.is_shutdown():
    #     print("loop")
        
    #     rate.sleep()
    rospy.spin()
    

    # while not rospy.is_shutdown():
         
        
