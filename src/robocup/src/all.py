import rospy

from quadrotor_msgs.msg import TakeoffLand

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np 



find_man = False
arrived_fixed_goal=False
goal_id=0


fixed_goal=np.array([[10,0],[10,10],[0,10],[0,0]])
track_goal=PoseStamped()
odom_msg_now= Odometry()

actor_id_dict = {0:'green', 1:'blue', 2:'brown', 3:'white', 4:'red'}





def takeoff():
    
    takeoff_msg=TakeoffLand()
    time_begin= rospy.get_time()
    while(rospy.get_time()<time_begin+5):

        takeoff_msg.takeoff_land_cmd=1
        takeoff_pub.publish(takeoff_msg)


def track_man():
    time_begin= rospy.get_time()
    while(rospy.get_time()<time_begin+15):
        convert_manPosition_to_goal()


def goal_sub_callback(goal_msg,odom_msg):
    global find_man,goal_id,odom_msg_now
    odom_msg_now=odom_msg

    find_man=true
   
    track_goal.pose.position.x=goal_msg.point.x
    track_goal.pose.position.y=goal_msg.point.y

    track_goal.pose.position.z=4.5

    track_goal.pose.orientation.w= odom_msg.pose.pose.orientation.w
    track_goal.pose.orientation.x= odom_msg.pose.pose.orientation.x
    track_goal.pose.orientation.y= odom_msg.pose.pose.orientation.y
    track_goal.pose.orientation.z= odom_msg.pose.pose.orientation.z





    


    


def cal_find_man():
    global find_man
    find_man =False
    # xy_sub = message_filters.Subscriber("xy_goal", goal_xy_in_picture)
    # actor_sub=  message_filters.Subscriber("/actor_"+actor_info_msg.cls+"_info",ActorInfo)
    odom_sub = message_filters.Subscriber("mavros/local_position/odom", Odometry)
    goal_sub = message_filters.Subscriber("track_point_xy", PointStamped)
    xy = message_filters.ApproximateTimeSynchronizer([goal_sub,odom_sub], 10, 1, allow_headerless=True)#时间辍同步
    xy.registerCallback(goal_sub_callback) 
    






def track_man():
    time_begin= rospy.get_time()
    
    rate = rospy.Rate(200)
    while(rospy.get_time()<time_begin+20):
        track_goal_pub.publish(track_goal)
        rate.sleep()

def cal_arrived(goal,odom)
    odom=odom_msg_now
  


if __name__ == '__main__':
    rospy.init_node("all")

    rospy.sleep(10.)#sleep 10s
    drone_name=rospy.get_param("~drone_name")
    drone_id= str(rospy.get_param("~drone_id"))

    takeoff_pub=rospy.Publisher("px4ctrl/takeoff_land",TakeoffLand,queue_size=1)
    track_goal_pub=rospy.Publisher("move_base/goal",PoseStamped,queue_size=1)

    fixed_goal_pub=rospy.Publisher("move_base/goal",PoseStamped,queue_size=1)
    takeoff()

    fixed_goal_msg=PoseStamped()

    while find_man!=True:
        for i in range(4):
            fixed_goal_msg.pose.position.x = fixed_goal[i][0]
            fixed_goal_msg.pose.position.y = fixed_goal[i][1]
            fixed_goal_msg.pose.position.z = 4.5
            fixed_goal_msg.pose.orientation.w = odom_msg_now.pose.pose.orientation.w
            fixed_goal_msg.pose.orientation.x =odom_msg_now.pose.pose.orientation.x
            fixed_goal_msg.pose.orientation.y =odom_msg_now.pose.pose.orientation.y
            fixed_goal_msg.pose.orientation.z =odom_msg_now.pose.pose.orientation.z
            while arrived_fixed_goal !=True:
                fixed_goal_pub.publish(fixed_goal_msg)
                rate.sleep()

    
   
    
  
    
    rate = rospy.Rate(200)
    rospy.spin()