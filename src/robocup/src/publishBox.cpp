#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>
//#include <publishBox/BoundingBox.h>
#include<robocup/goal_xy_in_picture.h>
#include <iostream>

std::vector<int> xcenter;
std::vector<int> ycenter;
int box_count;
int biggestBox_mianji;
int maxposition;
int width_err;
int height_err;
int biggestBox_index;
//publishBox::BoundingBox box_msg;
robocup::goal_xy_in_picture box_msg;

int find_biggestbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr &a)
{
    box_count= end(a->bounding_boxes)-begin(a->bounding_boxes); // num of boxes
    // printf("cnt:--- %d  ---\r\n",cnt);
    int mianji[box_count];
    int x_lenth, y_lenth;
    for(int i=0;i<box_count;i++)
        {
        x_lenth=a->bounding_boxes[i].xmax-a->bounding_boxes[i].xmin;
        y_lenth=a->bounding_boxes[i].ymax-a->bounding_boxes[i].ymin;
        //   printf("width: %d   height: %d\r\n  ",x_lenth,y_lenth);
        mianji[i]=x_lenth*y_lenth;
        }
    int maxposition = std::max_element(mianji,mianji+box_count )-mianji;
    
    box_msg.x= a->bounding_boxes[maxposition].xmin + (a->bounding_boxes[maxposition].xmax-a->bounding_boxes[maxposition].xmin)/2;
    box_msg.y=a-> bounding_boxes[maxposition].ymin +( a->bounding_boxes[maxposition].ymax-a->bounding_boxes[maxposition].ymin)/2;
    biggestBox_mianji=(a->bounding_boxes[maxposition].ymax-a->bounding_boxes[maxposition].ymin)*(a->bounding_boxes[maxposition].xmax-a->bounding_boxes[maxposition].xmin);
    // printf("maxposition %d\r\f",maxposition);
    return maxposition;
}

void darknet_ros_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &a)
{
    biggestBox_index=find_biggestbox(a);
    //std::cout<<"index:"<<biggestBox_index<<std::endl;
    box_msg.goal_id = a->bounding_boxes[biggestBox_index].id;
    std::cout<<"id:"<<a->bounding_boxes[biggestBox_index].id<<std::endl;
    width_err=box_msg.x-320;
    height_err=box_msg.y-240;
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "publishBox");
    ros::NodeHandle nh;
    ros::Subscriber darknet_ros_sub;
    ros::Publisher box_pub;
    ros::Rate rate(20.0);
    darknet_ros_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>(
        "darknet_ros/bounding_boxes",10,darknet_ros_cb);
    box_pub = nh.advertise<robocup::goal_xy_in_picture>(
        "xy_goal",10);
    while(ros::ok())
    {
        box_pub.publish(box_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}