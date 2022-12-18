#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>
#include <publishbox/BoundingBox.h>
#include <iostream>

ros::Time current_time;
ros::Time last_time;
std::vector<int> xcenter;
std::vector<int> ycenter;
int box_count;
int biggestBox_mianji;
int maxposition;
int width_err;
int height_err;
int biggestBox_index;
publishbox::BoundingBox box_msg;

    ros::Publisher box_pub;
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

box_msg.xmin = a->bounding_boxes[maxposition].xmin;
box_msg.xmax = a->bounding_boxes[maxposition].xmax;
box_msg.ymin = a->bounding_boxes[maxposition].ymin;
box_msg.ymax = a->bounding_boxes[maxposition].ymax;
box_msg.x= a->bounding_boxes[maxposition].xmin + (a->bounding_boxes[maxposition].xmax-a->bounding_boxes[maxposition].xmin)/2;
box_msg.y=a-> bounding_boxes[maxposition].ymin +( a->bounding_boxes[maxposition].ymax-a->bounding_boxes[maxposition].ymin)/2;
biggestBox_mianji=(a->bounding_boxes[maxposition].ymax-a->bounding_boxes[maxposition].ymin)*(a->bounding_boxes[maxposition].xmax-a->bounding_boxes[maxposition].xmin);
// printf("maxposition %d\r\f",maxposition);
return maxposition;
}

void darknet_ros_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &a)
{
    last_time = ros::Time::now();
    biggestBox_index=find_biggestbox(a);
    //std::cout<<"index:"<<biggestBox_index<<std::endl;
    box_msg.goal_Id = a->bounding_boxes[biggestBox_index].id;
    std::cout<<"id:"<<box_msg.goal_Id<<std::endl;
    width_err=box_msg.x-320;
    height_err=box_msg.y-240;
    box_pub.publish(box_msg);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "publishBox");
    ros::NodeHandle nh;
    ros::Subscriber darknet_ros_sub;

    ros::Rate rate(200.0);
    darknet_ros_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>(
        "darknet_ros/bounding_boxes",1,darknet_ros_cb);
    box_pub = nh.advertise<publishbox::BoundingBox>(
        "bounding_box",1);
    while(ros::ok())
    {
        // while(ros::Time::now()-last_time<ros::Duration(1.0))
        // {
            
            // rate.sleep();
        // }
        // printf("no human\n");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
