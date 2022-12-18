//c++
#include <iostream>
#include<string>

#include <Eigen/Eigen>

//ros 
#include <ros/ros.h>
#include<robocup/goal_xy_in_picture.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<quadrotor_msgs/TakeoffLand.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
using namespace Eigen;




class logic
{
private:
    /* data */
public:
    logic(/* args */);
    ~logic();
    void takeoff();



};

logic::logic(/* args */)
{
}

logic::~logic()
{
}
