
// #include<logic.h>
//c++

#include <iostream>
#include<string>
#include <Eigen/Eigen>

//ros 
#include <ros/ros.h>

#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>

#include<std_msgs/Int16.h>
#include<std_msgs/String.h>

#include<mavros_msgs/MountControl.h>
#include<mavros_msgs/State.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<tf/transform_listener.h>
//self msg
#include<robocup/goal_xy_in_picture.h>
#include<quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include<robocup/pubgoal_bool.h>
#include<robocup/goal_distance.h>
#include<publishbox/BoundingBox.h>


using namespace message_filters;
using namespace Eigen;

geometry_msgs::PoseStamped  trans_pose_cam_to_world(double distance  );

class Pid
{
public:
double dirPosition;// desired speed 
double nowPosition;
double position_err;
double position_err_threshold;//误差阈值

double position_err_last;
double Kp, Ki, Kd;
double Integral;//累计误差
Pid(double dirPos, double position_err_threshold,
double Kp, double Ki, double Kd);
    int work();
double pid_cal(double nowPosition);
~Pid();

private:

};
Pid::Pid(double dirPos, double position_err_threshold,
double  Kp, double Ki, double Kd)
{
this->dirPosition = dirPos;
this->position_err_threshold = position_err_threshold;
this->Kp = Kp;
this->Ki = Ki;
this->Kd = Kd;
this->Integral = 0;
this->position_err_last = 0;
printf(" dirpos::%.2f\n", this->dirPosition);


}
double Pid::pid_cal(double nowPosition)
{

double output;
this->nowPosition = nowPosition;
this->position_err = this->dirPosition - this->nowPosition;
if (abs(this->position_err) < this->position_err_threshold)return 0;
else {
this->Integral += this->position_err;
output = this->Kp * this->position_err +
this->Ki * this->Integral +
this->Kd * (this->position_err_last - this->position_err);
this->position_err_last = this->position_err;
return output;
}
}


int width_err,height_err,area;
  
std::string drone_name ;
int drone_id ;
float max_err;

//sub msg
nav_msgs::Odometry now_odom;
geometry_msgs::PointStamped now_goal_point;
mavros_msgs::State now_state;
std::string  left_actors;


//pub msg
geometry_msgs::PoseStamped fixed_goal_msg;
geometry_msgs::PoseStamped move_goal_msg;
quadrotor_msgs::TakeoffLand takeOff_msg;
robocup::goal_distance box_msg;
mavros_msgs::MountControl mountcontrol_msg;
robocup::pubgoal_bool  pub_bool_msg;


// Eigen::Matrix<double, Dynamic, Dynamic>   fixed_goal;
Eigen::MatrixXd fixed_goal(4,2);
bool find_man=false;
bool arrived_fixed_goal=false,arrived_move_goal=false;	



    ros::Subscriber state_sub  , xy_sub;
    ros::Publisher fixed_goal_pub,move_goal_pub,takeOff_pub ,gimbal_track_mode_pub,gimbal_cmd_pub; //

	double  gimbal_pitch_=-15,gimbal_yaw_=0,gimbal_roll_=0;
    double  last_gimbal_pitch_=-15,last_gimbal_yaw_=0;     


void callback(const nav_msgs::Odometry::ConstPtr & odom_msg , const geometry_msgs::PointStamped::ConstPtr & track_goal_msg )
{
    now_odom= *odom_msg;
    now_goal_point=*track_goal_msg;
    ROS_WARN("receved_msg");
}




void odom_callback(const nav_msgs::Odometry::ConstPtr & odom_msg  )
{
    
    now_odom= *odom_msg;
    // ROS_WARN("receved_odom_msg");
}

void  state_callback(const mavros_msgs::State::ConstPtr & state_msg)
{
    now_state=*state_msg;

}

void xy_callback(const robocup::goal_distance::ConstPtr & goal_distance_msg){

ROS_INFO("find    man !!");
find_man=true;
  box_msg=*goal_distance_msg  ;
  width_err=box_msg.x_image_frame-320;
    height_err=240-box_msg.y_image_frame;
   area = (box_msg.xmax-box_msg.xmin  )   * (box_msg.ymax -box_msg.ymin);

}

void left_actors_callback(const std_msgs::String::ConstPtr  &msg )
{
    left_actors=msg->data;
}




void   hover()
{
    


    ROS_WARN("hover!");
      pub_bool_msg.gimbal_track_mode=true;
     gimbal_track_mode_pub.publish(pub_bool_msg);
  
//   ros::spinOnce();
    //   move_goal_msg.pose=now_odom.pose.pose;
    // move_goal_pub.publish(move_goal_msg);  


}



void gimbal_reset()
{
    ROS_WARN("gimbal  reseting!!!");
    mountcontrol_msg.header.stamp =ros::Time::now();
    mountcontrol_msg.header.frame_id = "map";
    mountcontrol_msg.mode = 2;
    mountcontrol_msg.pitch = -15;
    mountcontrol_msg.roll = 0;
    mountcontrol_msg.yaw = 0;
gimbal_cmd_pub.publish(mountcontrol_msg);
ros::Duration(0.2).sleep();
}

geometry_msgs::PoseStamped  trans_pose_cam_to_world(double distance  )
{
    ros::spinOnce();
    tf::TransformListener   listener;
    listener.waitForTransform(drone_name+"_"+std::to_string(drone_id) +"/gazebo_camera_link", drone_name+"_"+std::to_string(drone_id) +"/gazebo", ros::Time(0),ros::Duration(10.0));
geometry_msgs::PoseStamped  pose_in_cam_link;
   geometry_msgs::PoseStamped pose_in_world;
     pose_in_cam_link.header.frame_id = drone_name+"_"+std::to_string(drone_id)+"/gazebo_camera_link";
    pose_in_cam_link.header.stamp =ros::Time(0);
    pose_in_cam_link.pose.position.x=distance;
    ROS_WARN("transfering    point   ");

try
{
      listener.transformPose(drone_name+"_"+std::to_string(drone_id) +"/gazebo",pose_in_cam_link,pose_in_world);
      ROS_WARN("transfer    success   ");
}
catch( tf::TransformException &ex )
{

    ROS_ERROR("%s",ex.what());
    // ROS_BREAK();
    
}

 
    return pose_in_world;
}

void  avoid_too_high(){
    if(now_odom.pose.pose.position.z>4.50)
    {
      ROS_ERROR("too high  !   now height:  %f",now_odom.pose.pose.position.z);
        while(now_odom.pose.pose.position.z>3)
        {

              hover();
            ROS_ERROR("too high  !   now height:  %f",now_odom.pose.pose.position.z);
            
             ros::spinOnce();
            
        takeOff_msg.takeoff_land_cmd=2;
        takeOff_pub.publish(takeOff_msg);

        }
        if(now_odom.pose.pose.position.z<3)
        {
            arrived_fixed_goal=true;
        pub_bool_msg.gimbal_track_mode=false;
        gimbal_track_mode_pub.publish(pub_bool_msg);
        }
             
    }

}



void gimbal_track_start()
{
    ros::spinOnce();
  
    ROS_INFO("width_ERR : %d",width_err);
    // Pid gimbal_yaw_rate_pid(320, 0.3, 0.005, 0.005*0.05, 0);// 期望位置   误差阈值  kp  ki  kd  
    //  Pid gimbal_pitch_rate_pid(240, 0.3, 0.005, 0.005*0.05, 0);


    if  (abs(width_err)>10)
        gimbal_yaw_=gimbal_yaw_+0.05*width_err - 0.01*(gimbal_yaw_-last_gimbal_yaw_)  ;
        last_gimbal_yaw_=gimbal_yaw_;
    if ( abs(height_err)>10)
        gimbal_pitch_=gimbal_pitch_+0.05*height_err- 0.01 *( gimbal_pitch_-last_gimbal_pitch_  );
        last_gimbal_pitch_=gimbal_pitch_;
    ROS_INFO("now_gimbal_pitch : %d",gimbal_pitch_);
    
mountcontrol_msg.header.stamp =ros::Time::now();
mountcontrol_msg.header.frame_id = "map";
mountcontrol_msg.mode = 2;
mountcontrol_msg.pitch = gimbal_pitch_;
mountcontrol_msg.roll = gimbal_roll_;
mountcontrol_msg.yaw = gimbal_yaw_;
gimbal_cmd_pub.publish(mountcontrol_msg);

}








void cal_arrvied_fixed_goal()
{
    arrived_fixed_goal=false;
    // ROS_INFO("not_arrived_fixed_goal");
    if(abs(fixed_goal_msg.pose.position.x-now_odom.pose.pose.position.x)<max_err &&
    abs(fixed_goal_msg.pose.position.y-now_odom.pose.pose.position.y)<max_err &&
    abs(fixed_goal_msg.pose.position.z-now_odom.pose.pose.position.z)<max_err 
    )
    {
        arrived_fixed_goal=true;
        ROS_INFO("arrived_fixed_goal");

    }
}

void cal_arrvied_move_goal()
{
    arrived_move_goal=false;
    // ROS_INFO("not_arrived_fixed_goal");
    if(abs(move_goal_msg.pose.position.x-now_odom.pose.pose.position.x)<max_err &&
    abs(move_goal_msg.pose.position.y-now_odom.pose.pose.position.y)<max_err &&
    abs(move_goal_msg.pose.position.z-now_odom.pose.pose.position.z)<max_err 
    )
    {
        arrived_move_goal=true;
        ROS_INFO("arrived_fixed_goal");

    }
}



template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};







int main(int argc,char *argv[])
{
    ros::init(argc, argv, "logic");
          ros::NodeHandle nh;
    ros::Rate fixed_goal_rate(1);
    ros::Rate takeoff_rate(20);
    ros::Rate  mini_control_rate(200);
 printf("111");


   

    // nh.getParam("drone_name",drone_name);
    // ROS_INFO("221");
    // nh.getParam("drone_id",drone_id);
    // nh.getParam("max_err",max_err_str);
     read_essential_param(nh, "drone_name", drone_name);
      read_essential_param(nh, "drone_id", drone_id);
    read_essential_param(nh, "max_err", max_err);
    

    switch (drone_id)
    {
    case 0:
            fixed_goal <<-16,0,
                        -43,0,
                    -43, -44, 
                    -14,-43;
        break;
    case 1:
         fixed_goal << 44,0,
                        46,-45,
                    12, -26, 
                    18,-1;
      break;
    case 2:
         fixed_goal << 110,0.5,
                        120,22,
                    72, 44, 
                    40,23;
      break;
      case 3:
          fixed_goal << 49,-0.4,
                        49,-45,
                    122, -47, 
                    122,-0.7;
      break;
      case 4:
          fixed_goal << -6,10,
                        -15,43,
                    -44, 44, 
                    -39,11;
      break;
      case 5:
         fixed_goal << 44.7,2.0,
                        45,44,
                    -4, 39, 
                    -6,10;
      break;
    default:
     fixed_goal << 1,0,
     1,0,
     1,0,
     1,0;

        break;
    }




    std::cout<<max_err<<std::endl;  

    printf("%f",max_err);



    state_sub=nh.subscribe<mavros_msgs::State>("mavros/state",1,state_callback);
    xy_sub=nh.subscribe<robocup::goal_distance>("max_goal_distance",1,xy_callback);
    ros::Subscriber only_odom_sub=nh.subscribe<nav_msgs::Odometry>("odom_world",1,odom_callback);
    ros::Subscriber  left_actor_sub=nh.subscribe< std_msgs::String>("/left_actors",1,left_actors_callback);



    fixed_goal_pub=nh.advertise<geometry_msgs::PoseStamped> ("/typhoon_h480_"+ std::to_string(drone_id) +"_ego_planner_node/move_base_simple/goal",1);
    move_goal_pub= nh.advertise<geometry_msgs::PoseStamped> ("/typhoon_h480_"+ std::to_string(drone_id) +"_ego_planner_node/move_base_simple/goal",1);
    takeOff_pub= nh.advertise<quadrotor_msgs::TakeoffLand> ("px4ctrl/takeoff_land",1);
    // cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("planning/pos_cmd",1);
    gimbal_cmd_pub=nh.advertise<mavros_msgs::MountControl>("mavros/mount_control/command",1);
    gimbal_track_mode_pub=nh.advertise<robocup::pubgoal_bool>("gimble_track_mode",1);


//     message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom_world", 1);
//     message_filters::Subscriber<geometry_msgs::PointStamped> track_point_sub(nh, "track_point_xy", 1);
//   typedef sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PointStamped> MySyncPolicy;
//   // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//   Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), odom_sub, track_point_sub)
//   sync.registerCallback(boost::bind(&callback, _1, _2));
  
  


  auto begin_time1=ros::Time::now();  
  ROS_INFO("127");



  /*        takeoff */
    takeOff_msg.takeoff_land_cmd=1;

  while(now_state.armed==false or now_state.mode!="OFFBOARD" )
  {
        
        takeOff_pub.publish(takeOff_msg);
        ros::spinOnce();
        takeoff_rate.sleep();
  }


while(begin_time1.toSec()<0.1)
{
    begin_time1=ros::Time::now();
    ROS_INFO("reading time");
}

ROS_INFO("sleep 4s");
ros::Duration(5.0).sleep();
ROS_INFO("sleep over ");

ROS_INFO("134");

 find_man=false;
while(ros::ok())
{
    ros::spinOnce();
                for(int i=0;i<fixed_goal.rows();i++)
            {
                arrived_fixed_goal=false;
                fixed_goal_msg.pose.position.x= fixed_goal(i,0) ;//now_goal_point.point.x;
                fixed_goal_msg.pose.position.y= fixed_goal(i,1);
                fixed_goal_msg.pose.position.z=2.80;
                fixed_goal_msg.pose.orientation=now_odom.pose.pose.orientation;
                if(!arrived_fixed_goal && !find_man)
                {
                    gimbal_reset();
                    fixed_goal_pub.publish(fixed_goal_msg); 
                      
                        while(!arrived_fixed_goal && !find_man)
                        {
                            cal_arrvied_fixed_goal();
                            avoid_too_high();
                            ros::spinOnce();
                            mini_control_rate.sleep();
                        }
                }
                 if(arrived_fixed_goal  && !find_man){
                    ROS_INFO(" a  o    ,no  people---   +__+   ");
                            continue;
                        }
                
                 while(find_man )
            {   
                ROS_INFO("gimble  tracking  489");
                    find_man=false;
                    hover();
                    gimbal_track_start();
                    pub_bool_msg.gimbal_track_mode=true;
                    gimbal_track_mode_pub.publish(pub_bool_msg);
                    avoid_too_high();    
                        ros::spinOnce();
                        mini_control_rate.sleep();
                    if(!find_man)
                    {
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();

                        if(!find_man)
                        {
                         move_goal_msg= trans_pose_cam_to_world( 7.0);
                        gimbal_reset();
                        
                        arrived_move_goal=false;
                         pub_bool_msg.gimbal_track_mode=false;
                          gimbal_track_mode_pub.publish(pub_bool_msg);
                          ROS_WARN("gimbal   reseting  511");
                           ros::Duration(0.5).sleep();
                       move_goal_pub.publish(move_goal_msg);
                       
                       auto last_time=ros::Time::now();
                        while(!arrived_move_goal  && !find_man   && ros::Time::now()<last_time+ros::Duration(20.0) )
                        {
                            ROS_INFO("go to track 516 ");
                            cal_arrvied_move_goal();
                            avoid_too_high();
                            ros::spinOnce();
                            mini_control_rate.sleep();
                        }
                        }
                        
                    }
                    if(!find_man)
                    {
                        ROS_INFO("   omg!!!  where is he ?  ");
                        break;
                    }

                    ros::spinOnce();
            }
                 
            }
           

  


}



    ROS_INFO("197");
    return 0;
}

