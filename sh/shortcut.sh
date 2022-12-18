#!/bin/bash

#gnome-terminal --tab -- bash -c "roslaunch px4 robocup.launch ;exec bash"
#sleep 5
#gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash && cd python &&    bash multi_gimbal_control.sh  ;exec bash"


#1.发布无人机世界坐标系下的odom（包含位姿 和 速度）
gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash &&    roslaunch ego_planner my_odom_world.launch   ;exec bash"



#2.相机位姿转换
gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash && roslaunch ego_planner my_ego_transfer.launch   ;exec bash"

#3.无人机底层控制器
gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash &&    roslaunch px4ctrl run_ctrl.launch  ;exec bash"

sleep 8

#gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash &&     sh shfiles/takeoff.sh          ;exec bash"

gnome-terminal --tab -- bash -c "cd ..&& source  devel/setup.bash &&    cd python && rviz -d default.rviz         ;exec bash"

#4.最大目标的xy
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch publishbox multi_publish_boxes.launch ;exec bash"

#5.最大目标的深度,发布tf，坐标转换得到目标的世界坐标；发布相机位姿(真值)
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch robocup get_image_depth_multi.launch  ;exec bash"

#6.多无人机建图规划
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch ego_planner my_multi_uav.launch ;exec bash"

#7.darknet ros
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch darknet_ros multi_target_search.launch ;exec bash"






