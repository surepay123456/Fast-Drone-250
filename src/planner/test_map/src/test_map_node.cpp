#include "plan_env/grid_map.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

//using namespace ego_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "map_test_node");
  ros::NodeHandle nh("~");
  GridMap::Ptr grid_map_;
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);
  ros::spin();

  return 0;
}