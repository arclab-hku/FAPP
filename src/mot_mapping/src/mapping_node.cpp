#include "ros/ros.h"
#include "mapping_manager.h"

using namespace mot_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh("~");

  MappingRos mapping_ros(nh);
  mapping_ros.init();

  ros::spin();
  
  return 0;
}