#include <ros/ros.h>
#include "vi_grid_map_ros/ImageToViGridmap.hpp"

using namespace::std;
int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_vi_gridmap");
  ros::NodeHandle nh("~");
  grid_map_vi::ImageToViGridmap ImageToViGridmap(nh);
  
  ros::spin();
  return 0;
}
