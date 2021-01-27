#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace grid_map_vi {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class ImageToViGridmap
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ImageToViGridmap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ImageToViGridmap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();
  bool transparency_processing(const sensor_msgs::Image& image);

  void imageCallback(const sensor_msgs::Image& msg);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;
  ros::NodeHandle pnh_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Image subscriber
  ros::Subscriber imageSubscriber_;

  //! Name of the grid map topic.
  std::string imageTopic_;

  //! Length of the grid map in x direction.
  double mapLengthX_;

  //! Resolution of the grid map.
  double resolution_;

  //! Range of the height values.
  double minHeight_;
  double maxHeight_;

  //! Frame id of the grid map.
  std::string mapFrameId_;

  bool mapInitialized_;
};

} /* namespace */
