#include "vi_grid_map_ros/ImageToViGridmap.hpp"

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// STD
#include <vector>

namespace grid_map_vi {

ImageToViGridmap::ImageToViGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pnh_("~"),
      map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToViGridmap::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
}

ImageToViGridmap::~ImageToViGridmap()
{
}

bool ImageToViGridmap::readParameters()
{
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
  nodeHandle_.param("resolution", resolution_, 0.03);
  nodeHandle_.param("min_height", minHeight_, 0.0);
  nodeHandle_.param("max_height", maxHeight_, 1.0);
  return true;
}

//透過処理
bool ImageToViGridmap::transparency_processing(const sensor_msgs::Image& image)
{
  

  //cv::imwrite("/tmp/test.png", alpha_image);
  
}

void ImageToViGridmap::imageCallback(const sensor_msgs::Image& msg)
{
  //初回のみ
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
             map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }

  //cv_bridge & chanell探索 & map作成
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  //カラー画像は使用しないのでコメントアウト
  //grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);

  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
}

} /* namespace */
