#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>

#include <string>

#include <algorithm>
#include <functional>

#include "jps_puzzle_piece/ImageWithContour.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

class ImagedPieceViz
{
  private:
    ros::NodeHandle nh;
  public:
    ImagedPieceViz(ros::NodeHandle& nh);
};

ImagedPieceViz::ImagedPieceViz(ros::NodeHandle& nh)
{
  this->nh = nh;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imaged_piece_viz_node");
  ros::NodeHandle nh;
  ImagedPieceViz(nh);
  ros::spin();

  return 0;
}

