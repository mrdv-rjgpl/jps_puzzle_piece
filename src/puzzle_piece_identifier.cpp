#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "puzzle_piece_identifier");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());

  if(argc != 2)
  {
    ROS_INFO_STREAM("Usage: " << ros::this_node::getName() << " <img-file-name>");
    return 1;
  }

  Mat image;
  image = imread(argv[1], CV_LOAD_IMAGE_COLOR);

  if(!image.data)
  {
    ROS_INFO_STREAM("Could not read image " << argv[1] << ".");
    return 2;
  }

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", image);
  waitKey(0);

  return 0;
}

