#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Point.h>

#include <algorithm>
#include <functional>

#include <exception>

#include "jps_feature_matching/ImageTransform.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

class PieceLocator
{
  private:
    ros::NodeHandle nh;
    ros::Timer timer;
    tf::StampedTransform base_camera_tf;
    tf::TransformListener tf_listener;
    vector< vector< vector<Point2f> > > piece_central_points;
    vector< vector<Mat> > projection_matrices;
    Mat camera_matrix;
    /*
     * \brief Image subscriber object
     */
    ros::Subscriber image_sub;
    ros::Subscriber camera_info_sub;

  public:
    PieceLocator(ros::NodeHandle& nh);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imageSubscriberCallback(
        const jps_feature_matching::ImageTransformConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
};

PieceLocator::PieceLocator(ros::NodeHandle& nh)
{
  int i;
  this->nh = nh;
  this->timer = this->nh.createTimer(
      ros::Duration(0.1),
      &PieceLocator::timerCallback,
      this);
  this->image_sub = this->nh.subscribe(
      "input_image",
      1,
      &PieceLocator::imageSubscriberCallback,
      this);
  this->camera_info_sub = this->nh.subscribe(
      "camera_info",
      1,
      &PieceLocator::cameraInfoCallback,
      this);

  for(i = 0; i < 4; ++i)
  {
    this->piece_central_points.push_back(vector< vector<Point2f> >());
    this->projection_matrices.push_back(vector<Mat>());
  }
}

void PieceLocator::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  int i;

  if(this->camera_matrix.rows != 3)
  {
    this->camera_matrix = Mat::zeros(3, 3, CV_64F);

    for(i = 0; i < msg->K.size(); ++i)
    {
      this->camera_matrix.at<double>(
          i / this->camera_matrix.cols,
          i % this->camera_matrix.cols) = msg->K[i];
    }
  }
  else
  {
    // No operation
  }
}

void PieceLocator::imageSubscriberCallback(
    const jps_feature_matching::ImageTransformConstPtr& msg)
{
  int i;
  int j = msg->piece_index;
  int k;

  this->piece_central_points[j].push_back(vector<Point2f>());
  k = this->piece_central_points[j].size();

  for(i = 0; i < msg->transformed_points.size(); ++i)
  {
    piece_central_points[j][k].push_back(Point2f(
          msg->transformed_points[i].x,
          msg->transformed_points[i].y));
  }

  // TODO: Continue from here.
  // Generate the projection matrix as [H 0] * E_cw and store both.
  // p_c = E_cw * p_w
}

void PieceLocator::timerCallback(const ros::TimerEvent& event)
{
  try
  {
    this->tf_listener.lookupTransform(
        "/camera_link", "/base", ros::Time(0), this->base_camera_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void QuaternionToRotation(tf::Quaternion q, Mat &r)
{
  double qx2 = q.x() * q.x();
  double qy2 = q.y() * q.y();
  double qz2 = q.z() * q.z();
  double qxqy = q.x() * q.y();
  double qxqz = q.x() * q.z();
  double qxqw = q.x() * q.w();
  double qyqz = q.y() * q.z();
  double qyqw = q.y() * q.w();
  double qzqw = q.z() * q.w();

  r = Mat::zeros(3, 3, CV_64F);
  r.at<double>(0, 0) = 1 - (2.0 * (qy2 + qz2));
  r.at<double>(0, 1) = 2.0 * (qxqy + qzqw);
  r.at<double>(0, 2) = 2.0 * (qxqz - qyqw);
  r.at<double>(1, 0) = 2.0 * (qxqy - qzqw);
  r.at<double>(1, 1) = 1 - (2.0 * (qx2 + qz2));
  r.at<double>(1, 2) = 2.0 * (qyqz + qxqw);
  r.at<double>(2, 0) = 2.0 * (qxqz + qyqw);
  r.at<double>(2, 1) = 2.0 * (qyqz - qxqw);
  r.at<double>(2, 2) = 1 - (2.0 * (qx2 + qy2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "piece_locator_node");
  ros::NodeHandle nh;

  return 0;
}

