#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <string>

using namespace cv;
using namespace std;

class PieceParser
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *img_transport;
    Mat img_raw;
    Mat img_bin;
    image_transport::Subscriber image_sub;
    ros::Publisher image_pub;

  public:
    PieceParser(ros::NodeHandle& nh);
    void imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageBinarize(Mat img_raw);
};

PieceParser::PieceParser(ros::NodeHandle& nh)
{
  ROS_INFO("Initializing piece parser...");
  this->nh = ros::NodeHandle(nh);
  this->img_transport = new image_transport::ImageTransport(this->nh);
  this->image_sub = this->img_transport->subscribe("/camera/image", 1, &PieceParser::imageSubscriberCallback, this);
}

void PieceParser::imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg)
{
  Mat conn_centroids;
  Mat img_labels;
  Mat img_stats;

  // Binarize the image with preset thresholds.
  // TODO: tweak the thresholds if need be.
  this->imageBinarize(cv_bridge::toCvShare(msg, "bgr8")->image);

  // Obtain the connected components and their centroids.
  int num_labels = connectedComponentsWithStats(this->img_bin, img_labels, img_stats, conn_centroids);
  ROS_INFO_STREAM(num_labels << " connected components identified, with " << conn_centroids.size() <<  " centroids.");

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", img_bin);
  waitKey(1);
}

void PieceParser::imageBinarize(Mat img_input)
{
  Mat img_blur;
  Mat img_thresh;

  if(this->img_raw.rows == 0)
  {
    this->img_raw = Mat(img_input);
    this->img_bin = Mat(img_input.size(), img_input.type());
  }
  else
  {
    img_input.copyTo(this->img_raw);
  }

  cvtColor(this->img_raw, this->img_bin, CV_RGB2GRAY);
  img_blur = Mat(this->img_bin.size(), this->img_bin.type());
  img_thresh = Mat(this->img_bin.size(), this->img_bin.type());
  medianBlur(this->img_bin, img_blur, 5);
  threshold(img_blur, img_thresh, 120, 255, THRESH_BINARY_INV);
  blur(img_thresh, this->img_bin, Size(3, 3));
}

/*
PieceParser::PieceParser(
    ros::NodeHandle *nh,
    string img_file_name,
    bool use_roi,
    int x,
    int y,
    int width,
    int height)
*/
/*
{
  this->nh = nh;
  Mat img_raw = imread(img_file_name, CV_LOAD_IMAGE_COLOR);

  if(img_raw.data)
  {
    if(use_roi)
    {
      this->region_of_interest = Rect(x, y, width, height);
      this->img_bin = Mat(Size(region_of_interest.width, region_of_interest.height), img_raw.type());
      cvtColor(img_raw(region_of_interest), this->img_bin, CV_RGB2GRAY);
    }
    else
    {
      this->img_bin = Mat(img_raw.size(), img_raw.type());
      cvtColor(img_raw, this->img_bin, CV_RGB2GRAY);
    }
  }
  else
  {
    throw runtime_error("Could not open file.");
  }

  ROS_INFO_STREAM("Median blurring image of type " << img_bin.type() << "...");
  Mat img_blur(this->img_bin.size(), this->img_bin.type());
  medianBlur(this->img_bin, img_blur, 5);

  ROS_INFO_STREAM("Thresholding image...");
  Mat img_thresh(img_blur.size(), img_blur.type());
  threshold(img_blur, img_thresh, 120, 255, THRESH_BINARY_INV);

  ROS_INFO_STREAM("Blurring image...");
  blur(img_thresh, img_blur, Size(3, 3));

  ROS_INFO_STREAM("Coloring connected components...");
  vector<Vec3b> colors(num_labels + 1);
  colors[0] = Vec3b(0, 0, 0);
  int i;
  int j;

  for(i = 1; i <= num_labels; ++i)
  {
    colors[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }

  Mat img_labelled(img_blur.size(), CV_8UC3);

  for(i = 0; i < img_labelled.rows; ++i)
  {
    for(j = 0; j < img_labelled.cols; ++j)
    {
      img_labelled.at<Vec3b>(i, j) = colors[img_labels.at<int>(i, j)];
    }
  }

  ROS_INFO_STREAM("Finding contours...");
  vector< vector<Point> > contours;
  findContours(img_blur, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  ROS_INFO_STREAM(contours.size() << " contours found.");
  Mat img_edges = Mat::zeros(img_blur.size(), CV_8UC3);
  vector<Point2f> centroids(contours.size());

  for(i = 0; i < contours.size(); ++i)
  {
    Moments mu = moments(contours[i], false);
    centroids[i] = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    Scalar colour = Scalar(rand() % 256, rand() % 256, rand() % 256);
    drawContours(img_edges, contours, i, colour, 2); // Thickness = 2
    circle(img_edges, centroids[i], 16, colour, -1, 8, 0);
  }

  string out_file_name;
  // imwrite(out_file_name, img_edges);

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", img_edges);
  waitKey(0);
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "piece_parser");
  ros::NodeHandle nh;
  PieceParser p(nh);
  ros::spin();

  return 0;
}

