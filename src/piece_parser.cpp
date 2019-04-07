#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <string>

#include <algorithm>
#include <functional>

using namespace cv;
using namespace std;

class PieceParser
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *img_transport;
    Mat img_bin;
    image_transport::Subscriber image_sub;
    ros::Publisher image_pub;
    // TODO: Figure out what unary predicates are, and how to use them within a class.
    bool checkPiece(
        vector<Point> piece_candidate,
        int area_threshold,
        int num_pixel_threshold,
        int edge_distance_threshold);

  public:
    PieceParser(ros::NodeHandle& nh);
    void imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg);
    void binarizeImage(Mat img_input, int median_blur_size, int bin_threshold, int blur_kernel_size);
    vector< vector< Point> > findPieces(int area_threshold, int num_pixel_threshold, int edge_distance_threshold);
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
  // Binarize the image with preset thresholds.
  // TODO: tweak the thresholds if need be.
  ROS_INFO("Binarizing image...");
  this->binarizeImage(cv_bridge::toCvShare(msg, "bgr8")->image, 5, 120, 3);

  // Obtain the connected components and their centroids.
  ROS_INFO("Finding puzzle pieces in image...");
  vector< vector<Point> > piece_contours = this->findPieces(100000, 100, 20);

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", img_bin);
  waitKey(1);
}

void PieceParser::binarizeImage(Mat img_input, int median_blur_size, int bin_threshold, int blur_kernel_size)
{
  Mat img_blur;
  Mat img_thresh;

  ROS_INFO("Checking if image sizes have already been defined...");
  if(this->img_bin.rows == 0)
  {
    ROS_INFO("Initializing img_bin...");
    this->img_bin = Mat(img_input.size(), img_input.type());
  }
  else
  {
    // No operation
  }

  ROS_INFO("Converting to grayscale...");
  cvtColor(img_input, this->img_bin, CV_RGB2GRAY);

  ROS_INFO("Median blurring...");
  img_blur = Mat(this->img_bin.size(), this->img_bin.type());
  medianBlur(this->img_bin, img_blur, median_blur_size);

  ROS_INFO("Running thresholding operation...");
  img_thresh = Mat(this->img_bin.size(), this->img_bin.type());
  threshold(img_blur, img_thresh, bin_threshold, 255, THRESH_BINARY_INV);

  ROS_INFO("Running post-thresholding blurring...");
  blur(img_thresh, this->img_bin, Size(blur_kernel_size, blur_kernel_size));
}

vector< vector<Point> > PieceParser::findPieces(
    int area_threshold,
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  int i;
  vector< vector<Point> > piece_contours;
  vector< vector<Point> > piece_candidates;
  ROS_INFO("Finding puzzle piece contours...");
  findContours(this->img_bin, piece_candidates, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  ROS_INFO("Checking puzzle piece candidates...");

  for(i = 0; i < piece_candidates.size(); ++i)
  {
    if(this->checkPiece(piece_candidates[i], area_threshold, num_pixel_threshold, edge_distance_threshold))
    {
      ROS_INFO("Adding piece to valid piece list...");
      piece_contours.push_back(piece_candidates[i]);
    }
    else
    {
      // No operation
    }
  }

  return piece_contours;
}

bool PieceParser::checkPiece(
    vector<Point> piece_candidate,
    int area_threshold,
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  bool piece_validity = true;
  vector<Point>::iterator pixel_iter = piece_candidate.begin();
  int num_pixels_near_edges = 0;

  ROS_INFO("Checking area of current piece candidate...");
  // Ensure that the area is above a certain threshold.
  if(contourArea(piece_candidate) < area_threshold)
  {
    ROS_INFO("Piece candidate invalidated due to insufficient area.");
    piece_validity = false;
  }
  else
  {
    // No operation
  }

  ROS_INFO("Checking distance of piece candidate from edge...");

  // Ensure that the piece candidate is not close to the edge of the image.
  while((piece_validity == true) && (pixel_iter != piece_candidate.end()))
  {
    if((pixel_iter->x < edge_distance_threshold)
        || (pixel_iter->x >= this->img_bin.cols - edge_distance_threshold)
        || (pixel_iter->y < edge_distance_threshold)
        || (pixel_iter->y >= this->img_bin.rows - edge_distance_threshold))
    {
      ++num_pixels_near_edges;

      if(num_pixels_near_edges >= num_pixel_threshold)
      {
        ROS_INFO("Piece candidate invalidated due to proximity to edge.");
        piece_validity = false;

        break;
      }
      else
      {
        // No operation
      }
    }
    else
    {
      // No operation
    }

    ++pixel_iter;
  }

  return true;
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

