#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

class PuzzlePieceIdentifier
{
  private:
    ros::NodeHandle *nh;
    Mat img_gray;
    Rect region_of_interest;

  public:
    PuzzlePieceIdentifier(ros::NodeHandle *nh, char *img_file_name, bool use_roi, int x, int y, int width, int height);
};

PuzzlePieceIdentifier::PuzzlePieceIdentifier(
    ros::NodeHandle *nh,
    char *img_file_name,
    bool use_roi,
    int x,
    int y,
    int width,
    int height)
{
  this->nh = nh;
  Mat img_raw = imread(img_file_name, CV_LOAD_IMAGE_COLOR);

  if(img_raw.data)
  {
    if(use_roi)
    {
      this->region_of_interest = Rect(x, y, width, height);
      this->img_gray = Mat(Size(region_of_interest.width, region_of_interest.height), img_raw.type());
      cvtColor(img_raw(region_of_interest), this->img_gray, CV_RGB2GRAY);
    }
    else
    {
      this->img_gray = Mat(img_raw.size(), img_raw.type());
      cvtColor(img_raw, this->img_gray, CV_RGB2GRAY);
    }
  }
  else
  {
    throw runtime_error("Could not open file.");
  }

  ROS_INFO_STREAM("Median blurring image of type " << img_gray.type() << "...");
  Mat img_blur(this->img_gray.size(), this->img_gray.type());
  medianBlur(this->img_gray, img_blur, 5);

  ROS_INFO_STREAM("Thresholding image...");
  Mat img_thresh(img_blur.size(), img_blur.type());
  threshold(img_blur, img_thresh, 120, 255, THRESH_BINARY_INV);

  ROS_INFO_STREAM("Blurring image...");
  blur(img_thresh, img_blur, Size(3, 3));

  ROS_INFO_STREAM("Identifying connected components in image...");
  Mat img_labels;
  Mat img_stats;
  Mat conn_centroids;
  int num_labels = connectedComponentsWithStats(img_blur, img_labels, img_stats, conn_centroids);
  ROS_INFO_STREAM(num_labels << " connected components identified, with " << conn_centroids.size() <<  " centroids.");

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

  for(i = 0; i < contours.size(); ++i)
  {
    drawContours(img_edges, contours, i, Scalar(rand() % 256, rand() % 256, rand() % 256), 2); // Thickness = 2
  }

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", img_edges);
  waitKey(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "puzzle_piece_identifier");
  ros::NodeHandle nh;

  if(argc == 2)
  {
    // PuzzlePieceIdentifier p(&nh, argv[1], 200, 180, 300, 300);
    PuzzlePieceIdentifier p(&nh, argv[1], false, 0, 0, 1278, 951);
  }
  else
  {
    ROS_INFO_STREAM("Usage: " << ros::this_node::getName() << " <img-file-name>");

    return 1;
  }

  return 0;
}

