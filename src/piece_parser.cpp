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

#include <exception>

using namespace cv;
using namespace std;

vector<Scalar> colours;

class PieceParser
{
  private:
    /*
     * \brief Image transporter
     */
    image_transport::ImageTransport *img_transport;
    /*
     * \brief Image subscriber object
     */
    image_transport::Subscriber image_sub;
    /*
     * \brief Binarized image
     */
    Mat img_bin;
    /*
     * \brief Grayscaled image
     */
    Mat img_gray;
    /*
     * \brief ROS node handler
     */
    ros::NodeHandle nh;
    /*
     * \brief Image publisher object
     */
    ros::Publisher image_pub;

    /*
     * \brief Check the current contour to determine if it is a valid image of a piece
     *
     * \details The candidate must be above an area threshold, with at most num_pixel_threshold pixels within
     * edge_distance_threshold of the edges of the image.
     * TODO: Figure out what unary predicates are, and how to use them within a class.
     *
     * \param[in] piece_candidate The contour to be checked
     * \param[in] area_threshold The minimum area for a puzzle piece
     * \param[in] num_pixel_threshold The maximum number of pixels close to the image edge
     * \param[in] edge_distance_threshold The distance from the edge outside of which the piece must be
     *
     * \retval true If the image contains a valid puzzle piece
     * \retval false If the image does not contain a valid puzzle piece
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    bool checkPiece(
        vector<Point> piece_candidate,
        int area_threshold,
        int num_pixel_threshold,
        int edge_distance_threshold);

  public:
    /*
     * \brief Construct an object of the type PieceParser
     *
     * \param[in] nh The ROS node handler
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    PieceParser(ros::NodeHandle& nh);
    /*
     * \brief Get the edges of the puzzle piece
     *
     * \param[in] piece_contour The contour of the piece
     * \param[in] block_size The block size to be used with the Harris corner detector
     * \param[in] aperture_size The aperture size to be used with the Harris corner detector
     * \param[in] harris_free_param The free parameter 'k' to be used with the Harris corner detector
     *
     * \return The edges of the piece
     */
    vector<Point> getEdges(
        vector<Point> piece_contour,
        int block_size,
        int aperture_size,
        double harris_free_param);
    /*
     * \brief Find all puzzle pieces in the given image
     *
     * \param[in] area_threshold The minimum area for a puzzle piece
     * \param[in] num_pixel_threshold The maximum number of pixels close to the image edge
     * \param[in] edge_distance_threshold The distance from the edge outside of which the piece must be
     *
     * \return The vector of contours of valid puzzle pieces in the supplied image
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    vector< vector< Point> > findPieces(int area_threshold, int num_pixel_threshold, int edge_distance_threshold);
    /*
     * \brief Callback function for the image subscriber object
     *
     * \param[in] msg The message containing the latest image from the camera
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    void imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg);
    /*
     * \brief Binarize the image
     *
     * \param[in] img_input The input image to be binarized
     * \param[in] median_blur_size The size to be used for median blurring
     * \param[in] bin_threshold The threshold for image binarization
     * \param[in] blur_kernel_size The kernel size to be used for blurring
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    void binarizeImage(Mat img_input, int median_blur_size, int bin_threshold, int blur_kernel_size);
};

bool PieceParser::checkPiece(
    vector<Point> piece_candidate,
    int area_threshold,
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  bool piece_validity = true;
  vector<Point>::iterator pixel_iter = piece_candidate.begin();
  int num_pixels_near_edges = 0;

  // Ensure that the area is above a certain threshold.
  if(contourArea(piece_candidate) < area_threshold)
  {
    piece_validity = false;
  }
  else
  {
    // No operation
  }

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

  return piece_validity;
}

PieceParser::PieceParser(ros::NodeHandle& nh)
{
  ROS_INFO("Initializing piece parser...");
  this->nh = ros::NodeHandle(nh);
  this->img_transport = new image_transport::ImageTransport(this->nh);
  this->image_sub = this->img_transport->subscribe("/usb_cam/image_raw", 1, &PieceParser::imageSubscriberCallback, this);
}

vector<Point> PieceParser::getEdges(
    vector<Point> piece_contour,
    int block_size,
    int aperture_size,
    double harris_free_param)
{
  int i;
  int j;
  int k;
  int max_corner_points = 32; // Twice of what we need, should contain the required corners.
  int pixel_dist_threshold = 10;

  ROS_INFO("Initializing Harris corner matrix...");
  Mat harris_corners = Mat::zeros(this->img_gray.size(), CV_32FC1);
  ROS_INFO("Fetching Harris corners...");
  cornerHarris(this->img_gray, harris_corners, block_size, aperture_size, harris_free_param);
  Point corner_points[max_corner_points];
  double corner_vals[max_corner_points];
  int num_indices = 0;

  for(i = 0; i < max_corner_points; ++i)
  {
    corner_vals[i] = -1e9;
    corner_points[i] = Point(0, 0);
  }

  ROS_INFO("Checking Harris corners for top values...");

  for(i = 0; i < piece_contour.size(); ++i)
  {
    const double harris_val_curr = harris_corners.at<double>(piece_contour[i].x, piece_contour[i].y);

    for(j = 0; j < max_corner_points; ++j)
    {
      if(harris_val_curr > corner_vals[j])
      {
        ROS_INFO_STREAM(++num_indices << " peak points found.");

        for(k = max_corner_points - 1; k > j; --k)
        {
          corner_vals[k] = corner_vals[k - 1];
          corner_points[k] = corner_points[k - 1];
        }

        Point temp_pt = Point(piece_contour[i].x, piece_contour[i].y);
        corner_vals[j] = harris_val_curr;
        corner_points[j] = temp_pt;
        i += pixel_dist_threshold;

        break;
      }
      else
      {
        // No operation
      }
    }
  }

  ROS_INFO_STREAM("Converting to vector form...");

  vector<Point> corner_points_vec;

  for(i = 0; i < max_corner_points; ++i)
  {
    corner_points_vec.push_back(corner_points[i]);
  }

  ROS_INFO_STREAM("Maximum Harris corner values successfully obtained.");

  return corner_points_vec;
}

vector< vector<Point> > PieceParser::findPieces(
    int area_threshold,
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  int i;
  vector< vector<Point> > piece_contours;
  vector< vector<Point> > piece_candidates;
  findContours(this->img_bin, piece_candidates, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  for(i = 0; i < piece_candidates.size(); ++i)
  {
    if(this->checkPiece(piece_candidates[i], area_threshold, num_pixel_threshold, edge_distance_threshold))
    {
      piece_contours.push_back(piece_candidates[i]);
    }
    else
    {
      // No operation
    }
  }

  return piece_contours;
}

void PieceParser::imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg)
{
  int i;
  int j;

  // Binarize the image with preset thresholds.
  // TODO: tweak the thresholds if need be.
  ROS_INFO("Binarizing image...");
  Mat img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
  this->binarizeImage(img_raw, 5, 105, 3);

  // Obtain the connected components and their centroids.
  ROS_INFO("Finding puzzle pieces in image...");
  vector< vector<Point> > piece_contours = this->findPieces(100000, 100, 20);
  ROS_INFO("Finding centroids of pieces in image...");
  vector<Point2f> centroids(piece_contours.size());

  for(i = 0; i < piece_contours.size(); ++i)
  {
    Moments mu = moments(piece_contours[i], false);
    centroids[i] = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    drawContours(img_raw, piece_contours, i, colours[0], 2); // Thickness = 2
    circle(img_raw, centroids[i], 16, colours[0], -1, 8, 0);
  }

  // TODO: tweak edge classification parameters if need be.
  ROS_INFO("Finding Harris corners...");

  for(i = 0; i < piece_contours.size(); ++i)
  {
    vector<Point> harris_corners = getEdges(piece_contours[i], 5, 5, 0.04);
    ROS_INFO("Displaying Harris corners...");

    for(j = 0; j < harris_corners.size(); ++j)
    {
      circle(img_raw, harris_corners[j], 16, colours[1], -1, 8, 0);
    }
  }

  namedWindow("display_window", WINDOW_AUTOSIZE);
  imshow("display_window", img_raw);
  waitKey(1);
}

void PieceParser::binarizeImage(Mat img_input, int median_blur_size, int bin_threshold, int blur_kernel_size)
{
  Mat img_blur;
  Mat img_thresh;

  if(this->img_bin.rows == 0)
  {
    this->img_gray = Mat(img_input.size(), CV_8UC1);
    this->img_bin = Mat(this->img_gray.size(), this->img_gray.type());
  }
  else
  {
    // No operation
  }

  cvtColor(img_input, this->img_gray, CV_RGB2GRAY);
  img_blur = Mat(this->img_gray.size(), this->img_gray.type());
  medianBlur(this->img_gray, img_blur, median_blur_size);
  img_thresh = Mat(this->img_gray.size(), this->img_gray.type());
  threshold(img_blur, img_thresh, bin_threshold, 255, THRESH_BINARY_INV);
  blur(img_thresh, this->img_bin, Size(blur_kernel_size, blur_kernel_size));
}

int main(int argc, char **argv)
{
  colours.push_back(Scalar(int(0.741 * 256), int(0.447 * 256), int(0.000 * 256)));
  colours.push_back(Scalar(int(0.098 * 256), int(0.325 * 256), int(0.850 * 256)));
  ros::init(argc, argv, "piece_parser_node");
  ros::NodeHandle nh;
  PieceParser p(nh);
  ros::spin();

  return 0;
}

