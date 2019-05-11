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

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>

#include <string>

#include <algorithm>
#include <functional>

#include <exception>

#include "jps_puzzle_piece/ImageWithContour.h"
#include "jps_puzzle_piece/PieceParserConfig.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

vector<Scalar> colours;

class PieceParser
{
  private:
    bool robot_stationary;
    /*
     * \brief Image transporter
     */
    image_transport::ImageTransport *img_transport;
    /*
     * \brief Image subscriber object
     */
    image_transport::Subscriber image_sub;
    /*
     * \brief Raw resized image
     */
    Mat img_raw;
    /*
     * \brief Binarized image
     */
    Mat img_bin;
    /*
     * \brief Grayscaled image
     */
    Mat img_gray;
    /*
     * \brief SURF detector object
     */
    Ptr<SURF> surf_detector;
    /*
     * \brief ROS node handler
     */
    ros::NodeHandle nh;
    /*
     * \brief Image publisher object
     */
    ros::Publisher image_pub;
    /*
     * \brief Robot status subscriber object
     */
    ros::Subscriber robot_status_sub;
    /*
     * \brief Visualization publisher object
     */
    image_transport::Publisher vis_pub;
    /*
     * \brief Image binarizing threshold
     */
    int bin_threshold;
    /*
     * \brief Minimum area for contour to be labeled as a piece
     */
    int area_threshold;
    /*
     * \brief Number of points to skip to form angle whose cosine is to be found
     */
    int cos_point_skip;
    /*
     * \brief Number of corners to be detected by algorithm
     */
    int num_corners;
    /*
     * \brief Dynamic reconfigure server
     */
    dynamic_reconfigure::Server<jps_puzzle_piece::PieceParserConfig> reconf_server;
    /*
     * \brief Dynamic reconfigure callback type
     */
    dynamic_reconfigure::Server<jps_puzzle_piece::PieceParserConfig>::CallbackType reconf_callback_type;

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
    void binarizeImage(
        Mat img_input,
        int median_blur_size,
        int bin_threshold,
        int blur_kernel_size);
    /*
     * \brief Check the current contour to determine if it is a valid image of
     * a piece
     *
     * \details The candidate must be above an area threshold, with at most
     * num_pixel_threshold pixels within edge_distance_threshold of the edges
     * of the image.
     * TODO: Figure out what unary predicates are, and how to use them within a
     * class.
     *
     * \param[in] piece_candidate The contour to be checked
     * \param[in] area_threshold The minimum area for a puzzle piece
     * \param[in] num_pixel_threshold The maximum number of pixels close to the
     * image edge
     * \param[in] edge_distance_threshold The distance from the edge outside of
     * which the piece must be
     *
     * \retval true If the image contains a valid puzzle piece
     * \retval false If the image does not contain a valid puzzle piece
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    bool checkPiece(
        vector<Point> piece_candidate,
        int num_pixel_threshold,
        int edge_distance_threshold);
    /*
     * \brief Callback function for dynamic reconfiguration
     *
     * \param[in] config The updated configuration to use
     * \param[in] level The mask indicating which parameters were changed
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    void dynamicReconfigureCallback(
        jps_puzzle_piece::PieceParserConfig& config,
        uint32_t level);
    /*
     * \brief Find all puzzle pieces in the given image
     *
     * \param[in] area_threshold The minimum area for a puzzle piece
     * \param[in] num_pixel_threshold The maximum number of pixels close to the
     * image edge
     * \param[in] edge_distance_threshold The distance from the edge outside of
     * which the piece must be
     *
     * \return The vector of contours of valid puzzle pieces in the supplied
     * image
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    vector< vector<Point> > findPieces(
        int num_pixel_threshold,
        int edge_distance_threshold);
    /*
     * \brief Extract the SURF features of the puzzle piece
     */
    void extractSurf(
        vector<Point2f> piece_contour_f,
        vector<KeyPoint> &kp_img,
        Mat &desc_img);

    /*
     * \brief Get the most likely corner vertices of the given piece
     *
     * \param[in] piece_contour A vector of points forming the piece contour
     * \param[in] num_corners The number of corners to be returned
     */
    /*vector<int> getCorners(vector<Point> piece_contour); */
    /*
     * \brief Get the edges of the puzzle piece
     *
     * \param[in] piece_contour The contour of the piece
     * \param[in] block_size The block size to be used with the Harris corner
     * detector
     * \param[in] aperture_size The aperture size to be used with the Harris
     * corner detector
     * \param[in] harris_free_param The free parameter 'k' to be used with the
     * Harris corner detector
     *
     * \return The edges of the piece
     */
    /*vector<Point> getEdges(
        vector<Point> piece_contour,
        int block_size,
        int aperture_size,
        double harris_free_param); */

  public:
    /*
     * \brief Construct an object of the type PieceParser
     *
     * \param[in] nh The ROS node handler
     * \param[in] min_hessian The minimum Hessian value to be supplied to
     * the SURF feature detection
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    PieceParser(ros::NodeHandle& nh, int min_hessian);
    /*
     * \brief Callback function for the image subscriber object
     *
     * \param[in] msg The message containing the latest image from the camera
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    void imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg);
    /*
     * \brief Callback function for the robot status subscriber object
     *
     * \param[in] msg The message indicating whether the robot is stationary or not
     *
     * \author Mardava Gubbi <mgubbi1@jhu.edu>
     */
    void robotStatusCallback(const std_msgs::BoolConstPtr& msg);
};

/*
 * \brief Construct an object of the type PieceParser
 */
PieceParser::PieceParser(ros::NodeHandle& nh, int min_hessian)
{
  string input_image_topic;
  string output_image_topic;
  ROS_INFO("Initializing piece parser...");
  this->nh = ros::NodeHandle(nh);
  this->img_transport = new image_transport::ImageTransport(this->nh);
  this->bin_threshold = 105;
  this->area_threshold = 100000;
  this->num_corners = 4;
  this->robot_stationary = false;

  // Setup publisher objects
  this->image_pub = this->nh.advertise<jps_puzzle_piece::ImageWithContour>(
      "output_image",
      1000);
  this->vis_pub = this->img_transport->advertise("vis_image", 1000);

  // Setup subscriber object
  this->image_sub = this->img_transport->subscribe(
      "/piece_parser/input_image",
      1,
      &PieceParser::imageSubscriberCallback,
      this);

  // Setup SURF related objects
  this->surf_detector = SURF::create(min_hessian);

  // Setup dynamic reconfiguration objects
  this->reconf_callback_type = boost::bind(
      &PieceParser::dynamicReconfigureCallback,
      this,
      _1,
      _2);
  this->reconf_server.setCallback(this->reconf_callback_type);
}

/*
 * \brief Binarize the image
 */
void PieceParser::binarizeImage(
    Mat img_input,
    int median_blur_size,
    int bin_threshold,
    int blur_kernel_size)
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
  threshold(img_blur, img_thresh, bin_threshold, 255, THRESH_BINARY);
  blur(img_thresh, this->img_bin, Size(blur_kernel_size, blur_kernel_size));
}

/*
 * \brief Check the current contour to determine if it is a valid image of
 * a piece
 */
bool PieceParser::checkPiece(
    vector<Point> piece_candidate,
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  bool piece_validity = true;
  vector<Point>::iterator pixel_iter = piece_candidate.begin();
  int num_pixels_near_edges = 0;
  int contour_area = contourArea(piece_candidate);

  // Ensure that the area is above a certain threshold.
  if(contour_area < this->area_threshold)
  {
    ROS_INFO_STREAM(
        "Contour area error: "
        << contour_area
        << " < area_threshold[="
        << this->area_threshold
        << "]");
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
        ROS_INFO_STREAM("Piece near edge error: Too many pixels near edge");
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

/*
 * \brief Callback function for dynamic reconfiguration
 */
void PieceParser::dynamicReconfigureCallback(
    jps_puzzle_piece::PieceParserConfig& config,
    uint32_t level)
{
  this->bin_threshold = config.bin_threshold;
  this->area_threshold = config.area_threshold;
  this->cos_point_skip = config.cos_point_skip;
}

/*
 * \brief Find all puzzle pieces in the given image
 */
vector< vector<Point> > PieceParser::findPieces(
    int num_pixel_threshold,
    int edge_distance_threshold)
{
  int i;
  vector< vector<Point> > piece_contours;
  vector< vector<Point> > piece_candidates;
  findContours(
      this->img_bin,
      piece_candidates,
      CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_NONE);
  ROS_INFO_STREAM(piece_candidates.size() << " piece candidates found.");

  for(i = 0; i < piece_candidates.size(); ++i)
  {
    if(this->checkPiece(
          piece_candidates[i],
          num_pixel_threshold,
          edge_distance_threshold))
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

/*
 * \brief Extract the SURF features of the puzzle piece
 */
void PieceParser::extractSurf(
    vector<Point2f> piece_contour_f,
    vector<KeyPoint> &kp_img,
    Mat &desc_img)
{
  Mat desc_img_raw;
  int i;
  int j;
  vector<KeyPoint> kp_img_raw;
  vector<int> kp_indices;

  this->surf_detector->detectAndCompute(
      this->img_raw,
      noArray(),
      kp_img_raw,
      desc_img_raw);

  for(i = 0; i < kp_img_raw.size(); ++i)
  {
    if(pointPolygonTest(piece_contour_f, kp_img_raw[i].pt, false) > 0)
    {
      kp_img.push_back(kp_img_raw[i]);
      kp_indices.push_back(i);
    }
    else
    {
      // No operation
    }
  }

  desc_img = Mat(kp_img.size(), desc_img_raw.cols, desc_img_raw.type());

  for(i = 0; i < kp_indices.size(); ++i)
  {
    for(j = 0; j < desc_img_raw.cols; ++j)
    {
      desc_img.at<float>(i, j) = desc_img_raw.at<float>(kp_indices[i], j);
    }
  }
}

/*
 * \brief Callback function for the image subscriber object
 */
void PieceParser::imageSubscriberCallback(
    const sensor_msgs::ImageConstPtr& msg)
{
  double dist_center;
  double min_dist_center = 1e12;
  double resize_factor = 1.0;
  geometry_msgs::Point pt_temp;
  int central_index = -1;
  int i;
  jps_puzzle_piece::ImageWithContour image_msg;
  Mat desc_img;
  Mat img_vis;
  sensor_msgs::ImagePtr vis_msg;
  vector<KeyPoint> kp_img;
  vector<Point2f> piece_contour_f;
  vector< vector<Point> > piece_contours;

  // Acquire robot status as close as possible to image status
  // so as to avoid it being set after the image.
  image_msg.robot_stationary = this->robot_stationary;
  resize(
      cv_bridge::toCvShare(msg, "bgr8")->image,
      this->img_raw,
      Size(),
      resize_factor,
      resize_factor);

  // Binarize the image with reconfigurable thresholds.
  ROS_INFO_STREAM(
      "Binarizing image with threshold " << this->bin_threshold << "...");
  this->binarizeImage(this->img_raw, 5, this->bin_threshold, 3);

  // Obtain the connected components and their centroids.
  ROS_INFO("Finding puzzle pieces in image...");
  piece_contours = this->findPieces(100, 20);
  ROS_INFO_STREAM(piece_contours.size() << " piece contours found.");
  // Ease visualization and debugging with a binarized image.
  cvtColor(this->img_bin, img_vis, CV_GRAY2BGR);

  // Draw all valid piece contours.
  for(i = 0; i < piece_contours.size(); ++i)
  {
    drawContours(img_vis, piece_contours, i, colours[0], 2);
  }

  // Determine the centroids of the contours.
  ROS_INFO("Finding centroids of pieces in image...");
  vector<Point2f> centroids(piece_contours.size());
  Point img_center(this->img_raw.cols / 2, this->img_raw.rows / 2);

  for(i = 0; i < piece_contours.size(); ++i)
  {
    Moments mu = moments(piece_contours[i], false);
    centroids[i] = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

    // Isolate the contour closest to the center of the image.
    dist_center =
      sqrt(
          ((centroids[i].x - img_center.x) * (centroids[i].x - img_center.x))
          + ((centroids[i].y - img_center.y) * (centroids[i].y - img_center.y)));

    if(dist_center < min_dist_center)
    {
      min_dist_center = dist_center;
      central_index = i;
    }
    else
    {
      // No operation
    }
  }

  if(central_index >= 0)
  {
    // Visualize the centroid of the central piece.
    ROS_INFO("Plotting centroid of central piece...");
    circle(
        img_vis,
        centroids[central_index],
        16,
        colours[0],
        -1,
        8,
        0);

    // Populate the messages for SURF feature matching and visualization.
    image_msg.header.stamp = ros::Time::now();
    cv_bridge::CvImage(image_msg.header, "bgr8", this->img_raw).toImageMsg(
        image_msg.image);
    image_msg.centroid_px.x = centroids[central_index].x;
    image_msg.centroid_px.y = centroids[central_index].y;

    for(i = 0; i < piece_contours[central_index].size(); ++i)
    {
      piece_contour_f.push_back(Point2f(
            (double) piece_contours[central_index][i].x,
            (double) piece_contours[central_index][i].y));

      pt_temp.x = (double) piece_contours[central_index][i].x;
      pt_temp.y = (double) piece_contours[central_index][i].y;
      pt_temp.z = 0.0;
      image_msg.contour_px.push_back(pt_temp);
    }

    // Extract SURF features
    this->extractSurf(piece_contour_f, kp_img, desc_img);

    cv_bridge::CvImage(image_msg.header, sensor_msgs::image_encodings::TYPE_32FC1, desc_img).toImageMsg(
        image_msg.surf_desc);

    for(i = 0; i < kp_img.size(); ++i)
    {
      pt_temp.x = (double) kp_img[i].pt.x;
      pt_temp.y = (double) kp_img[i].pt.y;
      image_msg.surf_key_points.push_back(pt_temp);
    }

    ROS_INFO_STREAM(
        "Publishing "
        << piece_contours[central_index].size()
        << " points on central contour...");
    this->image_pub.publish(image_msg);
  }
  else
  {
    ROS_INFO("No piece found.");
  }

  vis_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vis).toImageMsg();
  this->vis_pub.publish(vis_msg);
  this->robot_stationary = false;
}

void PieceParser::robotStatusCallback(const std_msgs::BoolConstPtr& msg)
{
  // Latch to 1 if required.
  if(msg->data == 1)
  {
    this->robot_stationary = true;
  }
  else
  {
    // No operation
  }
}

int main(int argc, char **argv)
{
  colours.push_back(
      Scalar(int(0.741 * 256), int(0.447 * 256), int(0.000 * 256)));
  colours.push_back(
      Scalar(int(0.098 * 256), int(0.325 * 256), int(0.850 * 256)));
  ros::init(argc, argv, "piece_parser_node");
  ros::NodeHandle nh;
  PieceParser p(nh, 400.0);
  ros::spin();

  return 0;
}
