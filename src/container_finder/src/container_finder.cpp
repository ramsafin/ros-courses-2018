#include <string>
#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>
#include <numeric>
#include <utility>
#include <unordered_map>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImageSubscriber.hpp"
#include "LaserSubscriber.hpp"

constexpr const char* kCmdVelocityTopic = "/mobile_base/commands/velocity";
constexpr const char* kImageRawTopic = "/camera/rgb/image_raw";
constexpr const char* kLaserScanTopic = "/scan";

constexpr const char* kCaptureWindow = "Raw Video";
constexpr const char* kHsvWindow = "HSV Video";

// use hsv_calibration node to obtain these parameters
constexpr int H_low = 35;
constexpr int H_high = 80;
constexpr int S_low =  50;
constexpr int S_high = 125;
constexpr int V_low =  5;
constexpr int V_high =  55;

constexpr double kLargeContainerSideMinRatio = 1.3;
constexpr double kSmallContainerSideMinRatio = 0.65;

// camera
cv::Mat binary_image;

// laser
double distance_to_obstacle = 0.0;

//
// Calculates the center point of white pixels area in the binary image.
// 
cv::Point find_centroid(const cv::Mat& binary_frame) {
  auto moments = cv::moments(binary_frame, /*binary image*/ true);

  // returns negative values in case if there are no white pixels
  return cv::Point(int(moments.m10 / moments.m00), int(moments.m01 / moments.m00));
}

enum class ContainerSide {
  LARGE, SMALL, NONE
};

std::unordered_map<ContainerSide, std::string> side_mapping = {
  {ContainerSide::LARGE, "large"},
  {ContainerSide::SMALL, "small"},
  {ContainerSide::NONE, "none"}
};

//
// Determines the container's side or its absence.
// Returns side - ratio pair 
//
std::pair<ContainerSide, double> determine_container_side(const cv::Mat& binary_frame) {
  // compute the number of horizontal and vertical pixels in the frame
  int max_horizontal = 0;
  int max_vertical = 0;

  // column-wise
  for (int j = 0; j < binary_frame.cols; ++j) {
    max_vertical = std::max(cv::countNonZero(binary_frame.col(j)), max_vertical);
  }

  // row-wise
  for (int i = 0; i < binary_frame.rows; ++i) {
    max_horizontal = std::max(cv::countNonZero(binary_frame.row(i)), max_horizontal);
  }

  double ratio = max_horizontal / static_cast<double>(max_vertical);

  if (std::isnan(ratio) || ratio < kSmallContainerSideMinRatio) {
    return std::make_pair(ContainerSide::NONE, ratio);
  }

  std::cout << "Ratio: " << ratio << '\n';

  return ratio < kLargeContainerSideMinRatio? std::make_pair(ContainerSide::SMALL, ratio) : std::make_pair(ContainerSide::LARGE, ratio);
}


void image_callback(const sensor_msgs::ImageConstPtr& image_msg) {
  cv::Mat hsv_frame, threshold_frame;

  auto image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);

  cv::GaussianBlur(image_ptr->image, image_ptr->image, cv::Size(5, 5), 0, 0);

  cv::cvtColor(image_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_frame, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), threshold_frame);

  binary_image = threshold_frame.clone();  // update the image

  auto centroid = find_centroid(threshold_frame);

  // draw the centroid of white pixels
  if (centroid.x >= 0 && centroid.y >= 0) {
    cv::circle(image_ptr->image, centroid, 3, cv::Scalar(0, 0, 255), -1);   
  }

  auto side_ratio = determine_container_side(binary_image);

  cv::putText(image_ptr->image, std::string{"Ratio: "} + std::to_string(side_ratio.second), cv::Point(5, 15),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::putText(image_ptr->image, std::string{"Distance (mean): "} + std::to_string(distance_to_obstacle), cv::Point(5, 35),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::putText(image_ptr->image, std::string{"Side: "} + side_mapping[side_ratio.first], cv::Point(5, 55),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::imshow(kCaptureWindow, image_ptr->image);
  cv::imshow(kHsvWindow, threshold_frame);

  cv::waitKey(50);
}


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  int maxScanIndex = std::floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

  std::vector<float> filtered_ranges;

  std::copy_if(scan->ranges.begin(), scan->ranges.end(),
   std::back_inserter(filtered_ranges), [&scan](float range) {
      return range > scan->range_min && range < scan->range_max;
   });

  double mean = 0.0;

  if (filtered_ranges.size() < 20) {
    mean = std::accumulate(filtered_ranges.begin(), filtered_ranges.end(), 0.0) / filtered_ranges.size();
  } else {
    // distance around the middle
    auto middle_it = filtered_ranges.begin() + filtered_ranges.size() / 2;
    mean = std::accumulate(middle_it - 10, middle_it + 9, 0.0) / 20.0;
  }

  distance_to_obstacle = mean;
}


void capture_container(ros::Publisher& pub) {

  // std::cout << "Capturing the container...\n";

  double threshold = 0.07 * binary_image.cols;

  ros::Rate r(5);

  while(binary_image.dims < 2) {
    ros::spinOnce();
    r.sleep();
  }

  auto centroid = find_centroid(binary_image);

  // while centroid is not near the image's center
  while(std::abs(centroid.x - binary_image.cols / 2) > 30) {
    centroid = find_centroid(binary_image);

    double diff = centroid.x - binary_image.cols / 2;

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = diff < 0 ? 0.15 : -0.15;
    pub.publish(cmd);

    ros::spinOnce();  // update sensor data

    // std::cout << "Equalling to centroid... " << diff << '\n';
    r.sleep();
  }

  // while white pixels area is not in the image
  auto left_range = cv::countNonZero(binary_image.colRange(0, 3)) / 3.0;
  auto right_range = cv::countNonZero(binary_image.colRange(binary_image.cols - 3, binary_image.cols - 1)) / 3.0;

  while (left_range > threshold && right_range > threshold) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.15;
    pub.publish(cmd);

    ros::spinOnce(); // update sensor data

    left_range = cv::countNonZero(binary_image.colRange(0, 3)) / 3.0;
    right_range = cv::countNonZero(binary_image.colRange(binary_image.cols - 3, binary_image.cols - 1)) / 3.0;

    // std::cout << "Equalling to the edges...\n";
    r.sleep();
  }
}


void search_large_side(ros::Publisher& pub) {
  ros::Rate r(10);

  auto side_ratio = determine_container_side(binary_image);

  while (side_ratio.first != ContainerSide::LARGE) {

    std::cout << "Searching for large side\n";

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.25;
    pub.publish(cmd);

    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "container_finder");

  ros::NodeHandle nh;

  ros::Rate rate(50);

  cv::namedWindow(kCaptureWindow);
  cv::namedWindow(kHsvWindow);

  // sensor subscribers
  ImageSubscriber img_subscriber(nh, kImageRawTopic, &image_callback);
  LaserSubscriber laser_subscriber(nh, kLaserScanTopic, &laser_callback);

  // actuator
  auto cmv_vel_publisher = nh.advertise<geometry_msgs::Twist>(kCmdVelocityTopic, 10);

  while (nh.ok()) {
    ros::spinOnce();  // process sensor data

    capture_container(cmv_vel_publisher);  // rotate around until the container is not found

    auto side_ratio = determine_container_side(binary_image);

    if (side_ratio.first == ContainerSide::LARGE) {

      auto centroid = find_centroid(binary_image);

      // approaching until it reaches 1 m distance (+/- 0.05m)
      if (centroid.x > 0 && centroid.y > 0) {
        if (std::abs(distance_to_obstacle - 1.0) < 0.05) {
          ros::shutdown();
          continue;
        }

        int err = centroid.x - binary_image.cols / 2;

        geometry_msgs::Twist cmd;
        cmd.linear.x = 1.0;
        cmd.angular.z = -(float) err / 100;
        cmv_vel_publisher.publish(cmd);
      }
    } else if (side_ratio.first == ContainerSide::SMALL) {
      search_large_side(cmv_vel_publisher);
    }

    rate.sleep();
  }

  cv::destroyAllWindows();

  return 0;
}