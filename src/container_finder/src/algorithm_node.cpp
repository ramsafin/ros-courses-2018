#include <cmath>
#include <string>
#include <utility>
#include <numeric>
#include <algorithm>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "LaserSubscriber.hpp"
#include "ImageSubscriber.hpp"


// topic names
constexpr const char* kCmdVelocityTopic = "/mobile_base/commands/velocity";

constexpr const char* kLaserScanTopic   = "/scan";
constexpr const char* kImageRawTopic    = "/camera/rgb/image_raw";

// OpenCV window names
constexpr const char* kCaptureWindow = "Raw Video";
constexpr const char* kHsvWindow     = "HSV Video";

// HSV filter parameters
constexpr std::pair<int, int> kHue        = {35, 80};
constexpr std::pair<int, int> kValue      = {5, 55};
constexpr std::pair<int, int> kSaturation = {50, 125};

using RPY = boost::tuple<double, double, double>;


RPY rpy_from(const geometry_msgs::Quaternion& q) {
  tf::Quaternion q_tf(q.x, q.y, q.z, q.w);
  double r, p, y;

  tf::Matrix3x3 m(q_tf);
  m.getRPY(r, p ,y);
  
  return {r, p, y};
}

cv::Point compute_centroid(const cv::Mat& binary_frame) {
  auto moments = cv::moments(binary_frame, /* binary */ true);
  // returns negative values in case if there are no white pixels
  return cv::Point(int(moments.m10 / moments.m00), int(moments.m01 / moments.m00));
}

float compute_ratio(const cv::Mat& binary_frame) {
  int max_horizontal = 0;
  int max_vertical = 0;

  for (int j = 0; j < binary_frame.cols; ++j) {
    max_vertical = std::max(cv::countNonZero(binary_frame.col(j)), max_vertical);
  }

  for (int i = 0; i < binary_frame.rows; ++i) {
    max_horizontal = std::max(cv::countNonZero(binary_frame.row(i)), max_horizontal);
  }

  return max_horizontal / static_cast<float>(max_vertical);
}

void image_callback(const sensor_msgs::ImageConstPtr& image, const float& scan_distance, cv::Mat& binary_frame) {
  if (scan_distance < 0.0f) return;  // no scan message arrived

  cv::Mat hsv_frame;

  auto image_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);

  cv::GaussianBlur(image_ptr->image, image_ptr->image, cv::Size(5, 5), 0, 0);

  cv::cvtColor(image_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_frame, cv::Scalar(kHue.first, kSaturation.first, kValue.first), cv::Scalar(kHue.second, kSaturation.second, kValue.second), binary_frame);

  auto white_pixels_num = cv::countNonZero(binary_frame);
  auto centroid = compute_centroid(binary_frame);
  auto ratio = compute_ratio(binary_frame);

  cv::putText(image_ptr->image, std::string{"Ratio: "} + std::to_string(ratio), cv::Point(5, 15),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::putText(image_ptr->image, std::string{"Distance (mean): "} + std::to_string(scan_distance), cv::Point(5, 35),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::putText(image_ptr->image, std::string{"White pixels: "} + std::to_string(white_pixels_num), cv::Point(5, 55),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::putText(image_ptr->image, std::string{"Pixels/Distance: "} + std::to_string(white_pixels_num / scan_distance), cv::Point(5, 75),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  if (centroid.x >= 0 && centroid.y >= 0) {
    cv::circle(image_ptr->image, centroid, 5, cv::Scalar(0, 0, 255), -1);   
  }

  cv::imshow(kCaptureWindow, image_ptr->image);
  cv::imshow(kHsvWindow, binary_frame);

  cv::waitKey(33);
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& scan, float& scan_distance) {
  int max_idx = std::floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

  std::vector<float> filtered_ranges;

  std::copy_if(scan->ranges.begin(), scan->ranges.end(),
   std::back_inserter(filtered_ranges), [&scan](float range) {
      return range > scan->range_min && range < scan->range_max;
   });

  float mean = 0.0f;

  if (filtered_ranges.size() < 20) {
    mean = std::accumulate(filtered_ranges.begin(), filtered_ranges.end(), 0.0) / filtered_ranges.size();
  } else {
    // distance around the middle
    auto middle_it = filtered_ranges.begin() + filtered_ranges.size() / 2;
    mean = std::accumulate(middle_it - 10, middle_it + 9, 0.0) / 20.0;
  }

  scan_distance = mean;  // update scan distance
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom, geometry_msgs::Pose& pose) {
  pose = odom->pose.pose;
}

void publish_cmd_vel(float x, float z, ros::Publisher& pub) {
  geometry_msgs::Twist cmd;
  cmd.linear.x = x;
  cmd.angular.z = z;
  pub.publish(cmd);
}

double compute_distance(double x0, double y0, double x1, double y1) {
  return std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2);
}

void turn_45deg_and_move_forward(ros::Publisher& pub, const geometry_msgs::Pose& pose) {  
  double roll, pitch, yaw;
  boost::tie(roll, pitch, yaw) = rpy_from(pose.orientation);

  double desired_yaw = yaw + M_PI_4;
  
  ROS_INFO("Reference yaw: %.6f, target: %.6f", yaw, desired_yaw);

  ros::Rate rate(60);

  while(std::abs(boost::get<2>(rpy_from(pose.orientation)) - desired_yaw) > 0.17f) {  // ~10 degrees accuracy
    publish_cmd_vel(0.0f, 0.1f, pub);

    ROS_INFO("Current yaw: %.6f", boost::get<2>(rpy_from(pose.orientation)));

    ros::spinOnce();
  }

  double ref_x = pose.position.x;
  double ref_y = pose.position.y;

  double desired_distance = 8.0f;  // meters^2

  ROS_INFO("Reference (x, y) = (%.6f, %.6f)", ref_x, ref_y);

  while (std::abs(compute_distance(ref_x, ref_y, pose.position.x, pose.position.y) - desired_distance) > 0.4f)  {
    publish_cmd_vel(0.1f, 0.0f, pub);

    ROS_INFO("Current position (x, y) = (%.3f, %3.f)", pose.position.x, pose.position.y);

    ros::spinOnce();
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "algorithm_node");

  ros::NodeHandle nh;

  float scan_distance = -1.0f;
  geometry_msgs::Pose pose;
  cv::Mat binary_frame;

  ImageSubscriber img_subscriber(nh, kImageRawTopic, boost::bind(&image_callback, _1, boost::cref(scan_distance), boost::ref(binary_frame)));
  LaserSubscriber laser_subscriber(nh, kLaserScanTopic, boost::bind(&laser_callback, _1, boost::ref(scan_distance)));

  auto odom_subscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 3, boost::bind(&odom_callback, _1, boost::ref(pose)));
  auto cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(kCmdVelocityTopic, 1);

  cv::namedWindow(kCaptureWindow);
  cv::namedWindow(kHsvWindow);

  ros::Rate rate(50);

  while(nh.ok()) {

    if (!binary_frame.empty() && (scan_distance > 0.0f || std::isnan(scan_distance))) {
      auto centroid = compute_centroid(binary_frame);
      auto white_pixels_num = cv::countNonZero(binary_frame);

      float err = centroid.x - binary_frame.cols / 2;

      if (std::abs(err) > 50 || white_pixels_num < 400) {
        ROS_INFO("Searching for the container ...");

        if (white_pixels_num < 400) {
          publish_cmd_vel(0.0f, -0.15f, cmd_vel_publisher);
        } else {
          publish_cmd_vel(0.0f, err < 0.0f? 0.15f : -0.15f, cmd_vel_publisher);          
        }

      } else {
        if (std::isinf(scan_distance)) ROS_WARN("INFINITY distance is detected");

        // approach the container (to 2-3 meters from it)
        if (scan_distance > 2.0f) {
          publish_cmd_vel(0.15f, 0.0f, cmd_vel_publisher);
        } else {
          auto ratio = compute_ratio(binary_frame);
          auto pixel_distance_ratio = white_pixels_num / scan_distance;
          bool is_large_side = false;

          if (ratio > 1.4f) {
            is_large_side = true;

          } else if (ratio > 1.2f && pixel_distance_ratio > 20 * 1E3f) {
            is_large_side = true;

          } else {
            is_large_side = false;
          }

          ROS_INFO("%s", is_large_side? "Large" : "Small");

          if (is_large_side && scan_distance > 1.03f) {
            ROS_INFO("Approaching container ...");
            publish_cmd_vel(0.1f, 0.0f, cmd_vel_publisher);
          }

          if (!is_large_side) {
            // turn 90 degrees and move forward for 2 meters
            turn_45deg_and_move_forward(cmd_vel_publisher, pose);
          }
        }
      }

    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}