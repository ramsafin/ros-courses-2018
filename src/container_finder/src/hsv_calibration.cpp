#include <string>
#include <cmath>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImageSubscriber.hpp"

constexpr int kSaveKey = 115;  // 'S'

// HSV thresholding
const int H_max = 180;
const int V_max = 255;

constexpr const char* kCaptureWindow = "Raw Video";
constexpr const char* kHsvWindow = "HSV Video";

int H_low = 0, S_low = 0, V_low = 0;
int H_high = H_max, S_high = V_max, V_high = V_max;

static void on_low_H_thresh_trackbar(int, void*) {
  H_low = std::min(H_high - 1, H_low);
  cv::setTrackbarPos("Low H", kHsvWindow, H_low);
}

static void on_high_H_thresh_trackbar(int, void*) {
  H_high = std::max(H_high, H_low + 1);
  cv::setTrackbarPos("High H", kHsvWindow, H_high);
}

static void on_low_S_thresh_trackbar(int, void*) {
  S_low = std::min(S_high - 1, S_low);
  cv::setTrackbarPos("Low S", kHsvWindow, S_low);
}

static void on_high_S_thresh_trackbar(int, void*) {
  S_high = std::max(S_high, S_low + 1);
  cv::setTrackbarPos("High S", kHsvWindow, S_high);
}

static void on_low_V_thresh_trackbar(int, void*) {
  V_low = std::min(V_high - 1, V_low);
  cv::setTrackbarPos("Low V", kHsvWindow, S_low);
}

static void on_high_V_thresh_trackbar(int, void*) {
  V_high = std::max(V_high, V_low + 1);
  cv::setTrackbarPos("High V", kHsvWindow, V_high);
}

void image_callback(const sensor_msgs::ImageConstPtr& image_msg) {
  cv::Mat hsv_frame, threshold_frame;

  auto image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  
  cv::cvtColor(image_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_frame, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), threshold_frame);

  int white_pixels_count = cv::countNonZero(threshold_frame);

  cv::putText(image_ptr->image, std::string{"Pixels: "} + std::to_string(white_pixels_count), cv::Point(5, 15),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255), 1);

  cv::imshow(kCaptureWindow, image_ptr->image);
  cv::imshow(kHsvWindow, threshold_frame);

  auto key = cv::waitKey(33);

  if (key == kSaveKey) {
    std::cout << "Low H: " << H_low << '\n'
              << "High H: " << H_high << '\n'
              << "Low S: " << S_low << '\n'
              << "High S: " << S_high << '\n'
              << "Low V: " << V_low << '\n'
              << "High V: " << V_high << '\n'
              << "=====================\n";
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsv_calibration");

  ros::NodeHandle nh;

  cv::namedWindow(kCaptureWindow);
  cv::namedWindow(kHsvWindow);

  cv::createTrackbar("Low H", kCaptureWindow, &H_low, H_max, on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", kCaptureWindow, &H_high, H_max, on_high_H_thresh_trackbar);

  cv::createTrackbar("Low S", kCaptureWindow, &S_low, V_max, on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", kCaptureWindow, &S_high, V_max, on_high_S_thresh_trackbar);

  cv::createTrackbar("Low V", kCaptureWindow, &V_low, V_max, on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", kCaptureWindow, &V_high, V_max, on_high_V_thresh_trackbar);

  ImageSubscriber img_subscriber(nh, "/camera/rgb/image_raw", &image_callback);

  ros::spin();

  return 0;
}