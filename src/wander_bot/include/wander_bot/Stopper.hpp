#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

class Stopper {
public:
  constexpr static auto FORWARD_SPEED = 0.5;
  constexpr static auto MIN_SCAN_ANGLE = -30.0/180*M_PI;
  constexpr static auto MAX_SCAN_ANGLE = +30.0/180*M_PI;
  constexpr static auto MIN_DIST_FROM_OBSTACLE = 0.5;

  Stopper();

  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub;
  ros::Subscriber laserSub;

  bool keepMoving;

  void moveForward();

  void onLaserScanData(const sensor_msgs::LaserScan::ConstPtr& scan);
};
