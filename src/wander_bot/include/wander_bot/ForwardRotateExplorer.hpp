#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

class ForwardRotateExplorer {
public:
  constexpr static auto FORWARD_SPEED = 0.2;
  constexpr static auto ROTATE_SPEED = 0.1;
  constexpr static auto MIN_SCAN_ANGLE = -30.0/180*M_PI;
  constexpr static auto MAX_SCAN_ANGLE = +30.0/180*M_PI;
  constexpr static auto MIN_DIST_FROM_OBSTACLE = 1;

  ForwardRotateExplorer();

  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub;
  ros::Subscriber laserSub;

  void moveForward();

  void rotateCounterClockwise();

  void stop();

  void onLaserScanData(const sensor_msgs::LaserScan::ConstPtr& scan);

  bool isObstacleInFront(const sensor_msgs::LaserScan::ConstPtr& scan);
};
