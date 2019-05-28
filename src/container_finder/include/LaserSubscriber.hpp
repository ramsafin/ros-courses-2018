#pragma once

#include <cmath>
#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserSubscriber final {
  public:
    using Callback = std::function<void(const sensor_msgs::LaserScanConstPtr&)>;

    LaserSubscriber(ros::NodeHandle& nh, const std::string topic, const Callback& callback) {
      laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(topic, 1, callback);
    }

  private:
    ros::Subscriber laser_sub_;
};
