#include <string>
#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>

#include <ros/ros.h>

#include "LaserSubscriber.hpp"

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  int maxScanIndex = std::floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

  std::vector<float> filtered_ranges;

  std::copy_if(scan->ranges.begin(), scan->ranges.end(),
   std::back_inserter(filtered_ranges), [&scan](float range) {
      return range > scan->range_min && range < scan->range_max;
   });

  // if filtered elements size is zero than it means that obstacle is really close or
  // there are no visible obstacles around
  std::cout << "Elements before: " << scan->ranges.size() << ", after: " << filtered_ranges.size() << '\n'; 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scanner");

  ros::NodeHandle nh;

  LaserSubscriber laser_sub(nh, "scan", &laser_callback);

  ros::spin();

  return 0;
}