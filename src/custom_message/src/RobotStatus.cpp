#include <ros/ros.h>
#include <string>

#include "custom_message/RobotStatus.h"

static constexpr auto STATUS_PUBLISHING_RATE = 1;

static constexpr auto HIGH_DURABILITY_LEVEL_SEC = 60;
static constexpr auto AVERAGE_DURABILITY_LEVEL_SEC = 45;
static constexpr auto LOW_DURABILITY_LEVEL_SEC = 30;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "robot_status_node");
    
    ros::NodeHandle node;
    ros::NodeHandle node_("~");

    // read parameters
    
    int robotId;
    std::string durabilityLevel;  // high, average, low

    node_.param("robot_id", robotId, -1);
    node_.param("durability_level", durabilityLevel, std::string{"average"});

    auto publisher = node.advertise<custom_message::RobotStatus>("status", 5);

    custom_message::RobotStatus statusMsg;
    statusMsg.robot_id = robotId;

    ros::Rate rate(STATUS_PUBLISHING_RATE);

    ROS_INFO("Starting to pubish robot status messages");

    auto startTime = ros::Time::now();

    double secondsTillDeath;
    
    if (durabilityLevel == "high") {
        secondsTillDeath = HIGH_DURABILITY_LEVEL_SEC;
    } else if (durabilityLevel == "average") {
        secondsTillDeath = AVERAGE_DURABILITY_LEVEL_SEC;
    } else {
        secondsTillDeath = LOW_DURABILITY_LEVEL_SEC;
    }

    while (node.ok()) {
      statusMsg.header.stamp = ros::Time::now();
      
      if ((statusMsg.header.stamp - startTime).toSec() > secondsTillDeath) {
        statusMsg.is_ok = false;
      } else {
        statusMsg.is_ok = true;
      }

      publisher.publish(statusMsg);
      rate.sleep();
    }

    return 0;
}
