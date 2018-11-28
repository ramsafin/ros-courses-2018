#include "wander_bot/ForwardRotateExplorer.hpp"

#include <geometry_msgs/Twist.h>

ForwardRotateExplorer::ForwardRotateExplorer() {
commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
laserSub = node.subscribe("scan", 1, &ForwardRotateExplorer::onLaserScanData, this);
}

void ForwardRotateExplorer::startMoving() {
	ros::Rate rate(10);
ROS_INFO("Start moving");

while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
}
}

void ForwardRotateExplorer::moveForward() {
geometry_msgs::Twist msg;
msg.linear.x = FORWARD_SPEED;
commandPub.publish(msg);
}

void ForwardRotateExplorer::rotateCounterClockwise() {
	geometry_msgs::Twist msg;
	msg.angular.z = ROTATE_SPEED;
	commandPub.publish(msg);
}

void ForwardRotateExplorer::stop() {
	geometry_msgs::Twist msg;
	commandPub.publish(msg);
}

void ForwardRotateExplorer::onLaserScanData(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (isObstacleInFront(scan)) {
		rotateCounterClockwise();
	} else {
		moveForward();
	}
}

bool ForwardRotateExplorer::isObstacleInFront(const sensor_msgs::LaserScan::ConstPtr& scan) {
	// Find the closest range between the defined minimum and maximum angles
int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

for (auto currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
    if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
    	return true;
    }
}
return false;
}
