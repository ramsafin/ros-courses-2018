#include "wander_bot/ForwardRotateExplorer.hpp"

#include <geometry_msgs/Twist.h>
#include <functional>
#include <numeric>

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
	
	ROS_INFO("Move forward");

	msg.linear.x = FORWARD_SPEED;
	commandPub.publish(msg);
}

void ForwardRotateExplorer::rotateCounterClockwise() {
	geometry_msgs::Twist msg;
	msg.angular.z = ROTATE_SPEED;
	commandPub.publish(msg);
}

void ForwardRotateExplorer::rotateToFreeSpace() {
	geometry_msgs::Twist msg;

	ROS_INFO("Rotate to free space");

	if (rotationSide_ == LEFT_SIDE) {
		msg.angular.z = ROTATE_SPEED;
	} else {
		msg.angular.z = -ROTATE_SPEED;
	}

	commandPub.publish(msg);
}

int ForwardRotateExplorer::findFreeSpace(const sensor_msgs::LaserScan::ConstPtr& scan) {
	int maxIndex = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

	// compute and compare halves densities

	auto filteredSum = [&scan](float sum, float range) {
                         return range <= scan->range_max && range >= scan->range_min? sum + range : sum;
                       };

    auto rigthSideSum = std::accumulate(std::begin(scan->ranges), std::begin(scan->ranges) + int(maxIndex / 2), 0, filteredSum);
    auto leftSideSum = std::accumulate(std::begin(scan->ranges) + int(maxIndex / 2), std::end(scan->ranges), 0, filteredSum);

    // all measurements are NaN or Infinity
    if (leftSideSum == 0 || rigthSideSum == 0) {
    	return leftSideSum == 0 ? LEFT_SIDE : RIGHT_SIDE;
    }

    ROS_INFO("Left: %d, Right: %d", leftSideSum, rigthSideSum);

    return rigthSideSum > leftSideSum ? RIGHT_SIDE : LEFT_SIDE;
}


void ForwardRotateExplorer::onLaserScanData(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (isObstacleInFront(scan)) {
		if (!isRotating_) {
			isRotating_ = true;
			rotationSide_ = findFreeSpace(scan);
		}

		rotateToFreeSpace();

		// rotateCounterClockwise();
		
	} else {
		isRotating_ = false;  // reset flag
		moveForward();
	}
}

bool ForwardRotateExplorer::isObstacleInFront(const sensor_msgs::LaserScan::ConstPtr& scan) {
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	for (auto currIndex = minIndex + 1; currIndex <= maxIndex; ++currIndex) {
	    if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
	    	return true;
	    }
	}
	return false;
}
