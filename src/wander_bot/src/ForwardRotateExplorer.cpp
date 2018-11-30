/*
 * MIT License
 *
 * Copyright (c) 2018 Ramil Safin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @author Ramil Safin.
 */

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
	msg.linear.x = FORWARD_SPEED;
	commandPub.publish(msg);
}

void ForwardRotateExplorer::rotateToFreeSpace() {
	geometry_msgs::Twist msg;
	
	if (rotationSide_ == RotationSide::LEFT) {
		msg.angular.z = ANGULAR_SPEED;
	} else {
		msg.angular.z = -ANGULAR_SPEED;
	}

	commandPub.publish(msg);
}

RotationSide ForwardRotateExplorer::findFreeSpaceRotationSide(const sensor_msgs::LaserScan::ConstPtr& scan) {
	int maxIndex = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

	// compute and compare halves densities

	auto filteredSum = [&](float sum, float measuredRange) {
                         return isInRange(measuredRange, scan->range_min, scan->range_max)? sum + measuredRange : sum;
                       };

    auto rigthSideSum = std::accumulate(std::begin(scan->ranges), std::begin(scan->ranges) + int(maxIndex / 2), 0, filteredSum);
    auto leftSideSum = std::accumulate(std::begin(scan->ranges) + int(maxIndex / 2), std::end(scan->ranges), 0, filteredSum);

    // In case if all measurements are NaN or Infinity or any garbage values
    if (leftSideSum == 0 || rigthSideSum == 0) {
    	return leftSideSum == 0 ? RotationSide::LEFT : RotationSide::RIGHT;
    }

    ROS_INFO("Left: %d, Right: %d", leftSideSum, rigthSideSum);

    return rigthSideSum > leftSideSum ? RotationSide::RIGHT : RotationSide::LEFT;
}

void ForwardRotateExplorer::onLaserScanData(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (isObstacleInFront(scan)) {
		if (!isRotating_) {
			isRotating_ = true;
			rotationSide_ = findFreeSpaceRotationSide(scan);
		}

		rotateToFreeSpace();

	} else {
		isRotating_ = false;
		moveForward();
	}
}

bool ForwardRotateExplorer::isInRange(float value, float min, float max) {
	return value <= max && value >= min;
}

bool ForwardRotateExplorer::isObstacleInFront(const sensor_msgs::LaserScan::ConstPtr& scan) {
	// Check for obstacles in front of the robot
	int maxScanIndex = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

	for (auto currIndex = 0; currIndex <= maxScanIndex; ++currIndex) {
		if (isInRange(scan->ranges[currIndex], scan->range_min, scan->range_max)) {
			if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
	    		return true;
	    	}
		}
	}

	return false;
}
