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
