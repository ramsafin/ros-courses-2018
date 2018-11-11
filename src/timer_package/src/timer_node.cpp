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

#include <ros/ros.h>
#include <ros/time.h>
#include <cstdlib>

static constexpr auto DEFAULT_FREQUENCY_HZ = 2;

int main(int argc, char **argv) {
  ros::init(argc, argv, "timer_node");

  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandle_("~");

  int frequency;
  nodeHandle_.param<int>("frequency", frequency, DEFAULT_FREQUENCY_HZ);

  ROS_INFO_STREAM("timer_node has started with " << frequency << "Hz frequency");

  ros::Rate loop_rate(frequency);

  while (ros::ok()) {
    // prints to /rosout
    ROS_WARN_STREAM(ros::Time::now());

    ros::spinOnce();

    loop_rate.sleep(); 
  }

  return 0;
}
