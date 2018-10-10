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

int main(int argc, char **argv) {

  // by default the frequency of printing something out is 2 Hz (2 message per second)
  int frequency = 2;

  if (argc == 2) {
      // try parsing CLI defined frequency
    int arg_frequency = std::atoi(argv[1]);

    if (arg_frequency > 0) frequency = arg_frequency;
  }

  ros::init(argc, argv, "timer_node");

  ROS_INFO_STREAM("timer_node has started with " << frequency << "Hz frequency");

  ros::NodeHandle nodeHandle;

  // 2 Hz frequency
  ros::Rate loop_rate(frequency);

  while (ros::ok()) {

    ROS_INFO_STREAM(ros::Time::now()); // print out current time

    ros::spinOnce();

    loop_rate.sleep(); 
  }

  return 0;
}