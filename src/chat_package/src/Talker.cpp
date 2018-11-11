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
#include <std_msgs/String.h>
#include <string>

static constexpr auto DEFAULT_FREQUENCY_HZ = 1;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");  // to obtain local parameters

  int frequency;
  nh_.param<int>("frequency", frequency, DEFAULT_FREQUENCY_HZ);

  ROS_INFO_STREAM("talker node has started with " << frequency << "Hz frequency");

  // create a topic '/chat' for std_msgs::String
  ros::Publisher publisher = nh.advertise<std_msgs::String>("chat", 25);

  ros::Rate loop_rate(frequency);

  int messageId = 0;

  while (nh.ok()) {
    if (publisher.getNumSubscribers() > 0) {

      std_msgs::String msg;

      msg.data = std::to_string(messageId) + std::string(" - How's it going?");

      ROS_INFO("[%s] => %s", ros::this_node::getName().c_str(), msg.data.c_str());

      // publish the message
      publisher.publish(msg);

      ++messageId;
    }

    // take time to process callbacks
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}
