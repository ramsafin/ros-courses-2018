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

static constexpr int FREQUENCY_HZ = 1;

int main(int argc, char ** argv) {

  // init the 'talker' node
  ros::init(argc, argv, "talker");

  ros::NodeHandle node;
  ros::NodeHandle nh_("~");  // node handle to obtain local parameters

  int frequency;
  nh_.param<int>("frequency", frequency, FREQUENCY_HZ);

  ROS_INFO_STREAM("talker node has started with " << frequency << "Hz frequency");

  // create a topic '/chat' where messages will be published
  // the 2nd parameter defines the outoging messages queue size 
  ros::Publisher chat_pub = node.advertise<std_msgs::String>("chat", 25);

  ros::Rate loop_rate(frequency);

  // used to count message id (number of messages)
  int count = 0;

  while (ros::ok()) {

    // check if there is at least one subscriber to the '/chat'
    if (chat_pub.getNumSubscribers() > 0) {

      std_msgs::String msg;

      msg.data = std::to_string(count) + std::string(" - How's it going?");

      ROS_INFO("[%s] => %s", ros::this_node::getName().c_str(), msg.data.c_str());

      // publish the message
      chat_pub.publish(msg);

      ++count;
    }

    // take time to process callbacks
    ros::spinOnce();

    // sleep for a necessary amount of time,
    // so the frequency of publishing is not less than 1 Hz
    loop_rate.sleep();
  }

  return 0;
}
