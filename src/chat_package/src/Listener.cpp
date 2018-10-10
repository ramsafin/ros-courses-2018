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

/**
 * @brief Callback function.
 * 
 * Acts as a hadnler method for new messages
 * coming from the topic one subscribed.
 * 
 * @param msg - standard ROS message representing a string. 
 */
void talkerCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("<= I heard: [%s]", msg->data.c_str());
}

int main(int argc, char ** argv) {

	// init the 'listener' node
	ros::init(argc, argv, "listener");

	ros::NodeHandle node;

	// subscribe to the '/chat' topic (message queue size: 50)
	// additionally we can define a callback function for every incoming message
	ros::Subscriber sub = node.subscribe("chat", 50, talkerCallback);

	ros::spin(); // will not return (blocking)

	return 0;
}
