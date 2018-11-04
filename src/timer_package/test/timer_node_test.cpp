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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

TEST(TestCase, shouldPass) {
	// actually it is possible to subscribe to the topics
	// and fully use ros features here
	EXPECT_EQ(1, 1) << "1 should equal to 1";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "timer_node_test");
  ::testing::InitGoogleTest(&argc, argv);

  std::thread t([]{ while(ros::ok()) ros::spinOnce(); });
  
  auto res = RUN_ALL_TESTS();
  
  ros::shutdown();
  
  return res;
}