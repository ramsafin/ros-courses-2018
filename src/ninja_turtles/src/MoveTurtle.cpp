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

#include <cmath>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

/* constants */
static auto const DEFAULT_MOVING_PATTERN = std::string{"forward_turn"};

// measurements accuracy
static constexpr auto EPS = 10E-3;

// PI constants
static constexpr auto PI = M_PI;
static constexpr auto PI_2 = M_PI_2;
static constexpr auto PI_4 = M_PI_4;

// checks if the given values are close to each other
bool is_near(double value, double otherValue) {
  return std::abs(value - otherValue) < EPS;
}

// checks if the given value is in the specified range
bool is_in_range(double value, double start, double end) {
  return value >= start && value <= end;
}

// possible orientations of the turtle (see turtlesim Pose)
enum class TurtleOrientation : uint8_t {
  NORTH,
  SOUTH,
  EAST,
  WEST,
  NORTH_EAST,
  NORTH_WEST,
  SOUTH_EAST,
  SOUTH_WEST
};

// finds the turtle's orientation given its pose
TurtleOrientation find_orientation(turtlesim::PoseConstPtr const &pose) {
  if (is_in_range(pose->theta, 0, PI_2) || is_in_range(pose->theta, -2 * PI, -PI - PI_2)) {
    // 1st quarter
    if (is_near(pose->theta, 0) || is_near(pose->theta, -2 * PI)) return TurtleOrientation::EAST;
    if (is_near(pose->theta, PI_2) || is_near(pose->theta, -PI - PI_2)) return TurtleOrientation::NORTH;
    return TurtleOrientation::NORTH_EAST;
  } else if (is_in_range(pose->theta, PI_2, PI) || is_in_range(pose->theta, -PI - PI_2, -PI)) {
    // 2nd quarter
    if (is_near(pose->theta, PI_2) || is_near(pose->theta, -PI - PI_2)) return TurtleOrientation::NORTH;
    if (is_near(pose->theta, PI) || is_near(pose->theta, -PI)) return TurtleOrientation::WEST;
    return TurtleOrientation::NORTH_WEST;
  } else if (is_in_range(pose->theta, PI, PI + PI_2) || is_in_range(pose->theta, -PI, -PI_2)) {
    // 3nd quarter
    if (is_near(pose->theta, PI + PI_2) || is_near(pose->theta, -PI_2)) return TurtleOrientation::SOUTH;
    if (is_near(pose->theta, PI) || is_near(pose->theta, -PI)) return TurtleOrientation::WEST;
    return TurtleOrientation::SOUTH_WEST;
  } else {
    // 4th quarter
    if (is_near(pose->theta, 0) || is_near(pose->theta, -2 * PI)) return TurtleOrientation::EAST;
    if (is_near(pose->theta, PI + PI_2) || is_near(pose->theta, -PI_2)) return TurtleOrientation::SOUTH;
    return TurtleOrientation::SOUTH_EAST;
  }
}

// abstract class for turtle's movement control
class MoveTurtleBehaviour {
  public:
    virtual ~MoveTurtleBehaviour() = default;

    MoveTurtleBehaviour(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
      // create a publisher to the 'turtle1/cmd_vel' topic 
      publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

      // create a subscriber to the 'turtle1/pose' topic
      subscriber_ = nodeHandle.subscribe("turtle1/pose", 1,
                                   &MoveTurtleBehaviour::OnMessageCallback, this);

      ROS_INFO("Sucessfully launched node!");
    }

    // moves the turtle by publishing geometry_msgs/Twist message to 'turtle1/cmd_vel'
    virtual void Move(turtlesim::PoseConstPtr pose, ros::Publisher& pub) = 0;

  private:
    // handles turtle's next move by forwarding its pose to specific Move implementation
    void OnMessageCallback(const turtlesim::PoseConstPtr& pose) {
      Move(pose, publisher_);
    }

    ros::NodeHandle& nodeHandle_;
    ros::Subscriber subscriber_;  // subscribes to the 'turtle1/pose'
    ros::Publisher publisher_;  // publishes to the 'turtle1/cmd_vel
};

class MoveTurtleForwardAndTurn final : public MoveTurtleBehaviour {
  public:
    MoveTurtleForwardAndTurn(ros::NodeHandle &nodeHandle) 
                  : MoveTurtleBehaviour(nodeHandle), moveForward(true) {}

    // moves turtle forward and then turns for 45 degrees counter-clockwise
    void Move(turtlesim::PoseConstPtr pose, ros::Publisher& pub) override {
      geometry_msgs::Twist nextMove;

      if (!initialPose_) {  // we just get started
        initialPose_ = pose;
        ROS_INFO("Initial position x: %.6f y: %.6f", initialPose_->x, initialPose_->y);
      } else if (moveForward) {  // move forward
        if (!is_near(std::abs(initialPose_->x - pose->x), FORWARD_DISTANCE_M) ) {
          nextMove.linear.x = 0.1;
        } else {
          ROS_INFO("Stop moving forward x: %.6f, y: %.6f", pose->x, pose->y);
          moveForward = false;
        }
      } else {  // turn 45 degrees
        if (!is_near(pose->theta, TURN_RADIANS)) {
          nextMove.angular.z = 0.02;
        } else {
          ROS_INFO("Stop turning x: %.6f, y: %.6f, theta: %.6f", pose->x, pose->y, pose->theta);
          ros::requestShutdown();
        }
      }

      pub.publish(nextMove);  // make turtle move by publishing message to the 'turtle1/cmd_vel'
    }

  private:
    turtlesim::PoseConstPtr initialPose_;

    // whether to move forward or not
    bool moveForward;

    /* constants */
    static constexpr auto FORWARD_DISTANCE_M = 1.0;
    static constexpr auto TURN_RADIANS = PI_4;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_turtle");
    
    ros::NodeHandle nodeHandle;
    ros::NodeHandle nodeHandle_("~");

    // parse node parameters

    std::string movingPattern;
    nodeHandle_.param("moving_pattern", movingPattern, DEFAULT_MOVING_PATTERN);

    // control the movement of the turtle

    std::unique_ptr<MoveTurtleBehaviour> behavior;

    if (movingPattern == "forward_turn") {
      behavior.reset(new MoveTurtleForwardAndTurn{nodeHandle});
    } else {
      ROS_ERROR_STREAM("Requested moving pattern is not supported: " << movingPattern);
      ros::requestShutdown();
    }

    ros::spin();

    return 0;
}
