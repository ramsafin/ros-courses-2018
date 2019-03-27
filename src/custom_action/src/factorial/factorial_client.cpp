#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <custom_action/FactorialAction.h>

constexpr auto kNodeName = "fibonacci_client";  
constexpr int kFibonacciOrder = 10;
constexpr double kResultWaitSeconds = 30.0;

using ActionClient = actionlib::SimpleActionClient<custom_action::FactorialAction>;

int main (int argc, char **argv) {
  ros::init(argc, argv, kNodeName);

  auto spin_on_separate_thread = true;
  const auto action_client_topic_prefix = "factorial";

  ActionClient ac(action_client_topic_prefix, spin_on_separate_thread);

  ROS_INFO("Waiting for action server to start...");

  ac.waitForServer();

  ROS_INFO("Action server has started, sending goal...");

  custom_action::FactorialGoal goal;
  goal.order = kFibonacciOrder;

  ac.sendGoal(goal);

  // wait for the action to return
  if (ac.waitForResult(ros::Duration(kResultWaitSeconds))) {
    // retrieve action's state
    actionlib::SimpleClientGoalState state = ac.getState();

    // SUCCESS
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
