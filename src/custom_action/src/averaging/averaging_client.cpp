#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <custom_action/AveragingAction.h>
#include <thread>

constexpr auto kNodeName = "averaging_client";

using ActionClient = actionlib::SimpleActionClient<custom_action::AveragingAction>;

void spinThread() {
  ros::spin();
}

int main (int argc, char **argv) {
  ros::init(argc, argv, kNodeName);

  const auto action_client_prefix = "averaging";

  ActionClient action_client(action_client_prefix);

  // boost::thread spin_thread(&spinThread);
  std::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start...");

  action_client.waitForServer();

  ROS_INFO("Action server started, Sending goal...");
  
  custom_action::AveragingGoal goal;
  goal.samples = 100;

  action_client.sendGoal(goal);

  ROS_INFO("Goal has been sent.");

  bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = action_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
