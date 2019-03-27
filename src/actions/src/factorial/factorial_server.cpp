#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actions/FactorialAction.h>

constexpr auto kNodeName = "factorial_server";

using ActionServer = actionlib::SimpleActionServer<actions::FactorialAction>;

class FactorialAction {
  protected:
    ros::NodeHandle node_handle_;
    ActionServer action_server_;
    
    std::string action_name_;
    
    actions::FactorialFeedback action_feedback_;
    actions::FactorialResult action_result_;

  public:
    FactorialAction(const std::string& name) : action_name_(name),
      action_server_(node_handle_, name, boost::bind(&FactorialAction::execute_callback, this, _1), false) {
      action_server_.start();
    }

  void execute_callback(const actions::FactorialGoalConstPtr &goal) {
    ros::Rate r(50);
    bool success = true;

    action_feedback_.sequence.clear();
    action_feedback_.sequence.push_back(1);  // 0!
    action_feedback_.sequence.push_back(1);  // 1!

    ROS_INFO("%s: Executing, creating factorial sequence of order %i...", action_name_.c_str(), goal->order);

    for(int i = 1; i <= goal->order; ++i) {
      // check that preempt has not been requested by the client
      if (action_server_.isPreemptRequested() || !node_handle_.ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        action_server_.setPreempted();
        success = false;
        break;  // finish computing factorial
      }

      // compute next element
      action_feedback_.sequence.push_back(action_feedback_.sequence[i] * (i + 1));
      action_server_.publishFeedback(action_feedback_);
      r.sleep();
    }

    if(success) {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      action_result_.factorial = action_feedback_.sequence.back();
      action_server_.setSucceeded(action_result_);
    }
  }

};


int main(int argc, char** argv) {
  ros::init(argc, argv, kNodeName);

  const auto action_prefix = "factorial";

  FactorialAction factorial(action_prefix);

  ros::spin();  // blocking

  return 0;
}

