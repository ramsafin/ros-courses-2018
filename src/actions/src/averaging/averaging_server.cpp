#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actions/AveragingAction.h>
#include <cmath>

constexpr auto kNodeName = "averaging";
constexpr double kMeanThreshold = 5.0;

using ActionServer = actionlib::SimpleActionServer<actions::AveragingAction>;

class AveragingAction {
public:

  AveragingAction(std::string name) : action_server_(nh_, name, false), action_name_(name) {
    action_server_.registerGoalCallback(boost::bind(&AveragingAction::goal_callback, this));
    action_server_.registerPreemptCallback(boost::bind(&AveragingAction::preempt_callback, this));

    random_number_subscriber_ = nh_.subscribe("/random_number", 1, &AveragingAction::analysis_callback, this);

    action_server_.start();
  }

  void goal_callback() {
    ROS_INFO("Goal has been recieved!");

    this->data_count_ = 0;
    this->sum_ = 0;
    this->sum_sq_ = 0;

    // accept new goal
    this->goal_ = action_server_.acceptNewGoal()->samples;
  }

  void preempt_callback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());

    action_server_.setPreempted();
  }

  void analysis_callback(const std_msgs::Float32::ConstPtr& msg) {
    if (!action_server_.isActive()) return;

    this->data_count_++;
    feedback_.sample_num = data_count_;
    feedback_.data = msg->data;

    //compute the std_dev and mean of the data
    this->sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;

    this->sum_sq_ += std::pow(msg->data, 2);
    feedback_.std_dev = std::sqrt(std::fabs((sum_sq_/data_count_) - std::pow(feedback_.mean, 2)));
    
    action_server_.publishFeedback(feedback_);

    if(data_count_ > goal_) {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < kMeanThreshold) {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        action_server_.setAborted(result_);

      } else {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        action_server_.setSucceeded(result_);
      }
    }
  }

protected:
  ros::NodeHandle nh_;
  ActionServer action_server_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  actions::AveragingFeedback feedback_;
  actions::AveragingResult result_;
  ros::Subscriber random_number_subscriber_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, kNodeName);

  const auto action_server_prefix = "averaging";

  AveragingAction averaging(action_server_prefix);

  ros::spin();

  return 0;
}
