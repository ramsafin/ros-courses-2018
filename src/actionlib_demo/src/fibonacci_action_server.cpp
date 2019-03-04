#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_demo/FibonacciAction.h>

class FibonacciAction {
  protected:
    ros::NodeHandle& nh_;
    actionlib::SimpleActionServer<actionlib_demo::FibonacciAction> action_server_;
    std::string action_name_;
    actionlib_demo::FibonacciFeedback feedback_;
    actionlib_demo::FibonacciResult result_;
  public:
  	FibonacciAction(ros::NodeHandle& nh, std::string name) 
  	  : nh_(nh), action_server_(nh, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), action_name_(name) {
  	  	action_server_.start();
  	}

  	void executeCB(const actionlib_demo::FibonacciGoalConstPtr &goal) {
  		ros::Rate rate(1);
  		bool success = true;

  		feedback_.sequence.clear();
  		feedback_.sequence.emplace_back(0);
  		feedback_.sequence.emplace_back(1);

  		ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
  			action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
  	
  		for (int i = 0; i <= goal->order; ++i) {
  			if (action_server_.isPreemptRequested() || !ros::ok()) {
  				action_server_.setPreempted();
  				success = false;
  				break;
  			}
  			feedback_.sequence.emplace_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
  			action_server_.publishFeedback(feedback_);

  			rate.sleep();
  		}

  		if (success) {
  			result_.sequence = feedback_.sequence;
  			ROS_INFO("%s: Succeeded", action_name_.c_str());
  			action_server_.setSucceeded(result_);
  		}
  	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "fibonacci");
	ros::NodeHandle nh;

	FibonacciAction fibonacci(nh, "fibonacci");

	ros::spin();
	return 0;
}
