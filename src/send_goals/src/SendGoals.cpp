#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goals_node");

    double x, y, theta; // Get the goal's x, y and angle from the launch file
    ros::NodeHandle nh;
    nh.getParam("goal_x", x);
    nh.getParam("goal_y", y);
    nh.getParam("goal_theta", theta);

    MoveBaseClient ac("move_base", true); // create the action client

    ROS_INFO("Waiting for the move_base action server");

    ac.waitForServer(ros::Duration(60)); // Wait 60 seconds for the action server to become available
   
    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // Convert the Euler angle to quaternion
    double radians = theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    goal.target_pose.pose.orientation = qMsg;

    // Send the goal command
    ROS_INFO("Sending robot to: x = %f, y = %f, theta = %f", x, y, theta);

    ac.sendGoal(goal);

    // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the goal!");
    else
        ROS_INFO("The base failed for some reason");

    return 0;
}
