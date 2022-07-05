// #include <iostream>
#include <string>
#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <tf2/LinearMath/Quaternion.h>

using namespace std;

ros::ServiceClient set_model_state_client;
ros::Time last_time;

double topic_dt_ = 1.0 / 240.0; 

void movingCallback(const ros::TimerEvent & /*event*/)
{
	ros::Time now = ros::Time::now();
	double dt = (now - last_time).toSec();
    last_time = now;
    
    gazebo_msgs::SetModelState moving_target;
    moving_target.request.model_state.model_name = "irlock_beacon";
    moving_target.request.model_state.pose.position.x = 0;
    moving_target.request.model_state.pose.position.y = 0;
    moving_target.request.model_state.pose.position.z = 0;
    moving_target.request.model_state.twist.linear.x = 0;
    moving_target.request.model_state.twist.linear.y = 0;
    moving_target.request.model_state.twist.linear.z = 0;

    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 0, ros::Time::now().toSec());
    moving_target.request.model_state.pose.orientation.x = tf_q.x();
    moving_target.request.model_state.pose.orientation.y = tf_q.y();
    moving_target.request.model_state.pose.orientation.z = tf_q.z();
    moving_target.request.model_state.pose.orientation.w = tf_q.w();
    
    moving_target.request.model_state.reference_frame = "world";

    set_model_state_client.call(moving_target);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moving_target");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Timer moving_timer = nh.createTimer(ros::Duration(0.05), &movingCallback);

    last_time = ros::Time::now();
    ros::spin();

    return 0;
}
