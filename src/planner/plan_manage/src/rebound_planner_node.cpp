#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/rebo_replan_fsm.h>

using namespace rebound_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rebound_planner_node");
  ros::NodeHandle nh("~");

  ReboReplanFSM rebo_replan;

  rebo_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
