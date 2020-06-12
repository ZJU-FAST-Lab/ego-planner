#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "rebound_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;

nav_msgs::Odometry odom;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using rebound_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void bsplineCallback(rebound_planner::BsplineConstPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  // traj_.push_back(yaw_traj);
  // traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void cmdCallback(const ros::TimerEvent& e) 
{
  /* no publishing before receive traj_ */
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw = 0, yawdot = 0;

  static ros::Time time_last = ros::Time::now();
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  if (t_cur < traj_duration_ && t_cur >= 0.0) 
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur+time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2( dir(1), dir(0) ) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC*(time_now-time_last).toSec();
    if ( yaw_temp - last_yaw_ > PI )
    {
      if ( yaw_temp - last_yaw_ - 2*PI < -max_yaw_change )
      {
        yaw = last_yaw_ - max_yaw_change;
        if ( yaw < -PI ) 
          yaw += 2*PI;
        
        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if ( yaw - last_yaw_ > PI )
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now-time_last).toSec();
      }
      
    }
    else if ( yaw_temp - last_yaw_ < -PI )
    {
      if ( yaw_temp - last_yaw_ + 2*PI > max_yaw_change )
      {
        yaw = last_yaw_ + max_yaw_change;
        if ( yaw > PI ) 
          yaw -= 2*PI;
        
        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if ( yaw - last_yaw_ < -PI )
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now-time_last).toSec();
      }
      
    }
    else 
    {
      if ( yaw_temp - last_yaw_ < -max_yaw_change )
      {
        yaw = last_yaw_ - max_yaw_change;
        if ( yaw < -PI ) 
          yaw += 2*PI;
        
        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else if ( yaw_temp - last_yaw_ > max_yaw_change )
      {
        yaw = last_yaw_ + max_yaw_change;
        if ( yaw > PI ) 
          yaw -= 2*PI;
        
        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if ( yaw - last_yaw_ > PI )
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else if ( yaw - last_yaw_ < -PI )
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now-time_last).toSec();
      }
    }

    if ( fabs( yaw-last_yaw_ ) <= max_yaw_change )
      yaw = 0.5*last_yaw_ + 0.5*yaw; // nieve LPF
    yawdot = 0.5*last_yaw_dot_ + 0.5*yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw = last_yaw_;
    yawdot = 0;

    pos_f = pos;

  } else {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}