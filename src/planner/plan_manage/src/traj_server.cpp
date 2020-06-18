#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "rebound_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
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

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
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
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

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