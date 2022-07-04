#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mavros_msgs/PositionTarget.h>

ros::Publisher setpoint_raw_pub;
Eigen::Vector3d odom_pos_, odom_vel_; // odometry state

ros::Publisher pos_cmd_pub, ref_target_pub;

struct rc_input_t {
    /* meaning axis, -1 ~ 1 value */
    double roll;
    double pitch;
    double yawrate;
    double throttle;
} rc_input;

bool have_odom_ = false;
Eigen::Vector3d last_odom_pos_(0, 0, 0);

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
bool use_velocity_control_;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
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

void customTrajCallback(visualization_msgs::MarkerConstPtr msg)
{
  if (msg->type == visualization_msgs::Marker::SPHERE_LIST)
  {
    if (msg->points.size() < 2) return;

    Eigen::MatrixXd pos_pts(3, msg->points.size());

    for (size_t i = 0; i < msg->points.size(); i++)
    {
      pos_pts(0, i) = msg->points[i].x;
      pos_pts(1, i) = msg->points[i].y;
      pos_pts(2, i) = msg->points[i].z;
    }
    
    UniformBspline pos_traj(pos_pts, 3, 1.0);

    start_time_ = ros::Time::now();

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;
  }
}

Eigen::Vector3d findClosestPoint(Eigen::Vector3d point, ego_planner::UniformBspline &bspline, double l_offset = 0)
{
    const double tol = 0.01, dt = tol * 0.49;
    double t_start = 0, t_end = bspline.getTimeSum();

    if (t_end <= 0) return point;
    
    while (t_end - t_start > tol)
    {
        double t_cur = (t_start + t_end) / 2.0;
        Eigen::Vector3d pos1 = bspline.evaluateDeBoorT(t_cur);
        Eigen::Vector3d pos2 = bspline.evaluateDeBoorT(t_cur + dt);
        double dist1 = (point - pos1).norm();
        double dist2 = (point - pos2).norm();
        if (dist1 < dist2)
            t_end = t_cur;
        else
            t_start = t_cur;
    }

    t_end = bspline.getTimeSum();

    double length = 0.0, res = 0.01;
    Eigen::Vector3d p_l = bspline.evaluateDeBoorT(t_start), p_n;
    for (double t = t_start; t <= t_end; t += res)
    {
      p_n = bspline.evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
      if (length >= l_offset) break;
    }

    return p_l;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.coordinate_frame = 1;

  // ignore velocity, acceleration, force, and yawrate reference
  msg.type_mask = msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ;
  msg.type_mask |= msg.FORCE;
    
  msg.position.x = last_odom_pos_.x();
  msg.position.y = last_odom_pos_.y();
  msg.position.z = last_odom_pos_.z();

  /* check receive traj_ before calculate target */
  if (receive_traj_) {
    Eigen::Vector3d closestPoint = findClosestPoint(odom_pos_, traj_[0]);
    Eigen::Vector3d refTarget = findClosestPoint(closestPoint, traj_[0], 1.0);
    Eigen::Vector3d refTarget_forward = findClosestPoint(closestPoint, traj_[0], 2.5);
    Eigen::Vector3d sub_vector(refTarget.x() - odom_pos_.x(), refTarget.y() - odom_pos_.y(), 0);
    double ref_yaw = atan2(refTarget_forward.y() - odom_pos_.y(), refTarget_forward.x() - odom_pos_.x());

    // visualize target
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = 0;
      marker.pose.position.x = refTarget.x();
      marker.pose.position.y = refTarget.y();
      marker.pose.position.z = refTarget.z();
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      ref_target_pub.publish(marker);
    }

    if (!use_velocity_control_) {
      msg.type_mask = msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ;

      msg.position.x = refTarget.x();
      msg.position.y = refTarget.y();
      msg.position.z = refTarget.z();

    } else {
      msg.type_mask |= msg.IGNORE_PX | msg.IGNORE_PY | msg.IGNORE_PZ;

      double alpha = 0.5;
      double max_velocity = 1.0;

      Eigen::Vector3d refVel = alpha * (refTarget - odom_pos_) + (1.0 - alpha) * (closestPoint - odom_pos_);
      refVel = refVel * (min(sub_vector.norm(), max_velocity) / refVel.norm());

      msg.velocity.x = refVel.x();
      msg.velocity.y = refVel.y();
      msg.velocity.z = refVel.z();
    }

    if (sub_vector.norm() > 0.5) {
      msg.type_mask |= msg.IGNORE_YAW_RATE;
      msg.yaw = ref_yaw;

      setpoint_raw_pub.publish(msg);
      return;

    } else if (sub_vector.norm() < 0.1) {
      receive_traj_ = false;
      last_odom_pos_ = odom_pos_;

    } else {
      last_odom_pos_ = odom_pos_;
    }
  }

  if (abs(rc_input.yawrate) > 0.2) {
    msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ | msg.IGNORE_YAW;
    msg.yaw_rate = rc_input.yawrate;
  } else {
    msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ | msg.IGNORE_YAW | msg.IGNORE_YAW_RATE;
  }

  if (abs(rc_input.throttle) > 0.2) {
    msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ;
    msg.position.z = odom_pos_.z() + rc_input.throttle * 0.5;
    last_odom_pos_.z() = odom_pos_.z();
  }

  setpoint_raw_pub.publish(msg);
  return;

  // ros::Time time_now = ros::Time::now();
  // double t_cur = (time_now - start_time_).toSec();

  // Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  // std::pair<double, double> yaw_yawdot(0, 0);

  // static ros::Time time_last = ros::Time::now();
  // if (t_cur < traj_duration_ && t_cur >= 0.0)
  // {
  //   pos = traj_[0].evaluateDeBoorT(t_cur);
  //   vel = traj_[1].evaluateDeBoorT(t_cur);
  //   acc = traj_[2].evaluateDeBoorT(t_cur);

  //   /*** calculate yaw ***/
  //   yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
  //   /*** calculate yaw ***/

  //   double tf = min(traj_duration_, t_cur + 2.0);
  //   pos_f = traj_[0].evaluateDeBoorT(tf);
  // }
  // else if (t_cur >= traj_duration_)
  // {
  //   /* hover when finish traj_ */
  //   pos = traj_[0].evaluateDeBoorT(traj_duration_);
  //   vel.setZero();
  //   acc.setZero();

  //   yaw_yawdot.first = last_yaw_;
  //   yaw_yawdot.second = 0;

  //   pos_f = pos;
  // }
  // else
  // {
  //   cout << "[Traj server]: invalid time." << endl;
  // }
  // time_last = time_now;

  // cmd.header.stamp = time_now;
  // cmd.header.frame_id = "map";
  // cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  // cmd.trajectory_id = traj_id_;

  // cmd.position.x = pos(0);
  // cmd.position.y = pos(1);
  // cmd.position.z = pos(2);

  // cmd.velocity.x = vel(0);
  // cmd.velocity.y = vel(1);
  // cmd.velocity.z = vel(2);

  // cmd.acceleration.x = acc(0);
  // cmd.acceleration.y = acc(1);
  // cmd.acceleration.z = acc(2);

  // cmd.yaw = yaw_yawdot.first;
  // cmd.yaw_dot = yaw_yawdot.second;

  // last_yaw_ = cmd.yaw;

  // pos_cmd_pub.publish(cmd);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  if (!have_odom_) {
    have_odom_ = true;
    last_odom_pos_ = odom_pos_;
  }
}

std::vector<std::string> split(std::string str)
{
    std::string temp = "";
    std::vector<std::string> ret;
    for (auto x : str) {
        if (x == ',') {
            ret.push_back(temp);
            temp = "";
        }
        else {
            temp = temp + x;
        }
    }

    ret.push_back(temp);
    return ret;
}

void rc_demo_callback(const std_msgs::String& msg) {
    auto splited = split(msg.data);
    rc_input.roll = std::stod(splited[0]);
    rc_input.pitch = std::stod(splited[1]);
    rc_input.yawrate = std::stod(splited[2]);
    rc_input.throttle = std::stod(splited[3]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber trajlist_sub = node.subscribe("/ego_planner_node/traj_list", 10, customTrajCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 10, odom_callback);
  ros::Subscriber rc_demo_sub =node.subscribe("rc_demo", 10, rc_demo_callback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  setpoint_raw_pub = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
  ref_target_pub = node.advertise<visualization_msgs::Marker>("/ref_target", 1);

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

  rc_input.roll = 0;
  rc_input.pitch = 0;
  rc_input.yawrate = 0;
  rc_input.throttle = 0;

  nh.param("traj_server/use_velocity_control", use_velocity_control_, false);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}