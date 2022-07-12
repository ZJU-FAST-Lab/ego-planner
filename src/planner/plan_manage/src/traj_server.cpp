#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

ros::Publisher setpoint_raw_pub;
Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
double odom_yaw_ = 0;
geometry_msgs::Pose control_;

ros::Publisher pos_cmd_pub, ref_target_pub;

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
bool use_velocity_control_, enable_rotate_head_;
double forward_length_;

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
  // traj_.push_back(traj_[0].getDerivative());
  // traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void customTrajCallback(visualization_msgs::MarkerConstPtr msg)
{
  if (msg->type == visualization_msgs::Marker::SPHERE_LIST)
  {
    if (msg->points.size() == 0) {
      receive_traj_ = false;
      last_odom_pos_ = odom_pos_;
      return;
    } else if (msg->points.size() < 2) return;

    Eigen::MatrixXd pos_pts(3, msg->points.size() + 1);

    pos_pts(0, 0) = odom_pos_.x();
    pos_pts(1, 0) = odom_pos_.y();
    pos_pts(2, 0) = odom_pos_.z();

    for (size_t i = 0; i < msg->points.size(); i++)
    {
      pos_pts(0, i + 1) = msg->points[i].x;
      pos_pts(1, i + 1) = msg->points[i].y;
      pos_pts(2, i + 1) = msg->points[i].z;
    }
    
    UniformBspline pos_traj(pos_pts, 3, 1.0);

    if (pos_traj.getTimeSum() > 0 && pos_traj.getLength(0.02) > forward_length_) {
      start_time_ = ros::Time::now();
      traj_.clear();
      traj_.push_back(pos_traj);
      traj_duration_ = traj_[0].getTimeSum();
    } else {
      traj_.push_back(pos_traj);
    }
    
    // traj_.push_back(traj_[0].getDerivative());
    // traj_.push_back(traj_[1].getDerivative());

    receive_traj_ = true;
  }
}

inline double getYaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion bt = tf::Quaternion(q.x, q.y, q.z, q.w).normalized();
    return tf::getYaw(bt);
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

    if (l_offset == 0) return p_l;

    for (double t = t_start; t <= t_end; t += res)
    {
      p_n = bspline.evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
      if (length >= l_offset) break;
    }

    return p_l;
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
    Eigen::Vector3d refTarget = findClosestPoint(closestPoint, traj_[0], forward_length_);
    Eigen::Vector3d refTarget_forward = findClosestPoint(closestPoint, traj_[0], forward_length_ * 2.5);
    Eigen::Vector3d sub_vector(refTarget.x() - odom_pos_.x(), refTarget.y() - odom_pos_.y(), 0);
    
    double ref_yaw = last_yaw_;
    if (enable_rotate_head_) {
      ref_yaw = atan2(refTarget_forward.y() - odom_pos_.y(), refTarget_forward.x() - odom_pos_.x());
    }

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
    } else if (sub_vector.norm() < 0.1) {
      msg.type_mask |= msg.IGNORE_YAW;
      receive_traj_ = false;
      last_odom_pos_ = odom_pos_;
    } else {
      msg.type_mask |= msg.IGNORE_YAW;
      last_odom_pos_ = odom_pos_;
    }

  } else {
    msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ;

    Eigen::Vector3d waypoint_pose(control_.position.x, control_.position.y, 0);

    double waypoint_yaw = getYaw(control_.orientation);
    if (waypoint_yaw > M_PI) waypoint_yaw -= M_PI * 2.0;
    if (waypoint_yaw < -M_PI) waypoint_yaw += M_PI * 2.0;

    // set altitude
    if (abs(control_.position.z) > 0.1) {
      msg.position.z = odom_pos_.z() + control_.position.z * 0.5;
      last_odom_pos_.z() = odom_pos_.z();
    }

    // set yawrate
    if (abs(waypoint_yaw) > 0.1) {
      msg.type_mask |= msg.IGNORE_YAW;
      msg.yaw_rate = waypoint_yaw;
      last_yaw_ = odom_yaw_;
    } else {
      msg.type_mask |= msg.IGNORE_YAW_RATE;
      msg.yaw = last_yaw_;
    }
  }

  setpoint_raw_pub.publish(msg);
  return;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_yaw_ = getYaw(msg->pose.pose.orientation);

  if (!have_odom_) {
    have_odom_ = true;
    last_odom_pos_ = odom_pos_;
    last_yaw_ = odom_yaw_;
  }
}

void controlCallback(const geometry_msgs::PoseConstPtr &msg) {
  control_ = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber trajlist_sub = node.subscribe("/ego_planner_node/traj_list", 10, customTrajCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 10, odom_callback);
  ros::Subscriber waypoint_sub = node.subscribe("/waypoint_generator/waypoint_manual", 10, controlCallback);

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

  nh.param("traj_server/use_velocity_control", use_velocity_control_, false);
  nh.param("traj_server/forward_length", forward_length_, 1.0);
  nh.param("traj_server/enable_rotate_head", enable_rotate_head_, true);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}