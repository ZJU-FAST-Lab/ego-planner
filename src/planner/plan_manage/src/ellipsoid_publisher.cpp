#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
// #include "ego_planner/FeasibleVelocity.h"
// #include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

struct ELLIPSOID
{
    Eigen::Vector3d center;
    Eigen::Vector3d abc;
    Eigen::Matrix3d R;
    ELLIPSOID(const Eigen::Vector3d &c, const Eigen::Vector3d &r,const Eigen::Matrix3d &rot)
    :center(c),abc(r),R(rot){};
    ELLIPSOID(){};
};
typedef ELLIPSOID ellipsoid;

ellipsoid *el;

ros::Publisher ellipsoid_pub;


void feasibleVelocityCallback(const nav_msgs::OdometryConstPtr &msg)
{
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
    Eigen::Quaterniond orient;
    pos(0) = msg->pose.pose.position.x;
    pos(1) = msg->pose.pose.position.y;
    pos(2) = msg->pose.pose.position.z;

    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;
    vel = vel.array().abs();
    
    orient.w() = msg->pose.pose.orientation.w;
    orient.x() = msg->pose.pose.orientation.x;
    orient.y() = msg->pose.pose.orientation.y;
    orient.z() = msg->pose.pose.orientation.z;
    Eigen::Matrix3d rot = orient.toRotationMatrix();
    
    el = new ellipsoid(pos,vel,rot);
}



void drawEllipsoidCallback(const ros::TimerEvent &e)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 1;
    
    //set pose
    marker.pose.position.x = el->center.x();
    marker.pose.position.y = el->center.y();
    marker.pose.position.z = el->center.z();
    Eigen::Quaterniond r(el->R);
    marker.pose.orientation.w = r.w();
    marker.pose.orientation.x = r.x();
    marker.pose.orientation.y = r.y();
    marker.pose.orientation.z = r.z();

    //set scale
    marker.scale.x = el->abc.x();
    marker.scale.y = el->abc.y();
    marker.scale.z = el->abc.z();
    ellipsoid_pub.publish(marker);

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"draw_ellipsoid");
    ros::NodeHandle node;



    ros::Subscriber odom_sub = node.subscribe("visual_slam/odom", 10, feasibleVelocityCallback);

    ellipsoid_pub = node.advertise<visualization_msgs::Marker>("/planning/ellipsoid",50);

    ros::Timer ellipsoid_timer = node.createTimer(ros::Duration(0.01),drawEllipsoidCallback);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}

