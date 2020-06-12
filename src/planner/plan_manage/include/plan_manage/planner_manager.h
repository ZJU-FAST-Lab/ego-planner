#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>

#include <rebound_planner/DataDisp.h>

#include <plan_env/sdf_map.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

#include <traj_utils/planning_visualization.h>

namespace rebound_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class ReboundPlannerManager {
  // SECTION stable
public:
  ReboundPlannerManager();
  ~ReboundPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
  bool EmergencyStop(Eigen::Vector3d stop_pos);
  bool slidingWindow(Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
                                        const Eigen::Vector3d& end_pos, const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc) ;
  bool topoReplan(bool collide);

  void planYaw(const Eigen::Vector3d& start_yaw);

  void initPlanModules(ros::NodeHandle& nh, PlanningVisualization::Ptr vis = NULL);

  bool checkTrajCollision(double& distance);
  bool checkTrajCollisionInflate(UniformBspline &traj);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  SDFMap::Ptr sdf_map_;

private:
  /* main planning algorithms & modules */
  PlanningVisualization::Ptr visualization_;

  BsplineOptimizer::Ptr bspline_optimizer_rebound_;

  int continous_failures_count_{0};

  void updateTrajInfo();

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  void selectBestTraj(UniformBspline& traj);
  void refineTraj(UniformBspline& best_traj, double& time_inc);
  void reparamBspline(UniformBspline& bspline, vector<Eigen::Vector3d>& start_end_derivative, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  bool refineTrajAlgo(UniformBspline& traj, vector<Eigen::Vector3d>& start_end_derivative, double ratio, double& ts, Eigen::MatrixXd& optimal_control_points);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<ReboundPlannerManager> Ptr;

  // !SECTION
};
}  // namespace rebound_planner

#endif