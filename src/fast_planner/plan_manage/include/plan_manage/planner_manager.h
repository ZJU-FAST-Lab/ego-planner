#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <plan_manage/DataDisp.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

#include <traj_utils/planning_visualization.h>

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

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
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  bool checkTrajCollision(double& distance);
  bool checkTrajCollisionInflate(NonUniformBspline &traj);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;

private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;
  PlanningVisualization::Ptr visualization_;

  unique_ptr<Astar> geo_path_finder_;
  unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;
  BsplineOptimizer::Ptr bspline_optimizer_rebound_;

  void updateTrajInfo();

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  void selectBestTraj(NonUniformBspline& traj);
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  // zxzx
  bool refineTrajAlgo2(NonUniformBspline& traj, double& time_inc, double& ts, Eigen::MatrixXd& optimal_control_points);

  // heading planning
  void calcNextYaw(const double& last_yaw, double& yaw);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif