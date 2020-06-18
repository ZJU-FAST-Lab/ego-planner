#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace rebound_planner {

class ControlPoints
{
public:

  double clearance;
  int size;
  Eigen::MatrixXd points;
  std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
  std::vector<std::vector<Eigen::Vector3d>> direction; // Direction vector, must be normalized.
  std::vector<bool> flag_temp; // A flag that used in many places. Initialize it everytime before using it.
  std::vector<bool> occupancy;

  void resize( const int size_set )
  {
    size = size_set;

    base_point.clear();
    direction.clear();
    flag_temp.clear();
    occupancy.clear();

    points.resize(3, size_set);
    base_point.resize(size);
    direction.resize(size);
    flag_temp.resize(size);
    occupancy.resize(size);
  }
};

class BsplineOptimizer {

public:

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */
  void            setEnvironment(const GridMap::Ptr& env);
  void            setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd         getControlPoints();

  AStar::Ptr a_star_;
  std::vector<Eigen::Vector3d> ref_pts_;

  std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd& init_points, bool flag_first_init = true);
  bool BsplineOptimizeTrajRebound(Eigen::MatrixXd& optimal_points, double ts); // must be called after initControlPoints()
  bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd& init_points, const double ts, Eigen::MatrixXd& optimal_points);

  inline int getOrder(void) { return order_; }

private:
  GridMap::Ptr grid_map_;

  enum FORCE_STOP_OPTIMIZE_TYPE
  {
    DONT_STOP,
    STOP_FOR_REBOUND,
    STOP_FOR_ERROR
  } force_stop_type_;

  // main input
  // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  // int             dim_;                // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  double start_time_;                  // global time for moving obstacles

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // curve fitting

  int a;
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  Eigen::VectorXd     best_variable_;  //
  double              min_cost_;       //

  ControlPoints cps_;

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const Eigen::MatrixXd& q, double& cost,
                          Eigen::MatrixXd& gradient, bool falg_use_jerk = true);
  void calcFeasibilityCost(const Eigen::MatrixXd& q, double& cost,
                           Eigen::MatrixXd& gradient);
  void calcDistanceCostRebound(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient, int iter_num, double smoothness_cost);
  void calcFitnessCost(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient);
  bool check_collision_and_rebound(void);

  static double costFunctionRebound(const Eigen::VectorXd& x, Eigen::VectorXd& grad, bool& force_return, void* func_data);
  static double costFunctionRefine(const Eigen::VectorXd& x, Eigen::VectorXd& grad, bool& force_return, void* func_data);
  // static double costFunctionRebound_nlopt(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  // static double costFunctionRefine_nlopt(const std::vector<double>& x, std::vector<double>& grad, void* func_data);

  bool rebound_optimize();
  // bool rebound_optimize_nlopt();
  bool refine_optimize();
  // bool refine_optimize_nlopt();
  void combineCostRebound(const Eigen::VectorXd& x, Eigen::VectorXd& grad, double& f_combine);
  void combineCostRefine(const Eigen::VectorXd& x, Eigen::VectorXd& grad, double& f_combine);

  /* for benckmark evaluation only */
public:

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace rebound_planner
#endif