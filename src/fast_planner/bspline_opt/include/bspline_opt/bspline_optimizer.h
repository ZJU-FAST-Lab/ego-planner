#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline/non_uniform_bspline.h>
#include <plan_env/sdf_map.h>
#include <ros/ros.h>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner {
class BsplineOptimizer {

public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */
  void            setEnvironment(const SDFMap::Ptr& env);
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

private:
  SDFMap::Ptr sdf_map_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  bool   dynamic_;                     // moving obstacles ?
  double start_time_;                  // global time for moving obstacles

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
  double lambda9_;                // curve fitting
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient, bool falg_use_jerk = true);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient);
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time      time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:

  AStar::Ptr a_star_;

  bool flag_continue_to_optimize_{false};
  //bool flag_record_intermediate_state_{false};

  struct ControlPoint
  {
    Eigen::Vector3d point;
    std::vector<Eigen::Vector3d> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<Eigen::Vector3d> direction; // Direction vector, must be normalized.
    double clearance;
    bool flag_temp; // A flag that used in many places. Initialize it everytime before using it.
    bool occupancy;
  };

  std::vector<ControlPoint> cps_;
  std::vector<Eigen::Vector3d> ref_pts_;

  std::vector<std::vector<Eigen::Vector3d>> initControlPoints(std::vector<Eigen::Vector3d> &init_points, bool flag_first_init = true);
  static double costFunctionRebound(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  static double costFunctionRefine(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void calcDistanceCostRebound(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient, int iter_num, double smoothness_cost);
  void calcFitnessCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  bool check_collision_and_rebound(void);
  bool BsplineOptimizeTrajRebound(const Eigen::MatrixXd init_points, Eigen::MatrixXd& optimal_points, double ts, double time_limit);
  bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd& init_points, const double ts, const double time_limit, Eigen::MatrixXd& optimal_points);
  bool rebound_optimize(double time_limit, double time_start);
  bool refine_optimize(double time_limit, double time_start);
  void combineCostRebound(const std::vector<double>& x, std::vector<double>& grad, double& f_combine);
  void combineCostRefine(const std::vector<double>& x, std::vector<double>& grad, double& f_combine);
  inline int getOrder(void) { return order_; }
};
}  // namespace fast_planner
#endif