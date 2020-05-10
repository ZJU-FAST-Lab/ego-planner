#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);
  nh.param("optimization/lambda9", lambda9_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  // ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = dim_ * (pt_num - 2 * order_);
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();


    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient,bool falg_use_jerk/* = true*/) {
  
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  if ( falg_use_jerk )
  {
    Eigen::Vector3d jerk, temp_j;

    // for (int i = 0; i < q.size(); i++)
    //   cout << "i=" << i << "@" << q[i].transpose() << endl;

    for (int i = 0; i < q.size() - 3; i++) {
      /* evaluate jerk */
      jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
      cost += jerk.squaredNorm();
      temp_j = 2.0 * jerk;
      /* jerk gradient */
      gradient[i + 0] += -temp_j;
      gradient[i + 1] += 3.0 * temp_j;
      gradient[i + 2] += -3.0 * temp_j;
      gradient[i + 3] += temp_j;

      // cout << "i=" << i << " jerk^2=" << jerk.squaredNorm()*1000 << endl;
    }

    // cout << endl;
  }
  else
  {    
    Eigen::Vector3d acc, temp_acc;

    for (int i = 0; i < q.size() - 2; i++) {
      /* evaluate jerk */
      acc = q[i + 2] - 2 * q[i + 1] + q[i];
      cost += acc.squaredNorm();
      temp_acc = 2.0 * acc;
      /* jerk gradient */
      gradient[i + 0] += temp_acc;
      gradient[i + 1] += -2.0 * temp_acc;
      gradient[i + 2] += temp_acc;
    }
  }
  
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

// void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
//                                            vector<Eigen::Vector3d>& gradient) {
//   cost = 0.0;
//   Eigen::Vector3d zero(0, 0, 0);
//   std::fill(gradient.begin(), gradient.end(), zero);

//   /* abbreviation */
//   double ts, vm2, am2, ts_inv2, ts_inv4;
//   vm2 = max_vel_ * max_vel_;
//   am2 = max_acc_ * max_acc_;

//   ts      = bspline_interval_;
//   ts_inv2 = 1 / ts / ts;
//   ts_inv4 = ts_inv2 * ts_inv2;

//   /* velocity feasibility */
//   for (int i = 0; i < q.size() - 1; i++) {
//     Eigen::Vector3d vi = q[i + 1] - q[i];

//     //cout << "temp_v * vi=" ;
//     for (int j = 0; j < 3; j++) {
//       double vd = vi(j) * vi(j) * ts_inv2 - vm2;
//       if (vd > 0.0) {
//         cost += vd;

//         double temp_v = 2.0 * vi(j) * ts_inv2;
//         gradient[i + 0](j) += -temp_v;
//         gradient[i + 1](j) += temp_v;
//       }
//       //cout << 4.0 * vd * ts_inv2 * vi(j) << " ";
//     }
//     //cout << endl;
//   }

//   /* acceleration feasibility */
//   for (int i = 0; i < q.size() - 2; i++) {
//     Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

//     //cout << "temp_a * ai=" ;
//     for (int j = 0; j < 3; j++) {
//       double ad = ai(j) * ai(j) * ts_inv4 - am2;
//       if (ad > 0.0) {
//         cost += ad;

//         double temp_a = 2.0 * ai(j) * ts_inv4;
//         gradient[i + 0](j) += temp_a;
//         gradient[i + 1](j) += -2 * temp_a;
//         gradient[i + 2](j) += temp_a;
//       }
//       //cout << 4.0 * ad * ts_inv4 * ai(j) << " ";
//     }
//     //cout << endl;
//   }
// }

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = (q[i + 1] - q[i])/ts;

    //cout << "temp_v * vi=" ;
    for (int j = 0; j < 3; j++) {
      if ( vi(j) > max_vel_ )
      {
        // cout << "fuck VEL" << endl;
        // cout << vi(j) << endl;
        cost += pow( vi(j)-max_vel_, 2 );

        gradient[i+0](j) += -2*(vi(j)-max_vel_)/ts;
        gradient[i+1](j) += 2*(vi(j)-max_vel_)/ts;
      }
      else if ( vi(j) < -max_vel_ )
      {
        cost += pow( vi(j)+max_vel_, 2 );

        gradient[i+0](j) += -2*(vi(j)+max_vel_)/ts;
        gradient[i+1](j) += 2*(vi(j)+max_vel_)/ts;
      }
      else
      {
        /* code */
      }
      //cout << 4.0 * vd * ts_inv2 * vi(j) << " ";
    }
    //cout << endl;
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i])*ts_inv2;

    //cout << "temp_a * ai=" ;
    for (int j = 0; j < 3; j++) 
    {
      if ( ai(j) > max_acc_ )
      {
        // cout << "fuck ACC" << endl;
        // cout << ai(j) << endl;
        cost += pow( ai(j)-max_acc_, 2 );

        gradient[i + 0](j) += 2*(ai(j)-max_acc_)*ts_inv2;
        gradient[i + 1](j) += -4*(ai(j)-max_acc_)*ts_inv2;
        gradient[i + 2](j) += 2*(ai(j)-max_acc_)*ts_inv2;
      }
      else if ( ai(j) < -max_acc_ )
      {
        cost += pow( ai(j)+max_acc_, 2 );

        gradient[i + 0](j) += 2*(ai(j)+max_acc_)*ts_inv2;
        gradient[i + 1](j) += -4*(ai(j)+max_acc_)*ts_inv2;
        gradient[i + 2](j) += 2*(ai(j)+max_acc_)*ts_inv2;
      }
      else
      {
        /* code */
      }
    }
    //cout << endl;
  }

}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(std::vector<Eigen::Vector3d> &init_points, bool flag_first_init /*= true*/)
{
  if ( flag_first_init )
  {
    cps_.clear();
    cps_.resize(init_points.size());

    for ( size_t i=0; i<init_points.size(); ++i )
    {
      cps_[i].point = init_points[i];
      cps_[i].clearance = dist0_;
    }
  }

  /*** Segment the initial trajectory according to obstacles ***/
  constexpr int ENOUGH_INTERVAL = 2;
  double step_size = edt_environment_->sdf_map_->getResolution() / ( (init_points[0] - init_points.back()).norm() / (init_points.size()-1) ) / 2;
  //cout << "step_size = " << step_size << endl;
  int in_id, out_id;
  vector<std::pair<int,int>> segment_ids;
  int same_occ_times = ENOUGH_INTERVAL + 1;
  bool occ, last_occ = false;
  bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
  for ( int i=order_; i<=(int)init_points.size()-order_; ++i )
  {
    for ( double a=1.0; a>=0.0; a-=step_size )
    {
      occ = edt_environment_->sdf_map_->getInflateOccupancy(a * init_points[i-1] + (1-a) * init_points[i]);

      // cout << "occ = " << occ << "  p = " << (a * init_points[i-1] + (1-a) * init_points[i]).transpose() << endl;
      // cout << "i=" << i <<endl;

      if ( occ && ! last_occ)
      {
        if ( same_occ_times > ENOUGH_INTERVAL || i == order_ )
        {
          in_id = i-1;
          flag_got_start = true;
        }
        same_occ_times = 0;
        flag_got_end_maybe = false;  // terminate in advance
      }
      else if( !occ && last_occ )
      {
        out_id = i;
        flag_got_end_maybe = true;
        same_occ_times = 0;
      }
      else
      {
        ++ same_occ_times; 
      }

      if ( flag_got_end_maybe && ( same_occ_times > ENOUGH_INTERVAL || ( i == (int)init_points.size()-order_ ) ) )
      {
        flag_got_end_maybe = false;
        flag_got_end = true;
      }
      
      last_occ = occ;
      
      if ( flag_got_start && flag_got_end )
      {
        flag_got_start = false;
        flag_got_end = false;
        segment_ids.push_back( std::pair<int,int>(in_id, out_id) );
      }
    }
  }

  // if ( segment_ids.empty() )
  // {
  //   ROS_ERROR("EMPTY?????????????????????????");

  //   Eigen::MatrixXd temp_cps( cps_.size(), 3 );

  //   for ( int i=0; i<cps_.size(); ++i )
  //     temp_cps.row(i) = cps_[i].point.transpose();

  //   auto traj = NonUniformBspline( temp_cps, 3, bspline_interval_ );
  //   double duration = traj.getTimeSum();
  //   for ( double t=0; t<duration; t+=0.02 )
  //   {
  //     bool occ = edt_environment_->sdf_map_->getInflateOccupancy( traj.evaluateDeBoorT(t) );
  //     cout << "occ = " << occ << "  p = " << traj.evaluateDeBoorT(t).transpose() << endl;
  //   }
  //   cout << endl;
  // }


  /*** a star search ***/
  vector<vector<Eigen::Vector3d>> a_star_pathes;
  for ( size_t i=0; i<segment_ids.size(); ++i )
  {
    //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
    Eigen::Vector3d in( init_points[segment_ids[i].first] ), out( init_points[segment_ids[i].second] );
    if ( a_star_->AstarSearch( /*(in-out).norm()/10+0.05*/0.1 , in, out) )
    {
      a_star_pathes.push_back( a_star_->getPath() );
    }
    else
    {
      ROS_ERROR("a star error, force return!");
      return a_star_pathes;
    }
  }

  // for ( int j=0; j<segment_ids.size(); ++j )
  // {
  //   cout << "------------ " << segment_ids[j].first << " " << segment_ids[j].second <<  endl;
  //   cout.precision(3);
  //   cout << "in=" << cps_[segment_ids[j].first].point.transpose() << " out=" << cps_[segment_ids[j].second].point.transpose() << endl;
  //   for ( int k=0; k<a_star_pathes[j].size(); ++k )
  //   {
  //     cout << "a_star_pathes[j][k]=" << a_star_pathes[j][k].transpose() << endl;
  //   }
  // }    

  /*** calculate bounds ***/
  int id_low_bound, id_up_bound;
  vector<std::pair<int,int>> bounds( segment_ids.size() );
  for (size_t i=0; i<segment_ids.size(); i++)
  {

    if ( i == 0 ) // first segment
    {
      id_low_bound = order_;
      if ( segment_ids.size() > 1 )
      {
        id_up_bound =  (int)(((segment_ids[0].second + segment_ids[1].first)-1.0f) / 2); // id_up_bound : -1.0f fix()
      }
      else
      {
        id_up_bound = init_points.size() - order_ - 1;
      }
    }
    else if ( i == segment_ids.size() - 1 ) // last segment, i != 0 here
    {
      id_low_bound = (int)(((segment_ids[i].first + segment_ids[i-1].second)+1.0f) / 2);   // id_low_bound : +1.0f ceil()
      id_up_bound = init_points.size() - order_ - 1;
    }
    else
    {
      id_low_bound = (int)(((segment_ids[i].first + segment_ids[i-1].second)+1.0f) / 2);  // id_low_bound : +1.0f ceil()
      id_up_bound = (int)(((segment_ids[i].second + segment_ids[i+1].first)-1.0f) / 2);  // id_up_bound : -1.0f fix()
    }
    
    bounds[i] = std::pair<int,int>(id_low_bound, id_up_bound);
  }

  // cout << "+++++++++" << endl;
  // for ( int j=0; j<bounds.size(); ++j )
  // {
  //   cout << bounds[j].first << "  " << bounds[j].second << endl;
  // }

  /*** Adjust segment length ***/
  vector<std::pair<int,int>> final_segment_ids( segment_ids.size() );
  constexpr double MINIMUM_PERCENT = 0.0;  // Each segment is guaranteed to have sufficient points to generate sufficient thrust
  int minimum_points = round(init_points.size() * MINIMUM_PERCENT), num_points;
  for (size_t i=0; i<segment_ids.size(); i++)
  {
    /*** Adjust segment length ***/
    num_points = segment_ids[i].second - segment_ids[i].first + 1;
    //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
    if ( num_points < minimum_points )
    {
      double add_points_each_side = (int)(((minimum_points - num_points)+1.0f) / 2);

      final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ?
        segment_ids[i].first - add_points_each_side :
        bounds[i].first;

      final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ?
        segment_ids[i].second + add_points_each_side :
        bounds[i].second;
    }
    else
    {
      final_segment_ids[i].first = segment_ids[i].first;
      final_segment_ids[i].second = segment_ids[i].second;
    }
    
    //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
  }

  /*** Assign parameters to each segment ***/
  for (size_t i=0; i<segment_ids.size(); i++)
  {
    // step 1
    for ( int j=final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j )
      cps_[j].flag_temp = false;

    // step 2
    int got_intersection_id = -1;
    for ( int j=segment_ids[i].first+1; j<segment_ids[i].second; ++j )
    {
      Eigen::Vector3d ctrl_pts_law(cps_[j+1].point - cps_[j-1].point), intersection_point;
      int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
      double val = (a_star_pathes[i][Astar_id] - cps_[j].point).dot( ctrl_pts_law ), last_val = val;
      while ( Astar_id >=0 && Astar_id < (int)a_star_pathes[i].size() )
      {
        last_Astar_id = Astar_id;

        if ( val >= 0 )
          -- Astar_id;
        else
          ++ Astar_id;
        
        val = (a_star_pathes[i][Astar_id] - cps_[j].point).dot( ctrl_pts_law );
        
        if ( val * last_val <= 0 && ( abs(val) > 0 || abs(last_val) > 0 ) ) // val = last_val = 0.0 is not allowed
        {
          intersection_point = 
            a_star_pathes[i][Astar_id] + 
            ( ( a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id] ) * 
              ( ctrl_pts_law.dot( cps_[j].point - a_star_pathes[i][Astar_id] ) / ctrl_pts_law.dot( a_star_pathes[i][Astar_id] -  a_star_pathes[i][last_Astar_id] ) ) // = t
            );

          //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

          got_intersection_id = j;
          break;
        }
      }

      if ( got_intersection_id >= 0 )
      {
        cps_[j].flag_temp = true;
        double length = (intersection_point - cps_[j].point).norm();
        if ( length > 1e-5 )
        {
          for ( double a=length; a>=0.0; a-=edt_environment_->sdf_map_->getResolution() )
          {
            occ =  edt_environment_->sdf_map_->getInflateOccupancy((a/length)*intersection_point + (1-a/length)*cps_[j].point);
      
            if ( occ || a < edt_environment_->sdf_map_->getResolution() )
            {
              if ( occ )
                a+=edt_environment_->sdf_map_->getResolution();
              cps_[j].base_point.push_back( (a/length)*intersection_point + (1-a/length)*cps_[j].point );
              cps_[j].direction.push_back( (intersection_point - cps_[j].point).normalized() );
              break;
            }
          }
        }
      }
    }

    /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
    if ( segment_ids[i].second - segment_ids[i].first == 1 ) 
    {
      Eigen::Vector3d ctrl_pts_law(cps_[segment_ids[i].second].point - cps_[segment_ids[i].first].point), intersection_point;
      Eigen::Vector3d middle_point = (cps_[segment_ids[i].second].point + cps_[segment_ids[i].first].point) / 2;
      int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
      double val = (a_star_pathes[i][Astar_id] - middle_point).dot( ctrl_pts_law ), last_val = val;
      while ( Astar_id >=0 && Astar_id < (int)a_star_pathes[i].size() )
      {
        last_Astar_id = Astar_id;

        if ( val >= 0 )
          -- Astar_id;
        else
          ++ Astar_id;
        
        val = (a_star_pathes[i][Astar_id] - middle_point).dot( ctrl_pts_law );
        
        if ( val * last_val <= 0 && ( abs(val) > 0 || abs(last_val) > 0 ) ) // val = last_val = 0.0 is not allowed
        {
          intersection_point = 
            a_star_pathes[i][Astar_id] + 
            ( ( a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id] ) * 
              ( ctrl_pts_law.dot( middle_point - a_star_pathes[i][Astar_id] ) / ctrl_pts_law.dot( a_star_pathes[i][Astar_id] -  a_star_pathes[i][last_Astar_id] ) ) // = t
            );

          cps_[segment_ids[i].first].flag_temp = true;
          cps_[segment_ids[i].first].base_point.push_back( cps_[segment_ids[i].first].point );
          cps_[segment_ids[i].first].direction.push_back( (intersection_point - middle_point).normalized() );

          got_intersection_id = segment_ids[i].first;
          break;
        }
      }
    }

    //step 3
    if ( got_intersection_id >= 0 )
    {
      for ( int j=got_intersection_id + 1; j <= final_segment_ids[i].second; ++j )
        if ( ! cps_[j].flag_temp )
        {
          cps_[j].base_point.push_back( cps_[j-1].base_point.back() );
          cps_[j].direction.push_back( cps_[j-1].direction.back() );
        }

      for ( int j=got_intersection_id - 1; j >= final_segment_ids[i].first; --j )
        if ( ! cps_[j].flag_temp )
        {
          cps_[j].base_point.push_back( cps_[j+1].base_point.back() );
          cps_[j].direction.push_back( cps_[j+1].direction.back() );
        }
    }
    else
    {
      ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      
      // cout << "↓↓↓↓↓↓↓↓↓↓↓↓ " << final_segment_ids[i].first << " " << final_segment_ids[i].second << endl;
      // cout.precision(3);
      // cout << "in=" << cps_[final_segment_ids[i].first].point.transpose() << " out=" << cps_[final_segment_ids[i].second].point.transpose() << endl;
      // for ( size_t k=0; k<a_star_pathes[i].size(); ++k )
      // {
      //   cout << "a_star_pathes[i][k]=" << a_star_pathes[i][k].transpose() << endl;
      // }
      // cout << "↑↑↑↑↑↑↑↑↑↑↑↑" << endl;
    }

  }

  return a_star_pathes;
}


double BsplineOptimizer:: costFunctionRebound(const std::vector<double>& x, std::vector<double>& grad, void* func_data)
{
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);

  double cost;
  opt->combineCostRebound(x, grad, cost);

  /* save the min cost result */
  opt->min_cost_ = cost;
  opt->best_variable_ = x;

  // early termination
  if ( opt->flag_continue_to_optimize_ )
  {
    cost = std::numeric_limits<double>::max();
    for ( size_t i=0; i<grad.size(); i++)
    {
      grad[i] = std::numeric_limits<double>::max();
    }
  }

  // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
  // cout << "cost=" << cost <<endl;

  opt->iter_num_ += 1;
  return cost;
}

double BsplineOptimizer:: costFunctionRefine(const std::vector<double>& x, std::vector<double>& grad, void* func_data)
{
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);

  double cost;
  opt->combineCostRefine(x, grad, cost);

  /* save the min cost result */
  opt->min_cost_ = cost;
  opt->best_variable_ = x;

  opt->iter_num_ += 1;
  return cost;
}

void BsplineOptimizer::calcDistanceCostRebound(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient, int iter_num, double smoothness_cost)
{
  //time_satrt = ros::Time::now();

  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  //ROS_WARN("iter_num=%d", iter_num);

  double dist;
  Eigen::Vector3d dist_grad;
  int end_idx = q.size() - order_;

  if ( iter_num > 3 && smoothness_cost / ( cps_.size() - 2*order_) < 0.1 ) // 0.1 is an experimental value that indicates the trajectory is smooth enough, leftover shit!!!
  {
    flag_continue_to_optimize_ = check_collision_and_rebound();
    //cout << "flag_continue_to_optimize_ = " << flag_continue_to_optimize_ << endl;
  }


  // cout << "iter_num = " << iter_num << endl;
  // for ( int i=0; i<=cps_.size()-1; ++i )
  // {
  //   cout << "--------------" <<endl;
  //   cout.precision(3);
  //   cout << "i=" << i << " point = " << cps_[i].point.transpose() << endl;
  //   for ( int j=0; j<cps_[i].direction.size(); ++j )
  //   {
  //     cout.precision(3);
  //     cout << "dir = " << cps_[i].direction[j].transpose() << " colli = " << cps_[i].base_point[j].transpose() << endl;
  //   }
  // }
  // cout <<endl;

  /*** calculate distance cost and gradient ***/
  for ( auto i=order_; i<end_idx; ++i )
  {
    for ( size_t j=0; j<cps_[i].direction.size(); ++j )
    {
      dist = (cps_[i].point - cps_[i].base_point[j]).dot(cps_[i].direction[j]);
      dist_grad = cps_[i].direction[j];
      if (dist < cps_[i].clearance)
      {
        if ( dist < -cps_[i].clearance )
        {
          // linear if the distance is too far.
          // cost += pow(dist - cps_[i].clearance, 2) - pow(dist + cps_[i].clearance, 2);
          cost += -4 * cps_[i].clearance * dist;  // == pow(dist - cps_[i].clearance, 2) - pow(dist + cps_[i].clearance, 2)
          gradient[i] += -4 * cps_[i].clearance * dist_grad;
          //cout << "run to here! i=" << i << " dist=" << dist << endl;
        }
        else
        {
          cost += pow(dist - cps_[i].clearance, 2);
          gradient[i] += 2.0 * (dist - cps_[i].clearance) * dist_grad;
        }
        
        // if ( iter_num <= 2 )
        // {
        //   cout << "[new xxx] iter_num=" << iter_num << " i=" << i << " cps_[i].direction.size()=" << cps_[i].direction.size() << " cost=" << cost << " gradient[i]" << gradient[i].transpose() << endl; 
        // }
      }
    }
  }

  // time_end = ros::Time::now();
  // cout << "time=" << (time_end - time_satrt).toSec()*1000000 << endl;

}

void BsplineOptimizer::calcFitnessCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient)
{
  //time_satrt = ros::Time::now();

  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d(0, 0, 0));

  int end_idx = q.size() - order_;

  // // def: f = x^2
  // for ( auto i=order_-1; i<end_idx+1; ++i )
  // {
  //   Eigen::Vector3d temp = (q[i-1]+4*q[i]+q[i+1])/6.0 - ref_pts_[i-1];
  //   cost += temp.squaredNorm();

  //   gradient[i-1] +=   temp/3.0;
  //   gradient[i]   += 4*temp/3.0;
  //   gradient[i+1] +=   temp/3.0;
  // }

  // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
  double a2 = 25, b2 = 1;
  for ( auto i=order_-1; i<end_idx+1; ++i )
  {
    Eigen::Vector3d x = (q[i-1]+4*q[i]+q[i+1])/6.0 - ref_pts_[i-1];
    Eigen::Vector3d v = (ref_pts_[i]-ref_pts_[i-2]).normalized();

    double xdotv = x.dot(v);
    Eigen::Vector3d xcrossv = x.cross(v);

    double f = pow((xdotv),2)/a2 + pow(xcrossv.norm(),2)/b2;
    cost += f;

    Eigen::Matrix3d m;
    m << 0,-v(2),v(1), v(2),0,-v(0), -v(1),v(0),0;
    Eigen::Vector3d df_dx = 2*xdotv/a2*v + 2/b2*m*xcrossv;

    gradient[i-1] += df_dx/6;
    gradient[i] += 4*df_dx/6;
    gradient[i+1] += df_dx/6;
  }

}



bool BsplineOptimizer::check_collision_and_rebound(void)
{

  int end_idx = cps_.size() - order_;

  /*** Check and segment the initial trajectory according to obstacles ***/
  int in_id, out_id;
  vector<std::pair<int,int>> segment_ids;
  bool flag_new_obs_valid = false;
  for ( int i=order_-1; i<=end_idx; ++i )
  {

    bool occ = edt_environment_->sdf_map_->getInflateOccupancy(cps_[i].point);

    /*** check if the new collision will be valid ***/
    if ( occ )
    {
      for ( size_t k=0; k<cps_[i].direction.size(); ++k )
      {
        cout.precision(2);
        //cout << "Test_02" << " i=" << i << " k=" << k << " direction[k]=" << cps_[i].direction[k].transpose() << " base_point[k]=" << cps_[i].base_point[k].transpose() << " point=" << cps_[i].point.transpose() << " dot=" << ( cps_[i].point - cps_[i].base_point[k] ).dot(cps_[i].direction[k]) << endl;
        //if ( dir.dot(cps_[j].direction[k]) > 1e-5 ) // the angle of two directions is smaller than 90 degree. 
        if ( ( cps_[i].point - cps_[i].base_point[k] ).dot(cps_[i].direction[k]) < 1 * edt_environment_->sdf_map_->getResolution() ) // current point is outside any of the collision_points. 
        {
          occ = false;
          //cout << "Test_00" << " flag_new_obs=" << flag_new_obs << " j=" << j << " k=" << k << " dir=" << dir.transpose() << " cps_[j].direction[k]=" << cps_[j].direction[k].transpose() << " dot=" << ( cps_[j].point - cps_[j].base_point[k] ).dot(cps_[j].direction[k]) << endl;
          break;
        }
      }
      //cout << "flag_new_obs_valid = " << flag_new_obs_valid << " iter = " << iter_num << endl;
    }

    if ( occ )
    {
      flag_new_obs_valid = true;
      cout << "hit new obs, cp_id = " << i << " iter=" << iter_num_ << endl;

      int j;
      for ( j=i-1; j>=0; --j )
      {
        occ = edt_environment_->sdf_map_->getInflateOccupancy(cps_[j].point);
        if ( !occ )
        {
          in_id = j;
          break;
        }
      }
      if ( j < 0 ) // fail to get the obs free point
      {
        ROS_ERROR( "ERROR! the drone is in obstacle. This should not happen." );
        in_id = 0;
      }

      for ( j=i+1; j<cps_.size(); ++j )
      {
        occ = edt_environment_->sdf_map_->getInflateOccupancy(cps_[j].point);
        if ( !occ )
        {
          out_id = j;
          break;
        }
      }
      if ( j >= cps_.size() ) // fail to get the obs free point
      {
        ROS_WARN( "WARN! terminal point of the current trajectory is in obstacle, skip this planning." );
        return 0;
      }

      i = j+1;

      segment_ids.push_back( std::pair<int,int>(in_id, out_id) );
    }
  }


  if ( flag_new_obs_valid )
  {
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for ( size_t i=0; i<segment_ids.size(); ++i )
    {
      /*** a star search ***/
      Eigen::Vector3d in( cps_[segment_ids[i].first].point ), out( cps_[segment_ids[i].second].point );
      if ( a_star_->AstarSearch( /*(in-out).norm()/10+0.05*/0.1, in, out) )
      {
        a_star_pathes.push_back( a_star_->getPath() );
      }
      else
      {
        ROS_ERROR("a star error");
        segment_ids.erase( segment_ids.begin() + i );
        i--;
      }

    }

    // if (flag_record_intermediate_state_)
    // {
    //   for ( auto pts : a_star_pathes )
    //   {
    //     a_star_pathes_log_.push_back(pts);
    //   }
    // }


    // for ( int j=0; j<segment_ids.size(); ++j )
    // {
    //   cout << "------------" << endl << segment_ids[j].first << " " << segment_ids[j].second << endl;
    //   for ( int k=0; k<a_star_pathes[j].size(); ++k )
    //   {
    //     cout << "a_star_pathes[j][k]=" << a_star_pathes[j][k].transpose() << endl;
    //   }
    // }

    /*** Assign parameters to each segment ***/
    for (size_t i=0; i<segment_ids.size(); ++i)
    {
      // step 1
      for ( int j=segment_ids[i].first; j <= segment_ids[i].second; ++j )
        cps_[j].flag_temp = false;

      // for ( auto x : segment_ids )
      // {
      //   cout << "first=" << x.first << " second=" << x.second << endl;
      // }
      // step 2
      int got_intersection_id = -1;
      for ( int j=segment_ids[i].first+1; j<segment_ids[i].second; ++j )
      {
        Eigen::Vector3d ctrl_pts_law(cps_[j+1].point - cps_[j-1].point), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - cps_[j].point).dot( ctrl_pts_law ), last_val = val;
        while ( Astar_id >=0 && Astar_id < (int)a_star_pathes[i].size() )
        {
          last_Astar_id = Astar_id;

          if ( val >= 0 )
            -- Astar_id;
          else
            ++ Astar_id;
          
          val = (a_star_pathes[i][Astar_id] - cps_[j].point).dot( ctrl_pts_law );
          
          if ( val * last_val <= 0 && ( abs(val) > 0 || abs(last_val) > 0 ) ) // val = last_val = 0.0 is not allowed
          {
            intersection_point = 
              a_star_pathes[i][Astar_id] + 
              ( ( a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id] ) * 
                ( ctrl_pts_law.dot( cps_[j].point - a_star_pathes[i][Astar_id] ) / ctrl_pts_law.dot( a_star_pathes[i][Astar_id] -  a_star_pathes[i][last_Astar_id] ) ) // = t
              );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if ( got_intersection_id >= 0 )
        {
          cps_[j].flag_temp = true;
          double length = (intersection_point - cps_[j].point).norm();
          if ( length > 1e-5 )
          {
            for ( double a=length; a>=0.0; a-=edt_environment_->sdf_map_->getResolution() )
            {
              bool occ = edt_environment_->sdf_map_->getInflateOccupancy((a/length)*intersection_point + (1-a/length)*cps_[j].point);
        
              if ( occ || a < edt_environment_->sdf_map_->getResolution() )
              {
                if ( occ )
                  a+=edt_environment_->sdf_map_->getResolution();
                cps_[j].base_point.push_back( (a/length)*intersection_point + (1-a/length)*cps_[j].point );
                cps_[j].direction.push_back( (intersection_point - cps_[j].point).normalized() );
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      //step 3
      if ( got_intersection_id >= 0 )
      {
        for ( int j=got_intersection_id + 1; j <= segment_ids[i].second; ++j )
          if ( ! cps_[j].flag_temp )
          {
            cps_[j].base_point.push_back( cps_[j-1].base_point.back() );
            cps_[j].direction.push_back( cps_[j-1].direction.back() );
          }

        for ( int j=got_intersection_id - 1; j >= segment_ids[i].first; --j )
          if ( ! cps_[j].flag_temp )
          {
            cps_[j].base_point.push_back( cps_[j+1].base_point.back() );
            cps_[j].direction.push_back( cps_[j+1].direction.back() );
          }
      }
      else
        ROS_WARN("Failed to generate direction!");
    }

    return true;
  }

  return false;
}

bool BsplineOptimizer::BsplineOptimizeTrajRebound(const Eigen::MatrixXd init_points, Eigen::MatrixXd& optimal_points, double ts, double time_limit)
{
  setControlPoints(init_points);
  setBsplineInterval(ts);
  //setTerminateCond(max_num_id, max_time_id);

  bool flag_success =  rebound_optimize(time_limit, 99999999999);

  optimal_points.resize(cps_.size(),3);
  for ( size_t i=0; i<cps_.size(); i++ )
  {
    optimal_points.row(i) = cps_[i].point;
  }

  return flag_success;
}


bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd& init_points, const double ts, const double time_limit, Eigen::MatrixXd& optimal_points)
{
  setControlPoints(init_points);
  setBsplineInterval(ts);
  //setTerminateCond(max_num_id, max_time_id);

  bool flag_success =  refine_optimize(time_limit, 99999999999);

  optimal_points.resize(control_points_.rows(),3);
  for ( size_t i=0; i<control_points_.rows(); i++ )
  {
    optimal_points.row(i) = control_points_.row(i);
  }

  return flag_success;
}

bool BsplineOptimizer::rebound_optimize(double time_limit, double time_start)
{
  /* ---------- initialize solver ---------- */
  iter_num_ = 0;
  int start_id = order_;
  int end_id = this->cps_.size() - order_;
  variable_num_ = 3 * (end_id - start_id);

  // cout << "variable_num_" << variable_num_ << endl;

  nlopt::opt opt(nlopt::algorithm(11 /*LBFGS*/), variable_num_);


  opt.set_min_objective(BsplineOptimizer::costFunctionRebound, this);

  opt.set_maxeval(50);
  opt.set_xtol_rel(1e-5);
  opt.set_maxtime(time_limit);

  /* ---------- init variables ---------- */
  vector<double> q(variable_num_);
  double final_cost;
  // for (size_t i = start_id; i < end_id; ++i)
  // {
  //   for (int j = 0; j < 3; j++)
  //     q[3 * (i - start_id) + j] = cps_[i].point(j);
  // }

  clock_t t0 = clock(), t1, t2;
  int restart_nums = 0, rebound_times = 0;;
  bool flag_occ = false;
  bool success = false;
  bool flag_nlopt_error_and_totally_fail = false;
  double original_lambda2 = lambda2_;
  constexpr int max_restart_nums_set = 3;
  do
  {
    min_cost_ = std::numeric_limits<double>::max();
    flag_continue_to_optimize_ = false;
    iter_num_ = 0;
    try
    {
      /* ---------- optimization ---------- */
      // cout << "[Optimization]: begin-------------" << endl;
      cout << fixed << setprecision(7);
      vec_time_.clear();
      vec_cost_.clear();
      t1 = clock();

      for (size_t i = start_id; i < end_id; ++i)
      {
        for (int j = 0; j < 3; j++)
          q[3 * (i - start_id) + j] = cps_[i].point(j);
      }

      /*nlopt::result result = */opt.optimize(q, final_cost);

      t2 = clock();
      /* ---------- get results ---------- */
      double time_ms = (double)(t2-t1)/CLOCKS_PER_SEC*1000;
      double total_time_ms = (double)(t2-t0)/CLOCKS_PER_SEC*1000;

      for (size_t i = start_id; i < end_id; ++i)
      {
        for (int j = 0; j < 3; j++)
          cps_[i].point(j) = best_variable_[3 * (i - start_id) + j];
      }

      // check collision
      for ( int i=0; i<cps_.size(); ++i )
        control_points_.row(i) = cps_[i].point.transpose();
      NonUniformBspline traj =  NonUniformBspline(control_points_, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      constexpr double t_step = 0.02;
      for ( double t = tm; t<tmp; t+=t_step )
      {
        flag_occ = edt_environment_->sdf_map_->getInflateOccupancy( traj.evaluateDeBoor(t) );
        if ( flag_occ )
        {
          cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoor(t).transpose() << endl;

          if ( t <= bspline_interval_ ) // First 3 control points in obstacles!
          {
            cout << cps_[1].point.transpose() << "\n"  << cps_[2].point.transpose() << "\n"  << cps_[3].point.transpose() << "\n" << cps_[4].point.transpose() << endl;
            ROS_ERROR("First 3 control points in obstacles! return false, t=%f",t);
            return false;
          }
          else if ( t > tmp-bspline_interval_ ) // First 3 control points in obstacles!
          {
            cout << "P=" << traj.evaluateDeBoor(t).transpose() << endl;
            ROS_ERROR("Last 3 control points in obstacles! return false, t=%f",t);
            return false;
          }

          goto hit_obs;
        }
      }
      
      success = true;

      hit_obs:; 

      // double step_size = edt_environment_->sdf_map_->getResolution() / ( (cps_[0].point - cps_.back().point ).norm() / cps_.size() );
      // for ( size_t i=1; i<cps_.size()-order_-1; ++i )
      // {
      //   for ( double a=1.0; a>=0.0; a-=step_size )
      //   {
      //     flag_occ = edt_environment_->sdf_map_->getInflateOccupancy(a * cps_[i].point + (1-a) * cps_[i+1].point) == 0 ? false : true;
      //     if ( flag_occ )
      //     {
      //       if ( i < order_-1 ) // First 3 control points in obstacles!
      //       {
      //         cout << cps_[1].point.transpose() << "\n"  << cps_[2].point.transpose() << "\n"  << cps_[3].point.transpose() << "\n" << cps_[4].point.transpose() << endl;
      //         ROS_ERROR("First 3 control points in obstacles! return false, i=%d",i);
      //         return false;
      //       }
      //       cout << "occ_pt=" << (a * cps_[i-1].point + (1-a) * cps_[i].point).transpose() << endl;
      //       goto out_loop;
      //     }
      //   }
      // }
      // out_loop:;

      if ( !flag_occ )
      {
        printf("\033[32miter(+1)=%d,time=%5.3f,total_t=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
      }
      else
      {
        restart_nums++;
        vector<Eigen::Vector3d> control_points(cps_.size());
        for ( size_t i=0; i<cps_.size(); i++ )
        {
          control_points[i] = cps_[i].point;
        }
        initControlPoints(control_points, false);
        //initControlPointsForESDF(control_points, false); lambda2_ *= 2; //ESDF TEST
                             

        printf("\033[32miter(+1)=%d,time=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
      }
      

      // cout << "[Optimization]: end-------------" << endl;
    }
    catch (std::exception& e)
    {
      t2 = clock();
      double time_ms = (double)(t2-t1)/CLOCKS_PER_SEC*1000;
      // printf("\033[36mfailure.iter=%d,time=%5.3f\n\033[0m", iter_num_, time_ms);
      cout << e.what();
      if ( flag_continue_to_optimize_ )
      {
        rebound_times ++;
        cout << "(Doesn't matter) iter=" << iter_num_ << ",time=" << time_ms << endl;
      }
      else
      {
        cout << endl;
        flag_nlopt_error_and_totally_fail = true;
      }
      
    }
  } while ( (flag_continue_to_optimize_ || flag_occ ) && 
            rebound_times <= 100 && 
            restart_nums < max_restart_nums_set && 
            !flag_nlopt_error_and_totally_fail && 
            !success
          );

  lambda2_ = original_lambda2;

  // if ( restart_nums < max_restart_nums_set || rebound_times > 100 ) return true; // rebound_times > 100? why???
  return success;
}

bool BsplineOptimizer::refine_optimize(double time_limit, double time_start)
{
  /* ---------- initialize solver ---------- */
  iter_num_ = 0;
  int start_id = order_;
  int end_id = this->control_points_.rows() - order_;
  variable_num_ = 3 * (end_id - start_id);

  // cout << "variable_num_" << variable_num_ << endl;

  nlopt::opt opt(nlopt::algorithm(11 /*LBFGS*/), variable_num_);


  opt.set_min_objective(BsplineOptimizer::costFunctionRefine, this);

  opt.set_maxeval(100);
  opt.set_xtol_rel(1e-5);
  opt.set_maxtime(time_limit);

  /* ---------- init variables ---------- */
  vector<double> q(variable_num_);
  double final_cost;
  for (size_t i = start_id; i < end_id; ++i)
  {
    for (int j = 0; j < 3; j++)
      q[3 * (i - start_id) + j] = control_points_(i,j);
  }

  // // leftover shit!!!
  // NonUniformBspline traj =  NonUniformBspline(control_points_, 3, bspline_interval_);
  // double tm, tmp;
  // traj.getTimeSpan(tm, tmp);
  // constexpr double t_step = 0.02;
  // for ( double t = tm; t<tmp; t+=t_step )
  // {
  //   if ( edt_environment_->sdf_map_->getInflateOccupancy( traj.evaluateDeBoor(t) ) )
  //   {
  //     cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoor(t).transpose() << endl;

  //     break;
  //   }
  // }

  double origin_lambda9 = lambda9_;
  bool flag_safe = true;
  int iter_count = 0;
  do
  {
    try
    {
      nlopt::result result = opt.optimize(q, final_cost);
      //cout << "result=" << result << endl;

      /* ---------- get results ---------- */
      for (size_t i = start_id; i < end_id; ++i)
      {
        for (int j = 0; j < 3; j++)
          control_points_(i,j) = best_variable_[3 * (i - start_id) + j];
      }

      NonUniformBspline traj =  NonUniformBspline(control_points_, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);

      constexpr double t_step = 0.01;
      for ( double t = tm; t<tmp; t+=t_step )
      {
        if ( edt_environment_->sdf_map_->getInflateOccupancy( traj.evaluateDeBoor(t) ) )
        {
          cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoor(t).transpose() << endl;
          flag_safe = false;
          break;
        }
      }

      if ( !flag_safe ) lambda9_*=2;

      iter_count++;
    }
    catch (std::exception& e)
    {
      cout << e.what() << endl;
      return false;
    }
  } while ( !flag_safe && iter_count <= 3 );

  if ( iter_count > 1 && iter_count <=3 )
  {
    ROS_ERROR("Refine iter_count > 1 && iter_count <=3");
  }
  if ( iter_count > 3 )
  {
    cout << "iter_count=" << iter_count << endl;
    ROS_ERROR("Refine iter_count > 3");
  }
  
  lambda9_ = origin_lambda9;

  //cout << "iter_num_=" << iter_num_ << endl;

  return flag_safe;
}

void BsplineOptimizer::combineCostRebound(const std::vector<double>& x, std::vector<double>& grad, double& f_combine)
{
  /* ---------- convert to control point vector ---------- */
  vector<Eigen::Vector3d> q;
  q.reserve( cps_.size() );

  /* first p points */
  for (int i = 0; i < order_; i++)
    q.push_back(cps_[i].point);

  /* optimized control points */
  for (int i = 0; i < variable_num_ / 3; i++)
  {
    Eigen::Vector3d qi(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
    q.push_back(qi);
  }

  /* last p points */
  for (int i = 0; i < order_; i++)
    q.push_back(cps_[cps_.size() - order_ + i].point);

  for ( size_t i=order_; i<cps_.size()-order_; ++i )
  {
    cps_[i].point = q[i];
    cps_[i].occupancy = edt_environment_->sdf_map_->getInflateOccupancy(cps_[i].point);
  }

  // for ( int i=0; i<cps_.size(); i++ )
  //   cout << cps_[i].point.transpose() << endl;
  // cout << endl;

  /* ---------- evaluate cost and gradient ---------- */
  double f_smoothness, f_distance, f_feasibility;

  vector<Eigen::Vector3d> g_smoothness, g_distance, g_feasibility;
  g_smoothness.resize(cps_.size());
  g_distance.resize(cps_.size());
  g_feasibility.resize(cps_.size());

  //time_satrt = ros::Time::now();

  calcSmoothnessCost(q, f_smoothness, g_smoothness);
  calcDistanceCostRebound(q, f_distance, g_distance, iter_num_, f_smoothness);
  calcFeasibilityCost(q, f_feasibility, g_feasibility);

  // time_end = ros::Time::now();
  // cout << "cost compute time=" << (time_end - time_satrt).toSec()*1000000 << " whole iteration time=" << (ros::Time::now() - time_all).toSec()*1000000 << endl;
  // time_all = ros::Time::now();

  /* ---------- convert to NLopt format...---------- */
  f_combine = lambda1_ * f_smoothness + lambda2_ * f_distance + lambda3_ * f_feasibility;
  //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);


  // for (int i = 0; i < q.size(); i++) 
  // {
  //   cout << g_smoothness[i].transpose() << endl;
  //   cout << g_distance[i].transpose() << endl;
  //   cout << g_feasibility[i].transpose() << endl;
  //   cout << lambda1_*g_smoothness[i].transpose() + lambda2_*g_distance[i].transpose() + lambda3_*g_feasibility[i].transpose() << endl << endl;
  // }


  grad.resize(variable_num_);
  for (int i = 0; i < variable_num_ / 3; i++)
  {
    
    for (int j = 0; j < 3; j++)
    {
      /* the first p points is static here */
      grad[3 * i + j] = lambda1_ * g_smoothness[i + order_](j) + lambda2_ * g_distance[i + order_](j) +
                        lambda3_ * g_feasibility[i + order_](j);

    }
    //cout << "g_smoothness=" << g_smoothness[i + order_].transpose() << " g_distance=" << g_distance[i + order_].transpose() << " g_feasibility=" << g_feasibility[i + order_].transpose() << endl;
  }
}

void BsplineOptimizer::combineCostRefine(const std::vector<double>& x, std::vector<double>& grad, double& f_combine)
{
  /* ---------- convert to control point vector ---------- */
  vector<Eigen::Vector3d> q;
  q.reserve( control_points_.rows() );

  /* first p points */
  for (int i = 0; i < order_; i++)
    q.push_back(control_points_.row(i));

  /* optimized control points */
  for (int i = 0; i < variable_num_ / 3; i++)
  {
    Eigen::Vector3d qi(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
    q.push_back(qi);
  }

  /* last p points */
  for (int i = 0; i < order_; i++)
    q.push_back(control_points_.row(control_points_.rows()-order_+i));

  /* ---------- evaluate cost and gradient ---------- */
  double f_smoothness, f_fitness, f_feasibility;

  vector<Eigen::Vector3d> g_smoothness, g_fitness, g_feasibility;
  g_smoothness.resize(control_points_.rows());
  g_fitness.resize(control_points_.rows());
  g_feasibility.resize(control_points_.rows());

  //time_satrt = ros::Time::now();

  calcSmoothnessCost(q, f_smoothness, g_smoothness);
  calcFitnessCost(q, f_fitness, g_fitness);
  calcFeasibilityCost(q, f_feasibility, g_feasibility);

  // time_end = ros::Time::now();
  // cout << "cost compute time=" << (time_end - time_satrt).toSec()*1000000 << " whole iteration time=" << (ros::Time::now() - time_all).toSec()*1000000 << endl;
  // time_all = ros::Time::now();

  /* ---------- convert to NLopt format...---------- */
  f_combine = lambda1_ * f_smoothness + lambda9_ * f_fitness + lambda3_ * f_feasibility;
  // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);


  // for (int i = 0; i < q.size(); i++) 
  // {
  //   //cout << g_smoothness[i].transpose() << endl;
  //   cout << g_fitness[i].transpose() << endl;
  //   //cout << g_feasibility[i].transpose() << endl;
  //   //cout << lambda1_*g_smoothness[i].transpose() + lambda9_*g_fitness[i].transpose() + lambda3_*g_feasibility[i].transpose() << endl << endl;
  // }


  grad.resize(variable_num_);
  for (int i = 0; i < variable_num_ / 3; i++)
  {
    
    for (int j = 0; j < 3; j++)
    {
      /* the first p points is static here */
      grad[3 * i + j] = lambda1_ * g_smoothness[i + order_](j) + lambda9_ * g_fitness[i + order_](j) +
                        lambda3_ * g_feasibility[i + order_](j);

    }
    //cout << "g_smoothness=" << g_smoothness[i + order_].transpose() << " g_distance=" << g_distance[i + order_].transpose() << " g_feasibility=" << g_feasibility[i + order_].transpose() << endl;
  }
}


}  // namespace fast_planner