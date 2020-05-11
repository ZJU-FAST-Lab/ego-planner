// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh, PlanningVisualization::Ptr vis) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception, use_rebound;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_rebound", use_rebound, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(sdf_map_);
    }
  }

  if (use_rebound) {
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(sdf_map_);
    bspline_optimizer_rebound_->a_star_.reset( new AStar );
    bspline_optimizer_rebound_->a_star_->initGridMap(sdf_map_, Eigen::Vector3i(100,100,100));
  }

  visualization_ = vis;
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollisionInflate(NonUniformBspline &traj) {

  double tm, tmp;
  traj.getTimeSpan(tm, tmp);

  constexpr double t_step = 0.02;
  for ( double t = tm; t<tmp; t+=t_step )
  {
    if ( sdf_map_->getInflateOccupancy( traj.evaluateDeBoor(t) ) )
    {
      return false;
    }
  }

  return true;
}

// !SECTION

// SECTION rebond replanning

bool FastPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                           Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj) {

  static int count = 0;
  std::cout << endl << "[rebo replan]: -------------------------------------" << count++ << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
       << endl;

  if ((start_pt - local_target_pt).norm() < 0.2) {
    cout << "Close to goal" << endl;
    return false;
  }

  

  ros::Time t1, t2;

  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  // parameterize the path to bspline

  t1 = ros::Time::now();

  double ts = pp_.ctrl_pt_dist / pp_.max_vel_*1.0; // Leftover shit!!!
  vector<Eigen::Vector3d> point_set, start_end_derivatives; 
  static bool flag_first_call = true, flag_force_polynomial = false;
  bool flag_regenerate = false;
  do
  {
    point_set.clear();
    start_end_derivatives.clear();

    if ( flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)
    {
      flag_first_call = false;
      flag_force_polynomial = false;
      
      PolynomialTraj gl_traj;

      double dist = (start_pt - local_target_pt).norm();
      double time = pow(pp_.max_vel_,2) / pp_.max_acc_ > dist ?
        sqrt(dist / pp_.max_acc_) : 
        (dist - pow(pp_.max_vel_,2)/pp_.max_acc_)/pp_.max_vel_ + 2*pp_.max_vel_/pp_.max_acc_;
      
      if ( !flag_randomPolyTraj )
      {
        gl_traj = one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0,0,1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        Eigen::Vector3d random_inserted_pt =  (start_pt + local_target_pt)/2 + 
                                              (((double)rand())/RAND_MAX-0.5)*(start_pt-local_target_pt).norm()*horizen_dir*0.8 + 
                                              (((double)rand())/RAND_MAX-0.5)*(start_pt-local_target_pt).norm()*vertical_dir*0.4;
        Eigen::MatrixXd pos(3,3);
        pos.row(0) = start_pt.transpose();
        pos.row(1) = random_inserted_pt.transpose();
        pos.row(2) = local_target_pt.transpose();
        Eigen::VectorXd t(2);
        t(0) = t(1) = time / 2;
        gl_traj = minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
      }
      

      double t;
      bool flag_too_far;
      ts *= 1.5;  // ts will be divided by 1.5 in the next
      do
      {
        ts /= 1.5;
        point_set.clear();
        flag_too_far = false;
        Eigen::Vector3d last_pt = gl_traj.evaluate(0);
        for ( t=0; t<time; t+=ts )
        {
          Eigen::Vector3d pt = gl_traj.evaluate(t);
          if ( (last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5 )
          {
            flag_too_far = true;
            break;
          }
          last_pt = pt;
          point_set.push_back( pt );
        }
      } while ( flag_too_far || point_set.size() < 7 ); // If the start point is very close to end point, this will help
      t-=ts;
      start_end_derivatives.push_back( gl_traj.evaluateVel(0) );
      start_end_derivatives.push_back( local_target_vel );
      start_end_derivatives.push_back( gl_traj.evaluateAcc(0) );
      start_end_derivatives.push_back( gl_traj.evaluateAcc(t) );

    }
    else
    {

      double t;
      double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

      vector<double> pseudo_arc_length;
      vector<Eigen::Vector3d> segment_point;
      pseudo_arc_length.push_back(0.0);
      for ( t=t_cur; t<local_data_.duration_+1e-3; t+=ts )
      {
        segment_point.push_back( local_data_.position_traj_.evaluateDeBoorT( t ) );
        if ( t > t_cur )
        {
          pseudo_arc_length.push_back( (segment_point.back() - segment_point[ segment_point.size() - 2 ]).norm() + pseudo_arc_length.back() );
        }
      }
      t -= ts;

      double poly_time = (local_data_.position_traj_.evaluateDeBoorT( t ) - local_target_pt).norm() / pp_.max_vel_ * 2;
      if ( poly_time > ts )
      {
        PolynomialTraj gl_traj = one_segment_traj_gen( local_data_.position_traj_.evaluateDeBoorT( t ), 
                                              local_data_.velocity_traj_.evaluateDeBoorT( t ), 
                                              local_data_.acceleration_traj_.evaluateDeBoorT( t ), 
                                              local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time );

        for ( t = ts; t<poly_time; t+=ts )
        {
          if ( ! pseudo_arc_length.empty() )
          {
            segment_point.push_back( gl_traj.evaluate(t) );
            pseudo_arc_length.push_back( (segment_point.back() - segment_point[ segment_point.size() - 2 ]).norm() + pseudo_arc_length.back() );
          }
          else
          {
            ROS_ERROR("pseudo_arc_length is empty, return!");
            return false;
          }
        }
      }
      
      double sample_length = 0;
      double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
      int id = 0;
      do
      {
        cps_dist /= 1.5;
        point_set.clear();
        sample_length = 0;
        id = 0;
        while( (id <= pseudo_arc_length.size() -2) && sample_length <= pseudo_arc_length.back() )
        {
          if ( sample_length >= pseudo_arc_length[ id ] && sample_length < pseudo_arc_length[ id+1 ] )
          {
            point_set.push_back( (sample_length-pseudo_arc_length[id])/(pseudo_arc_length[id+1]-pseudo_arc_length[id])*segment_point[id+1] + 
                                (pseudo_arc_length[id+1]-sample_length)/(pseudo_arc_length[id+1]-pseudo_arc_length[id])*segment_point[id] );
            sample_length += cps_dist;
          }
          else
            id++;
        }
        point_set.push_back( local_target_pt );
      } while (point_set.size() < 7);  // If the start point is very close to end point, this will help
      


      start_end_derivatives.push_back( local_data_.velocity_traj_.evaluateDeBoorT(t_cur) );
      start_end_derivatives.push_back( local_target_vel );
      start_end_derivatives.push_back( local_data_.acceleration_traj_.evaluateDeBoorT(t_cur) );
      start_end_derivatives.push_back( Eigen::Vector3d::Zero() );

      if ( point_set.size() > 50 )
      {
        flag_force_polynomial = true;
        flag_regenerate = true;
      }
    }
  } while ( flag_regenerate );

  cout << "distance=" << (start_pt-local_target_pt).norm() << endl;
  cout << "point_set.size()=" << point_set.size() << endl;
  cout << "ts=" << ts << endl;
  cout << "all_time=" << ts*(point_set.size()-1) << endl;

  Eigen::MatrixXd ctrl_pts;
  //ts = 3.25/(point_set.size()-1);
  //ts *= 1.5;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  if ( ctrl_pts.rows() <= 2*bspline_optimizer_rebound_->getOrder() ) // Drone is very close to the goal point
    return true;
  
  vector<Eigen::Vector3d> raw_cps;
  vector<vector<Eigen::Vector3d>> a_star_pathes;
  for ( int i=0; i<ctrl_pts.rows(); ++i )
    raw_cps.push_back( ctrl_pts.row(i).transpose() );

  a_star_pathes = bspline_optimizer_rebound_->initControlPoints( raw_cps );

  t_search = (ros::Time::now() - t1).toSec();

  static int vis_id=0;
  visualization_->displayInitList(point_set,0);
  visualization_->displayAStarList(a_star_pathes,vis_id);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  
  bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ctrl_pts, ts, 0.1);
  cout << "first optimize step success=" << flag_step_1_success << endl;
  if ( !flag_step_1_success )
  {
    visualization_->displayOptimalList( ctrl_pts, vis_id );
    return false;
  } 

  t_opt = (ros::Time::now() - t1).toSec();

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);
  double time_inc;
  Eigen::MatrixXd optimal_control_points;

  bool flag_step_2_success;
  if ( !feasible )
  {
    plan_data_.local_start_end_derivative_ = start_end_derivatives;
    flag_step_2_success = refineTrajAlgo2(pos, time_inc, ts, optimal_control_points);
    if ( flag_step_2_success )
      pos = NonUniformBspline(optimal_control_points, 3, ts);

    cout << "infeasible" << endl;
  }
  else
  {
    cout << "feasible" << endl;

    double t_step = pos.getTimeSum() / (pos.getControlPoint().rows()-3);
    bspline_optimizers_[0]->ref_pts_.clear();
    for ( double t=0; t<pos.getTimeSum()+1e-4; t+=t_step )
      bspline_optimizers_[0]->ref_pts_.push_back( pos.evaluateDeBoorT(t) );
    flag_step_2_success = bspline_optimizers_[0]->BsplineOptimizeTrajRefine(ctrl_pts, ts, 0.1/*seconds*/, optimal_control_points);
    if ( flag_step_2_success )
      pos = NonUniformBspline(optimal_control_points, 3, ts) ;
  }

  if ( !flag_step_2_success )
  {

    ROS_WARN("Failed to refine trajectory, but I can not do more :(");
  }

  double tn = pos.getTimeSum();

  cout << "[rebo replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  local_data_.position_traj_ = pos;
  local_data_.start_time_ = ros::Time::now();
  updateTrajInfo();

  double t_total = t_search + t_opt + t_adjust;
  cout << "[rebo replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  visualization_->displayOptimalList( local_data_.position_traj_.get_control_points(), vis_id );
  //vis_id += 10;

  return flag_step_2_success;
}

bool FastPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
{
  Eigen::MatrixXd control_points(6,3);
  for ( int i=0; i<6; i++ )
  {
    control_points.row(i) = stop_pos.transpose();
  }
  NonUniformBspline bspline_container = NonUniformBspline(control_points, 3, 1.0);
  local_data_.position_traj_ = bspline_container;
  local_data_.start_time_ = ros::Time::now();
  updateTrajInfo();

  return true;
}

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
                                        const Eigen::Vector3d& end_pos, const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc) 
{

  // generate global reference trajectory

  vector<Eigen::Vector3d> points;
  points.push_back( start_pos );
  points.push_back( end_pos );

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());

  // write position matrix
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  PolynomialTraj gl_traj;
  if ( pos.rows() >= 3 )
    gl_traj =  minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
  else if ( pos.rows() == 2 )
    gl_traj = one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
  else
    return false;

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  return true;
}

bool FastPlannerManager::slidingWindow(Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel)
{
  /*** step 1: get point_set ***/
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;

  double t;
  double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

  vector<double> pseudo_arc_length;
  vector<Eigen::Vector3d> segment_point;
  pseudo_arc_length.push_back(0.0);
  for ( t=t_cur; t<local_data_.duration_; t+=ts )
  {
    segment_point.push_back( local_data_.position_traj_.evaluateDeBoorT( t ) );
    if ( t > t_cur )
    {
      pseudo_arc_length.push_back( (segment_point.back() - segment_point[ segment_point.size() - 2 ]).norm() + pseudo_arc_length.back() );
    }
  }
  t -= ts;

  double poly_time = (local_data_.position_traj_.evaluateDeBoorT( t ) - local_target_pt).norm() / pp_.max_vel_;
  PolynomialTraj gl_traj = one_segment_traj_gen( local_data_.position_traj_.evaluateDeBoorT( t ), 
                                        local_data_.velocity_traj_.evaluateDeBoorT( t ), 
                                        local_data_.acceleration_traj_.evaluateDeBoorT( t ), 
                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time );

  for ( t = ts; t<poly_time; t+=ts )
  {
    if ( ! pseudo_arc_length.empty() )
    {
      segment_point.push_back( gl_traj.evaluate(t) );
      pseudo_arc_length.push_back( (segment_point.back() - segment_point[ segment_point.size() - 2 ]).norm() + pseudo_arc_length.back() );
    }
    else
    {
      ROS_ERROR("pseudo_arc_length is empty, return!");
      return false;
    }
  }
  
  double sample_length = 0;
  int id = 0;
  while( (id <= pseudo_arc_length.size() -2) && sample_length <= pseudo_arc_length.back() )
  {
    if ( sample_length >= pseudo_arc_length[ id ] && sample_length < pseudo_arc_length[ id+1 ] )
    {
      point_set.push_back( (sample_length-pseudo_arc_length[id])/(pseudo_arc_length[id+1]-pseudo_arc_length[id])*segment_point[id+1] + 
                          (pseudo_arc_length[id+1]-sample_length)/(pseudo_arc_length[id+1]-pseudo_arc_length[id])*segment_point[id] );
      sample_length += pp_.ctrl_pt_dist;
    }
    else
      id++;
  }
  point_set.push_back(segment_point.back());

  start_end_derivatives.push_back( local_data_.velocity_traj_.evaluateDeBoorT(t_cur) );
  start_end_derivatives.push_back( local_target_vel );
  start_end_derivatives.push_back( local_data_.acceleration_traj_.evaluateDeBoorT(t_cur) );
  start_end_derivatives.push_back( Eigen::Vector3d::Zero() );

  /*** step 2: get b-spline ***/
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  
  /*** re-allocate time ***/
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  /***  ***/
  local_data_.position_traj_ = pos;
  local_data_.start_time_ = ros::Time::now();
  updateTrajInfo();

  return true;
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}

bool FastPlannerManager::refineTrajAlgo2(NonUniformBspline& traj, double& time_inc, double& ts, Eigen::MatrixXd& optimal_control_points) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    t_inc;
  const int max_iter = 1;

  Eigen::MatrixXd ctrl_pts      = traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(traj, ratio, ctrl_pts, ts, t_inc);
  time_inc += t_inc;

  traj = NonUniformBspline(ctrl_pts, 3, ts);
  
  double t_step = traj.getTimeSum() / (ctrl_pts.rows()-3);
  bspline_optimizers_[0]->ref_pts_.clear();
  for ( double t=0; t<traj.getTimeSum()+1e-4; t+=t_step )
    bspline_optimizers_[0]->ref_pts_.push_back( traj.evaluateDeBoorT(t) );

  bool success = bspline_optimizers_[0]->BsplineOptimizeTrajRefine(ctrl_pts, ts, 0.1/*seconds*/, optimal_control_points);

  return success;
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);

  if ( seg_num <= bspline_optimizers_[1]->getOrder() )
  {
    ROS_WARN( "plan_Yaw. seg_num is too small, return." );
    return;
  }

  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
