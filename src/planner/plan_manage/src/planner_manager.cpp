// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace rebound_planner {

// SECTION interfaces for setup and query

ReboundPlannerManager::ReboundPlannerManager() {}

ReboundPlannerManager::~ReboundPlannerManager() { std::cout << "des manager" << std::endl; }

void ReboundPlannerManager::initPlanModules(ros::NodeHandle& nh, PlanningVisualization::Ptr vis) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  bspline_optimizer_rebound_.reset(new BsplineOptimizer);
  bspline_optimizer_rebound_->setParam(nh);
  bspline_optimizer_rebound_->setEnvironment(sdf_map_);
  bspline_optimizer_rebound_->a_star_.reset( new AStar );
  bspline_optimizer_rebound_->a_star_->initGridMap( sdf_map_, Eigen::Vector3i(100,100,100) );

  visualization_ = vis;
}

bool ReboundPlannerManager::checkTrajCollisionInflate(NonUniformBspline &traj) {

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

bool ReboundPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                           Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj) {

  static int count = 0;
  std::cout << endl << "[rebo replan]: -------------------------------------" << count++ << std::endl;
  cout.precision(3);
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
       << endl;

  if ((start_pt - local_target_pt).norm() < 0.2) {
    cout << "Close to goal" << endl;
    continous_failures_count_++;
    return false;
  }

  // cout << "continous_failures_count_=" << continous_failures_count_ << endl;

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
                                              (((double)rand())/RAND_MAX-0.5)*(start_pt-local_target_pt).norm()*horizen_dir*0.8*(-0.978/(continous_failures_count_+0.989)+0.989) + 
                                              (((double)rand())/RAND_MAX-0.5)*(start_pt-local_target_pt).norm()*vertical_dir*0.4*(-0.978/(continous_failures_count_+0.989)+0.989);
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
            continous_failures_count_++;
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

  // cout << "distance=" << (start_pt-local_target_pt).norm() << endl;
  // cout << "point_set.size()=" << point_set.size() << endl;
  // cout << "ts=" << ts << endl;
  // cout << "all_time=" << ts*(point_set.size()-1) << endl;

  Eigen::MatrixXd ctrl_pts;
  //ts = 3.25/(point_set.size()-1);
  //ts *= 1.5;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  if ( ctrl_pts.rows() <= 2*bspline_optimizer_rebound_->getOrder() ) // Drone is very close to the goal point
  {
    continous_failures_count_ = 0;
    return true;
  }
  
  vector<Eigen::Vector3d> raw_cps;
  vector<vector<Eigen::Vector3d>> a_star_pathes;
  for ( int i=0; i<ctrl_pts.rows(); ++i )
    raw_cps.push_back( ctrl_pts.row(i).transpose() );

  // cout << "raw_cps.size()=" << raw_cps.size() << endl;

  a_star_pathes = bspline_optimizer_rebound_->initControlPoints( raw_cps, true );

  t_search = (ros::Time::now() - t1).toSec();

  static int vis_id=0;
  visualization_->displayInitList(point_set, 0.2, 0);
  visualization_->displayAStarList(a_star_pathes,vis_id);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  
  bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ctrl_pts, ts);
  cout << "first_optimize_step_success=" << flag_step_1_success << endl;
  if ( !flag_step_1_success )
  {
    // visualization_->displayOptimalList( ctrl_pts, vis_id );
    continous_failures_count_++;
    return false;
  } 
  visualization_->displayOptimalList( ctrl_pts, vis_id );

  t_opt = (ros::Time::now() - t1).toSec();

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
  double to = pos.getTimeSum();

  double ratio;
  bool flag_step_2_success = true;
  if ( !pos.checkFeasibility(ratio, false) )
  {
    cout << "infeasible" << endl;

    Eigen::MatrixXd optimal_control_points;
    flag_step_2_success = refineTrajAlgo2(pos, start_end_derivatives, ratio, ts, optimal_control_points);
    if ( flag_step_2_success )
      pos = NonUniformBspline(optimal_control_points, 3, ts);
  }
  else
  {
    cout << "feasible" << endl;

    // double t_step = pos.getTimeSum() / (pos.getControlPoint().rows()-3);
    // bspline_optimizer_rebound_->ref_pts_.clear();
    // for ( double t=0; t<pos.getTimeSum()+1e-4; t+=t_step )
    //   bspline_optimizer_rebound_->ref_pts_.push_back( pos.evaluateDeBoorT(t) );
    // flag_step_2_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);
    // if ( flag_step_2_success )
    //   pos = NonUniformBspline(optimal_control_points, 3, ts) ;
  }

  // cout << "ctrl_pts=" << endl;
  // cout << ctrl_pts << endl << endl;

  // cout << "ref_pts_=" << endl;
  // for ( auto e : bspline_optimizer_rebound_->ref_pts_ )
  // {
  //   cout << e.transpose() << endl;
  // }

  // cout << "optimal_control_points=" << endl;
  // cout << optimal_control_points << endl << endl;

  if ( !flag_step_2_success )
  {
    printf("\033[33mThis refined trajectory hits obstacles. It doesn't matter if appeares rarely. But if keeps appearing, Increase parameter \"lambda_collision\".\n\033[0m");
    continous_failures_count_++;
    return false;
  }

  double tn = pos.getTimeSum();

  //cout << "[rebo replan]: Reallocate ratio: " << tn / to << endl;
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

  // success. YoY
  continous_failures_count_ = 0;
  return true;
}

bool ReboundPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
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

bool ReboundPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
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

bool ReboundPlannerManager::refineTrajAlgo2(NonUniformBspline& traj, vector<Eigen::Vector3d>& start_end_derivative, double ratio, double& ts, Eigen::MatrixXd& optimal_control_points) {
  double    t_inc;

  Eigen::MatrixXd ctrl_pts;      // = traj.getControlPoint()

  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

  traj = NonUniformBspline(ctrl_pts, 3, ts);
  
  double t_step = traj.getTimeSum() / (ctrl_pts.rows()-3);
  bspline_optimizer_rebound_->ref_pts_.clear();
  for ( double t=0; t<traj.getTimeSum()+1e-4; t+=t_step )
    bspline_optimizer_rebound_->ref_pts_.push_back( traj.evaluateDeBoorT(t) );

  bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

  return success;
}

void ReboundPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void ReboundPlannerManager::reparamBspline(NonUniformBspline& bspline, vector<Eigen::Vector3d>& start_end_derivative, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
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
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

}  // namespace rebound_planner
