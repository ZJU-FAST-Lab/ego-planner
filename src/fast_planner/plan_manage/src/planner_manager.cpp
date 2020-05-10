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
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_rebound) {
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(edt_environment_);
    bspline_optimizer_rebound_->a_star_.reset( new AStar );
    bspline_optimizer_rebound_->a_star_->initGridMap(edt_environment_, Eigen::Vector3i(100,100,100));
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }

  visualization_ = vis;
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

bool FastPlannerManager::checkTrajCollisionInflate(NonUniformBspline &traj) {

  double tm, tmp;
  traj.getTimeSpan(tm, tmp);

  constexpr double t_step = 0.02;
  for ( double t = tm; t<tmp; t+=t_step )
  {
    if ( edt_environment_->sdf_map_->getInflateOccupancy( traj.evaluateDeBoor(t) ) )
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

  // Eigen::Vector3d init_pos = start_pt;
  // Eigen::Vector3d init_vel = start_vel;
  // Eigen::Vector3d init_acc = start_acc;

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

        // std::cout << "one_segment_traj_gen" << std::endl;
        // std::cout << gl_traj.evaluateAcc(time).transpose() << std::endl;
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

        // cout << "start_pt=" << start_pt.transpose() << " local_target_pt=" << local_target_pt.transpose() << " random_inserted_pt=" << random_inserted_pt.transpose() << endl;
        // for ( int i=0; i<10; i++ )
        //   ROS_ERROR("%.3f",((double)rand())/RAND_MAX-0.5);
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
      //start_end_derivatives.push_back( gl_traj.evaluateVel(t) );
      start_end_derivatives.push_back( local_target_vel );
      start_end_derivatives.push_back( gl_traj.evaluateAcc(0) );
      start_end_derivatives.push_back( gl_traj.evaluateAcc(t) );

    }
    else
    {

      double t;
      double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

      //cout << "t_cur=" << t_cur << " local_data_.duration_=" << local_data_.duration_ << " getTimeSum=" << local_data_.position_traj_.getTimeSum() << " ts=" << ts << endl;

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

      // int i;
      // for ( i=0; i<segment_point.size(); i++ )
      //   cout << segment_point[i].transpose() << endl;
      // cout << endl;

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
      // ROS_ERROR("%d",segment_point.size());
      
      // for ( ; i<segment_point.size(); i++ )
      //   cout << segment_point[i].transpose() << endl;
      // cout << endl;
      
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
  
  
  // for ( auto p : point_set )
  //   cout << p.transpose() << endl;
  



  Eigen::MatrixXd ctrl_pts;
  //ts = 3.25/(point_set.size()-1);
  //ts *= 1.5;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  // auto bsp = NonUniformBspline(ctrl_pts, 3, ts);

  // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
  // cout << "pos=" << endl;
  // for ( int i=0; i<bsp.get_control_points().rows(); i++ )
  //   cout << bsp.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "vel=" << endl;
  // for ( int i=0; i<bsp.getDerivative().get_control_points().rows(); i++ )
  //   cout << bsp.getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc=" << endl;
  // for ( int i=0; i<bsp.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << bsp.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;

  // cout << "bsp=" << endl;
  // cout << bsp.evaluateDeBoorT(0).transpose() << endl; 
  // cout << bsp.getDerivative().evaluateDeBoorT(0).transpose() << endl; 
  // cout << bsp.getDerivative().getDerivative().evaluateDeBoorT(0).transpose() << endl; 

  if ( ctrl_pts.rows() <= 2*bspline_optimizer_rebound_->getOrder() ) // Drone is very close to the goal point
    return true;

// for ( int i=0; i<ctrl_pts.rows(); i++ )
//   cout << ctrl_pts.row(i) << endl;
// cout << endl;
  
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
 
  // local_data_.position_traj_     = NonUniformBspline(ctrl_pts, 3, ts);
  // local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  // local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  // local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  // local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.position_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;
  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.velocity_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;
  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.acceleration_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;


// for ( int i=0; i<ctrl_pts.rows(); i++ )
//   cout << ctrl_pts.row(i) << endl;
// cout << endl;

  t_opt = (ros::Time::now() - t1).toSec();
  // iterative time adjustment

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  // cout << "planner manager AAA" << endl;
  // for ( double t=0; t<pos.getTimeSum(); t+=0.01 )
  //   cout << pos.getDerivative().evaluateDeBoorT(t).transpose() << endl;

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);
  double time_inc;
  Eigen::MatrixXd optimal_control_points;

  // cout << "pos before=" << endl;
  // for ( int i=0; i<pos.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << pos.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc before=" << endl;
  // for ( int i=0; i<pos.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << pos.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << "time span=" << pos.getTimeSum() << endl;
  // cout << endl;


  // cout << "pos before=" << endl;
  // for ( int i=0; i<pos.get_control_points().rows(); i++ )
  //   cout << pos.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc before=" << endl;
  // for ( int i=0; i<pos.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << pos.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;

  bool flag_step_2_success;
  if ( !feasible )
  {
    plan_data_.local_start_end_derivative_ = start_end_derivatives;
    // cout << "start_end_derivatives=" << endl;
    // cout << start_end_derivatives[0].transpose() << endl;
    // cout << start_end_derivatives[1].transpose() << endl;
    // cout << start_end_derivatives[2].transpose() << endl;
    // cout << start_end_derivatives[3].transpose() << endl;
    flag_step_2_success = refineTrajAlgo2(pos, time_inc, ts, optimal_control_points);
    if ( flag_step_2_success )
      pos = NonUniformBspline(optimal_control_points, 3, ts);

    cout << "infeasible" << endl;
  }
  else
  {
    cout << "feasible" << endl;
    // cout << "pos=" << endl;
    // for ( int i=0; i<pos.get_control_points().rows(); i++ )
    //   cout << pos.get_control_points().row(i) << endl ;
    // cout << endl;
    // cout << "vel=" << endl;
    // for ( int i=0; i<pos.getDerivative().get_control_points().rows(); i++ )
    //   cout << pos.getDerivative().get_control_points().row(i) << endl ;
    // cout << endl;
    // cout << "acc=" << endl;
    // for ( int i=0; i<pos.getDerivative().getDerivative().get_control_points().rows(); i++ )
    //   cout << pos.getDerivative().getDerivative().get_control_points().row(i) << endl ;
    // cout << endl;

    // bspline_optimizers_[0]->ref_pts_.clear();
    // for ( int i=1; i<ctrl_pts.rows()-1; i++ )
    //   bspline_optimizers_[0]->ref_pts_.push_back(ctrl_pts.row(i).transpose());
    double t_step = pos.getTimeSum() / (pos.getControlPoint().rows()-3);
    bspline_optimizers_[0]->ref_pts_.clear();
    for ( double t=0; t<pos.getTimeSum()+1e-4; t+=t_step )
      bspline_optimizers_[0]->ref_pts_.push_back( pos.evaluateDeBoorT(t) );
    flag_step_2_success = bspline_optimizers_[0]->BsplineOptimizeTrajRefine(ctrl_pts, ts, 0.1/*seconds*/, optimal_control_points);
    if ( flag_step_2_success )
      pos = NonUniformBspline(optimal_control_points, 3, ts) ;

    // cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << endl;
    // cout << "pos=" << endl;
    // for ( int i=0; i<pos.get_control_points().rows(); i++ )
    //   cout << pos.get_control_points().row(i) << endl ;
    // cout << endl;
    // cout << "vel=" << endl;
    // for ( int i=0; i<pos.getDerivative().get_control_points().rows(); i++ )
    //   cout << pos.getDerivative().get_control_points().row(i) << endl ;
    // cout << endl;
    // cout << "acc=" << endl;
    // for ( int i=0; i<pos.getDerivative().getDerivative().get_control_points().rows(); i++ )
    //   cout << pos.getDerivative().getDerivative().get_control_points().row(i) << endl ;
    // cout << endl;
  }

  if ( !flag_step_2_success )
  {

    // NonUniformBspline pos_after = NonUniformBspline(optimal_control_points, 3, ts);
    // cout << "pos after=" << endl;
    // for ( int i=0; i<pos_after.get_control_points().rows(); i++ )
    //   cout << pos_after.get_control_points().row(i) << endl ;
    // cout << endl;
    // cout << "acc after=" << endl;
    // for ( int i=0; i<pos_after.getDerivative().getDerivative().get_control_points().rows(); i++ )
    //   cout << pos_after.getDerivative().getDerivative().get_control_points().row(i) << endl ;
    // cout << endl;

    ROS_WARN("Failed to refine trajectory, but I can not do more :(");
  }

  // int iter_num = 0;
  // while (!feasible && ros::ok()) {

  //   feasible = pos.reallocateTime_mod2();

  //   if (++iter_num >= 3) break;
  // }

  // cout << pos.evaluateDeBoorT(0).transpose() << endl;
  // cout << pos.getDerivative().evaluateDeBoorT(0).transpose() << endl;
  // cout << pos.getDerivative().getDerivative().evaluateDeBoorT(0).transpose() << endl;

  // cout << "planner manager BBB" << endl;
  // for ( double t=0; t<pos.getTimeSum(); t+=0.01 )
  //   cout << pos.getDerivative().evaluateDeBoorT(t).transpose() << endl;

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  // ts *= tn/to;
  // ctrl_pts = pos.get_control_points();
  // point_set.clear();
  // for ( int i=0; i<ctrl_pts.rows(); i++ ) point_set.push_back( ctrl_pts.row(i).transpose() );
  // NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  // pos = NonUniformBspline( ctrl_pts, 3, ts );

  cout << "[rebo replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  local_data_.position_traj_ = pos;
  local_data_.start_time_ = ros::Time::now();
  updateTrajInfo();

  // if ( abs(local_data_.velocity_traj_.evaluateDeBoorT(0.0)(0)) < 0.5 )
  // {
  //   cout << "\033[31m";
  //   cout << local_data_.position_traj_.evaluateDeBoorT(0.0).transpose() << endl;
  //   cout << local_data_.velocity_traj_.evaluateDeBoorT(0.0).transpose() << endl;
  //   cout << local_data_.acceleration_traj_.evaluateDeBoorT(0.0).transpose() << endl;
  //   cout << "\033[0m";
  // }

  // bool flag_infeasi = true;
  // for ( double t=0; t<local_data_.duration_; t+=0.1 )
  // {
  //   if ( local_data_.acceleration_traj_.evaluateDeBoorT(t).cwiseAbs().maxCoeff() > 2 )
  //   {
  //     cout << local_data_.acceleration_traj_.evaluateDeBoorT(t).transpose() << endl;
  //     flag_infeasi = true;
  //   }
  // }
  // cout << endl;
  // if ( flag_infeasi )
  // {
  //   for ( int i=0; i<ctrl_pts.rows(); i++ )
  //     cout << ctrl_pts.row(i) << endl;
  //   cout << endl;
  //   for ( int i=0; i<local_data_.position_traj_.get_control_points().rows(); i++ )
  //     cout << local_data_.position_traj_.get_control_points().row(i) << endl ;
  //   cout << endl;
  //   for ( int i=0; i<local_data_.position_traj_.getKnot().rows(); i++ )
  //     cout << local_data_.position_traj_.getKnot().row(i) << endl;
  //   cout << endl;
  // }

  
  // cout << "pos after=" << endl;
  // for ( int i=0; i<local_data_.position_traj_.get_control_points().rows(); i++ )
  //   cout << local_data_.position_traj_.get_control_points().row(i) << endl ;
  // cout << endl;
  // for ( int i=0; i<local_data_.velocity_traj_.get_control_points().rows(); i++ )
  //   cout << local_data_.velocity_traj_.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc after=" << endl;
  // for ( int i=0; i<local_data_.acceleration_traj_.get_control_points().rows(); i++ )
  //   cout << local_data_.acceleration_traj_.get_control_points().row(i) << endl ;
  // cout << "time span=" << local_data_.acceleration_traj_.getTimeSum() << endl;
  // cout << endl;

  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.position_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;
  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.velocity_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;
  // for ( double t=0; t<local_data_.duration_; t+=0.02 )
  //   cout << local_data_.acceleration_traj_.evaluateDeBoorT(t).transpose() << " " << t << endl;
  // cout << endl;

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

bool FastPlannerManager::slidingWindow(Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel)
{
  /*** step 1: get point_set ***/
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;

  double t;
  double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

  //cout << "t_cur=" << t_cur << " local_data_.duration_=" << local_data_.duration_ << " getTimeSum=" << local_data_.position_traj_.getTimeSum() << " ts=" << ts << endl;

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

  // ROS_ERROR("%d",segment_point.size());

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
  // ROS_ERROR("%d",segment_point.size());
  
  // for ( ; i<segment_point.size(); i++ )
  //   cout << segment_point[i].transpose() << endl;
  // cout << endl;
  
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
  // double to = pos.getTimeSum();
  // pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  // bool feasible = pos.checkFeasibility(false);

  // int iter_num = 0;
  // while (!feasible && ros::ok()) {

  //   feasible = pos.reallocateTime();

  //   if (++iter_num >= 3) break;
  // }

  // // pos.checkFeasibility(true);
  // // cout << "[Main]: iter num: " << iter_num << endl;

  // double tn = pos.getTimeSum();

  // cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  // if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  /***  ***/
  local_data_.position_traj_ = pos;
  local_data_.start_time_ = ros::Time::now();
  updateTrajInfo();
    
  // for ( auto p : start_end_derivatives )
  // {
  //   cout << p.transpose() << endl;
  // }
  // cout << endl;
  // bool flag_infeasi = false;
  // for ( double t=0; t<local_data_.duration_; t+=0.1 )
  // {
  //   if ( local_data_.acceleration_traj_.evaluateDeBoorT(t).cwiseAbs().maxCoeff() > 2 )
  //   {
  //     cout << local_data_.acceleration_traj_.evaluateDeBoorT(t).transpose() << endl;
  //     flag_infeasi = true;
  //   }
  // }
  // cout << endl;
  // if ( flag_infeasi )
  // {
  //   for ( int i=0; i<point_set.size(); i++ )
  //     cout << point_set[i].transpose() << endl;
  //   cout << endl;
  //   for ( int i=0; i<ctrl_pts.rows(); i++ )
  //     cout << ctrl_pts.row(i) << endl;
  //   cout << endl;
  //   for ( int i=0; i<local_data_.position_traj_.get_control_points().rows(); i++ )
  //     cout << local_data_.position_traj_.get_control_points().row(i) << endl ;
  //   cout << endl;
  //   for ( int i=0; i<local_data_.acceleration_traj_.get_control_points().rows(); i++ )
  //     cout << local_data_.acceleration_traj_.get_control_points().row(i) << endl ;
  //   cout << endl;
  //   for ( int i=0; i<local_data_.position_traj_.getKnot().rows(); i++ )
  //     cout << local_data_.position_traj_.getKnot().row(i) << endl;
  //   cout << endl;
  // }

  /*** step 3: publish ***/
  return true;
}

// !SECTION

// SECTION topological replanning

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

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  local_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline

  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // generate global reference trajectory

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

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

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
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

  // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
  // cout << "pos=" << endl;
  // for ( int i=0; i<traj.get_control_points().rows(); i++ )
  //   cout << traj.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "vel=" << endl;
  // for ( int i=0; i<traj.getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc=" << endl;
  // for ( int i=0; i<traj.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;


  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(traj, ratio, ctrl_pts, ts, t_inc);
  time_inc += t_inc;

  traj = NonUniformBspline(ctrl_pts, 3, ts);


  // cout << "pos before=" << endl;
  // for ( int i=0; i<traj.get_control_points().rows(); i++ )
  //   cout << traj.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc before=" << endl;
  // for ( int i=0; i<traj.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << "time span=" << traj.getTimeSum() << endl;
  // cout << endl;

  // cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << endl;
  // cout << "pos=" << endl;
  // for ( int i=0; i<traj.get_control_points().rows(); i++ )
  //   cout << traj.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "vel=" << endl;
  // for ( int i=0; i<traj.getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc=" << endl;
  // for ( int i=0; i<traj.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  
  double t_step = traj.getTimeSum() / (ctrl_pts.rows()-3);
  bspline_optimizers_[0]->ref_pts_.clear();
  for ( double t=0; t<traj.getTimeSum()+1e-4; t+=t_step )
    bspline_optimizers_[0]->ref_pts_.push_back( traj.evaluateDeBoorT(t) );

  bool success = bspline_optimizers_[0]->BsplineOptimizeTrajRefine(ctrl_pts, ts, 0.1/*seconds*/, optimal_control_points);

  // ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
  //                                   << " seconds, time change is: " << time_inc);

  // cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << endl;
  // cout << "pos=" << endl;
  // for ( int i=0; i<traj.get_control_points().rows(); i++ )
  //   cout << traj.get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "vel=" << endl;
  // for ( int i=0; i<traj.getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;
  // cout << "acc=" << endl;
  // for ( int i=0; i<traj.getDerivative().getDerivative().get_control_points().rows(); i++ )
  //   cout << traj.getDerivative().getDerivative().get_control_points().row(i) << endl ;
  // cout << endl;

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

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  ros::Time t1;
  double    tm1, tm2, tm3;

  t1 = ros::Time::now();

  // parameterize B-spline according to the length of guide path
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

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
