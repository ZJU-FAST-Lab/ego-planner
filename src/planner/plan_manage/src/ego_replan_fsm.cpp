
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    nh.param("fsm/control_zero_reset", control_zero_reset_, false);
    // control_zero_reset_ = false;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.5), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    last_control_ = Eigen::Vector3d(0, 0, 0);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
      control_sub_ = nh.subscribe("/waypoint_generator/waypoint_manual", 1, &EGOReplanFSM::controlCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  double EGOReplanFSM::getRemainLength(Eigen::Vector3d point, ego_planner::UniformBspline &bspline) {
    const double tol = 0.01, dt = tol * 0.49;
    double t_start = 0, t_end = bspline.getTimeSum();

    if (t_end <= 0) return 0;
    
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

    for (double t = t_start; t <= t_end; t += res)
    {
      p_n = bspline.evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }

    return length;
  }

  UniformBspline EGOReplanFSM::generateTraj(const vector<Eigen::Vector3d> &traj_pts) {
    Eigen::MatrixXd pos_pts(3, traj_pts.size());

    for (size_t i = 0; i < traj_pts.size(); i++)
    {
      pos_pts(0, i) = traj_pts[i].x();
      pos_pts(1, i) = traj_pts[i].y();
      pos_pts(2, i) = traj_pts[i].z();
    }
    
    return UniformBspline(pos_pts, 3, 1.0);
  }

  void EGOReplanFSM::controlCallback(const geometry_msgs::PoseConstPtr &msg) {
    if (abs(msg->position.x) < 0.05 && abs(msg->position.y) < 0.05) {
      if (control_zero_reset_) {
        // cancel exist waypoint and stop
        traj_pts_.clear();
        visualization_->displayTrajList(traj_pts_, 0);

        last_control_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

        have_target_ = false;
        have_new_target_ = false;
        changeFSMExecState(WAIT_TARGET, "TRIG");

      } else if (is_need_replan_) {
        traj_pts_.clear();
        visualization_->displayTrajList(traj_pts_, 0);

        geometry_msgs::PoseStamped waypoint;
        waypoint.pose.position.x = end_pt_.x();
        waypoint.pose.position.y = end_pt_.y();
        waypoint.pose.position.z = end_pt_.z();

        nav_msgs::Path waypoints;
        waypoints.poses.push_back(waypoint);
        
        waypointCallback(waypoints);

      } else if ((end_pt_ - odom_pos_).norm() < 0.3) {
        traj_pts_.clear();
        visualization_->displayTrajList(traj_pts_, 0);

        have_target_ = false;
        have_new_target_ = false;
        changeFSMExecState(WAIT_TARGET, "TRIG");
      }

    } else {
      Eigen::Vector3d control(msg->position.x, msg->position.y, msg->position.z);

      if ((last_control_ - control).norm() < 0.1) {

        if (current_traj_.getControlPoint().size() > 2) {
          double remain_length = getRemainLength(odom_pos_, current_traj_);
          double total_length = current_traj_.getLength();

          if (remain_length < max(min(total_length * 0.3, 12.0), 0.5)) {
            is_need_replan_ = true;
          }
        }

        if (!is_need_replan_) return;
      }

      last_control_ = control;

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      Eigen::AngleAxisd rot_yaw(atan2(rot_x(1), rot_x(0)), Eigen::Vector3d::UnitZ());
      Eigen::Vector3d converted_control(control.x() * 20.0, control.y() * 20.0, control.z() * 2.0);
      converted_control = rot_yaw * converted_control + odom_pos_;

      geometry_msgs::PoseStamped waypoint;
      waypoint.pose.position.x = converted_control.x();
      waypoint.pose.position.y = converted_control.y();
      waypoint.pose.position.z = converted_control.z();

      nav_msgs::Path waypoints;
      waypoints.poses.push_back(waypoint);
      
      waypointCallback(waypoints);
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::Path &msg)
  {
    if (msg.poses[0].pose.position.z < -0.1)
      return;

    if (exec_state_ == REPLAN_TRAJ) return;

    is_need_replan_ = false;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg.poses[0].pose.position.x, msg.poses[0].pose.position.y, msg.poses[0].pose.position.z;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    odom_vel_ = odom_orient_ * odom_vel_; // fix for PX4

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      traj_pts_.clear();
      traj_pts_.push_back(start_pt_);

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else if (exec_state_ == GEN_NEW_TRAJ)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else if (exec_state_ == REPLAN_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      info->start_time_ -= ros::Duration(0.501);
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        traj_pts_.push_back(end_pt_);
        visualization_->displayTrajList(traj_pts_, 0);
        current_traj_ = generateTraj(traj_pts_);
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    traj_pts_.push_back(start_pt_);
    visualization_->displayTrajList(traj_pts_, 0);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      if (planner_manager_->getFailuresCount() < 0) return false;

      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    /* ---------- check trajectory ---------- */
    auto map = planner_manager_->grid_map_;
    double t_end = current_traj_.getTimeSum();

    for (double t = 0; t < t_end; t += 0.1) {
      if (map->getInflateOccupancy(current_traj_.evaluateDeBoorT(t))) {
        is_need_replan_ = true;
        break;
      }
    }

    // LocalTrajData *info = &planner_manager_->local_data_;
    // auto map = planner_manager_->grid_map_;

    // if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
    //   return;

    // /* ---------- check trajectory ---------- */
    // constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    // double t_2_3 = info->duration_ * 2 / 3;
    // for (double t = t_cur; t < info->duration_; t += time_step)
    // {
    //   if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
    //     break;

    //   if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
    //   {
    //     if (planFromCurrentTraj()) // Make a chance
    //     {
    //       changeFSMExecState(EXEC_TRAJ, "SAFETY");
    //       return;
    //     }
    //     else
    //     {
    //       if (t - t_cur < emergency_time_) // 0.8s of emergency time
    //       {
    //         ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
    //         changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    //       }
    //       else
    //       {
    //         //ROS_WARN("current traj in collision, replan.");
    //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //       }
    //       return;
    //     }
    //     break;
    //   }
    // }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);

      reboundReplan_fail_count_ = 0;

    } else {
      reboundReplan_fail_count_++;

      if (planner_manager_->getFailuresCount() < 0) {
        ROS_WARN("Remove wrong goal. reboundReplan_fail_count : %d", reboundReplan_fail_count_);

        traj_pts_.clear();
        visualization_->displayTrajList(traj_pts_, 0);

        have_target_ = false;
        have_new_target_ = false;
        changeFSMExecState(WAIT_TARGET, "TRIG");
      }
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;
    double start_t = planner_manager_->global_data_.last_progress_time_;
    double t_step = planning_horizen_ / 20.0 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = start_t; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < start_t + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        local_target_pt_ = end_pt_;
        local_target_vel_ = Eigen::Vector3d::Zero();
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
