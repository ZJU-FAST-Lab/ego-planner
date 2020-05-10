#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace fast_planner {
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
  pubs_.push_back(traj_pub_);

  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 20);
  pubs_.push_back(topo_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 20);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 20);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 20);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 20);
  pubs_.push_back(yaw_pub_);

  last_topo_path1_num_     = 0;
  last_topo_path2_num_     = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_       = 0;

  /************************** zxzx display ****************************/
  init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list",100);
  optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list",100);
  a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list",100);
  guide_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("guide_vector",100);
  intermediate_state_pub = nh.advertise<visualization_msgs::MarkerArray>("intermediate_state",100);
}


void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
  }
  last_bspline_phase1_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
                getColor(double(i) / bsplines.size()), i, i);
  }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50 + i) % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + (50 + i) % 100);
  }
  last_bspline_phase2_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.3), false, 1.5 * size,
                getColor(double(i) / bsplines.size()), 50 + i, 50 + i);
  }
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
  if (bspline.getControlPoint().size() == 0) return;

  vector<Eigen::Vector3d> traj_pts;
  double                  tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

  // draw the control point
  if (!show_ctrl_pts) return;

  Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctp;

  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }

  displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
}

void PlanningVisualization::drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size,
                                          double line_width, const Eigen::Vector4d& color1,
                                          const Eigen::Vector4d& color2, const Eigen::Vector4d& color3,
                                          int id) {
  // clear exsiting node and edge (drawn last time)
  vector<Eigen::Vector3d> empty;
  displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
  displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

  /* draw graph node */
  vector<Eigen::Vector3d> guards, connectors;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {

    if ((*iter)->type_ == GraphNode::Guard) {
      guards.push_back((*iter)->pos_);
    } else if ((*iter)->type_ == GraphNode::Connector) {
      connectors.push_back((*iter)->pos_);
    }
  }
  displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);

  /* draw graph edge */
  vector<Eigen::Vector3d> edge_pt1, edge_pt2;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
    for (int k = 0; k < (*iter)->neighbors_.size(); ++k) {

      edge_pt1.push_back((*iter)->pos_);
      edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
    }
  }
  displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths,
                                                double                           line_width) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path1_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
    displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
  }

  last_topo_path1_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, line_width, getColor(double(i) / (last_topo_path1_num_)),
                    SELECT_PATH + i % 100, 1);
  }
}

void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double size) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path2_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, size, color1, FILTERED_PATH + i % 100, 1);
  }

  last_topo_path2_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, size, getColor(double(i) / (last_topo_path2_num_), 0.2),
                    FILTERED_PATH + i % 100, 1);
  }
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                                              const Eigen::Vector4d& color, int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                               const Eigen::Vector4d& color, int id) {
  poly_traj.init();
  vector<Eigen::Vector3d> poly_pts = poly_traj.getTraj();
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawPrediction(ObjPrediction pred, double resolution,
                                           const Eigen::Vector4d& color, int id) {
  ros::Time    time_now   = ros::Time::now();
  double       start_time = (time_now - ObjHistory::global_start_time_).toSec();
  const double range      = 5.6;

  vector<Eigen::Vector3d> traj;
  for (int i = 0; i < pred->size(); i++) {

    PolynomialPrediction poly = pred->at(i);
    if (!poly.valid()) continue;

    for (double t = start_time; t <= start_time + range; t += 0.8) {
      Eigen::Vector3d pt = poly.evaluateConstVel(t);
      traj.push_back(pt);
    }
  }
  displaySphereList(traj, resolution, color, id % 100, 2);
}

void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw,
                                        const double& dt) {
  double                  duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.15;
    double          yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
}

void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw,
                                        const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
}

Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) {
    std::cout << "h out of range" << std::endl;
    h = 0.0;
  }

  double          lambda;
  Eigen::Vector4d color1, color2;
  if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

  } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

  } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

  } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

  } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

  } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3)              = alpha;

  return fcolor;
}







/************************** zxzx display ****************************/
/************************** zxzx display ****************************/
/************************** zxzx display ****************************/
/************************** zxzx display ****************************/
/************************** zxzx display ****************************/
/************************** zxzx display ****************************/

// real ids used: {id, id+1000}
void PlanningVisualization::displayMarkerList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale,
                                              Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "world";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 3;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  pub.publish(sphere);
  pub.publish(line_strip);
}

// real ids used: {id, id+1}
void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array, 
  const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "map";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 3;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  array.markers.push_back(sphere);
  array.markers.push_back(line_strip);
}

// real ids used: {1000*id ~ (arrow nums)+1000*id}
void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array, 
  const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp = ros::Time::now();
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;

  // geometry_msgs::Point start, end;
  // arrow.points

  arrow.color.r = color(0);
  arrow.color.g = color(1);
  arrow.color.b = color(2);
  arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  arrow.scale.x = scale;
  arrow.scale.y = 2 * scale;
  arrow.scale.z = 2 * scale;

  geometry_msgs::Point start, end;
  for (int i = 0; i < int(list.size() / 2); i++)
  {
    // arrow.color.r = color(0) / (1+i);
    // arrow.color.g = color(1) / (1+i);
    // arrow.color.b = color(2) / (1+i);

    start.x = list[2*i](0);
    start.y = list[2*i](1);
    start.z = list[2*i](2);
    end.x = list[2*i+1](0);
    end.y = list[2*i+1](1);
    end.z = list[2*i+1](2);
    arrow.points.clear();
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.id = i + id * 1000;

    array.markers.push_back(arrow);
  }
}

void PlanningVisualization::displayInitList(vector<Eigen::Vector3d> init_pts, int id)
{
    Eigen::Vector4d color(0,0,1,1);
    displayMarkerList(init_list_pub, init_pts, 0.1, color, id);
}

void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
{
    vector<Eigen::Vector3d> list;
    for ( int i = 0; i < optimal_pts.rows(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.row(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1,0,0,1);
    displayMarkerList(optimal_list_pub, list, 0.1, color,id);
}


void PlanningVisualization::displayAStarList( std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id/* = Eigen::Vector4d(0.5,0.5,0,1)*/)
{
  int i=0;
  vector<Eigen::Vector3d> list;

  Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
  double scale = 0.05 + (double)rand() / RAND_MAX / 10;

  // for ( int i=0; i<10; i++ )
  // {
  //   //Eigen::Vector4d color(1,1,0,0);
  //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
  // }


  for ( auto block : a_star_paths)
  {
    list.clear();
    for ( auto pt : block )
    {
      list.push_back(pt);
    }
    //Eigen::Vector4d color(0.5,0.5,0,1);
    displayMarkerList(a_star_list_pub, list, scale, color, id+i); // real ids used: [ id ~ id+a_star_paths.size() ]
    i++;
  }
}

void PlanningVisualization::displayArrowList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color, int id)
{
  visualization_msgs::MarkerArray array;
  // clear
  pub.publish(array);

  generateArrowDisplayArray(array, list, scale, color, id);

  pub.publish(array);
}

// void PlanningVisualization::displayIntermediateState(ros::Publisher& intermediate_pub, dyn_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration)
// {
//   visualization_msgs::MarkerArray array;
//   // clear
//   intermediate_pub.publish(array);

//   //for ( size_t j = 0; j < 8 && ros::ok() ; j++ )
//   //constexpr size_t START_ID = 0;
//   for ( size_t j = start_iteration; j < optimizer->q_output_.size() && ros::ok() ; j++ )
//   {
//     vector<Vector3d> pts = optimizer->q_output_[j];
//     Eigen::Vector4d color(0,1,1,1);
//     generatePathDisplayArray(array, pts, 0.1, color, 0); // real ids used: {id, id+1}

//     vector<Eigen::Vector3d> list;
//     vector<Vector3d> grads_sm = optimizer->grad_log_smooth_[j];
//     for ( size_t k = 0; k <  grads_sm.size(); k++ )
//     {
//       list.push_back(pts[k]);
//       list.push_back(pts[k] + grads_sm[k]);
//     }
//     color << 0, 1.0, 0.5, 1;
//     generateArrowDisplayArray(array, list, 0.02, color, 1);// real ids used: {1000*id ~ (arrow nums)+1000*id}

//     list.clear();
//     vector<Vector3d> grads_dist = optimizer->grad_log_dist_[j];
//     for ( size_t k = 0; k <  grads_dist.size(); k++ )
//     {
//       list.push_back(pts[k]);
//       list.push_back(pts[k] + grads_dist[k] + grads_sm[k] ); // all
//     }
//     color << 0, 0.9, 0.9, 1;
//     generateArrowDisplayArray(array, list, 0.02, color, 3);

//     list.clear();
//     for ( size_t k = 0; k <  grads_dist.size(); k++ )
//     {
//       list.push_back(pts[k]);
//       list.push_back(pts[k] + grads_dist[k]); // distance
//     }
//     color << 1, 0.0, 1.0, 1;
//     generateArrowDisplayArray(array, list, 0.02, color, 2);

//     intermediate_pub.publish(array);
//     cout << "\b\b\b\b" << j << flush;
//     ros::Duration(sleep_time).sleep();
//     ros::spinOnce();
//   }
//   cout << endl;
// }

// void PlanningVisualization::displayNewArrow(ros::Publisher& guide_vector_pub, dyn_planner::BsplineOptimizer::Ptr optimizer)
// {
//       vector<Vector3d> guide_vect_disp;
//       for ( size_t i=0; i<optimizer->constraint_type_.size(); i++ )
//       {
//         if ( optimizer->constraint_type_[i] == ConstraintType::ONE_SIDE )
//         {
//           guide_vect_disp.push_back(optimizer->guide_center_);
//           guide_vect_disp.push_back(optimizer->guide_center_ + optimizer->guide_vect_[i]);
//         }
//         else
//         {
//           guide_vect_disp.push_back(optimizer->opposite_guide_origin_pt_[i]);
//           guide_vect_disp.push_back(optimizer->guide_center_);
//         }
//       }

//       if ( optimizer->constraint_type_.size() > 0 )
//       {
//         Vector4d color(1,1,0,1);
//         displayArrowList(guide_vector_pub,guide_vect_disp,0.02,color,0);
//       }
// }




// PlanningVisualization::
}  // namespace fast_planner