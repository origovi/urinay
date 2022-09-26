#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(ros::NodeHandle *const nh) {
  std::string ns = ros::this_node::getName();
  // Main
  main.package_path = ros::package::getPath("urinay");
  nh->param<std::string>(ns + "/input_topic", main.input_topic, "/AS/P/ccat/cones");
  nh->param<std::string>(ns + "/input_pose_topic", main.input_pose_topic, "/limovelo/state");
  nh->param<std::string>(ns + "/output_full_topic", main.output_full_topic, "/AS/P/tracklimits/full");
  nh->param<std::string>(ns + "/output_partial_topic", main.output_partial_topic, "/AS/P/tracklimits/partial");

  // Visualization
  nh->param<bool>(ns + "/publish_markers", visualization.publish_markers, false);
  nh->param<std::string>(ns + "/marker_topics/triangulation", visualization.triangulation_topic, "/AS/P/urinay/markers/triangulation");
  nh->param<std::string>(ns + "/marker_topics/midpoints", visualization.midpoints_topic, "/AS/P/urinay/markers/midpoints");
  nh->param<std::string>(ns + "/marker_topics/way", visualization.way_topic, "/AS/P/urinay/markers/way");
  // nh->param<std::string>(ns + "/marker_topics/markers_full_topic", visualization.markers_full_topic, "/AS/P/urinay/markers/full");
  // nh->param<std::string>(ns + "/marker_topics/markers_partial_topic", visualization.markers_partial_topic, "/AS/P/urinay/markers/partial");

  // WayComputer
  nh->param<double>(ns + "/max_triangle_edge_len", wayComputer.max_triangle_edge_len, 9.0);
  nh->param<double>(ns + "/min_triangle_angle", wayComputer.min_triangle_angle, 0.25);
  nh->param<double>(ns + "/max_dist_circum_midPoint", wayComputer.max_dist_circum_midPoint, 1.0);
  nh->param<int>(ns + "/max_search_tree_height", wayComputer.max_search_tree_height, 5);
  nh->param<double>(ns + "/search_radius", wayComputer.search_radius, 5.0);
  nh->param<int>(ns + "/max_search_options", wayComputer.max_search_options, 2);
  nh->param<float>(ns + "/max_next_heuristic", wayComputer.max_next_heuristic, 3.0);
  nh->param<float>(ns + "/heur_dist_ponderation", wayComputer.heur_dist_ponderation, 0.6);
  nh->param<float>(ns + "/max_treeSearch_time", wayComputer.max_treeSearch_time, 0.05);
}