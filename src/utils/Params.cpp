/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(ros::NodeHandle *const nh) {
  std::string ns = ros::this_node::getName();
  // Main
  main.package_path = ros::package::getPath("urinay");
  nh->param<std::string>(ns + "/input_cones_topic", main.input_cones_topic, "/AS/P/ccat/cones");
  nh->param<std::string>(ns + "/input_pose_topic", main.input_pose_topic, "/AS/C/state");
  nh->param<std::string>(ns + "/output_full_topic", main.output_full_topic, "/AS/P/tracklimits/full");
  nh->param<std::string>(ns + "/output_partial_topic", main.output_partial_topic, "/AS/P/tracklimits/partial");
  nh->param<bool>(ns + "/shutdown_on_loop_closure", main.shutdown_on_loop_closure, true);
  nh->param<float>(ns + "/min_cone_confidence", main.min_cone_confidence, 0.0);
  nh->param<bool>(ns + "/verbose", main.verbose, false);

  // WayComputer
  nh->param<double>(ns + "/max_triangle_edge_len", wayComputer.max_triangle_edge_len, 9.0);
  nh->param<double>(ns + "/min_triangle_angle", wayComputer.min_triangle_angle, 0.25);
  nh->param<int>(ns + "/failsafe_max_way_horizon_size", wayComputer.failsafe_max_way_horizon_size, 6);
  nh->param<bool>(ns + "/general_failsafe", wayComputer.general_failsafe, true);
  nh->param<double>(ns + "/general_failsafe_safetyFactor", wayComputer.general_failsafe_safetyFactor, 1.4);
  // WayComputer::Search
  nh->param<int>(ns + "/max_way_horizon_size", wayComputer.search.max_way_horizon_size, 0);
  nh->param<int>(ns + "/best_search_options_to_keep", wayComputer.search.best_search_options_to_keep, 50);
  nh->param<double>(ns + "/search_radius", wayComputer.search.search_radius, 5.0);
  nh->param<double>(ns + "/max_angle_diff", wayComputer.search.max_angle_diff, 0.6);
  nh->param<double>(ns + "/min_track_width", wayComputer.search.min_track_width, 2.8);
  nh->param<int>(ns + "/max_search_options", wayComputer.search.max_search_options, 2);
  nh->param<double>(ns + "/overnext_midpoint_angle", wayComputer.search.overnext_midpoint_angle, 0.2);
  nh->param<double>(ns + "/max_next_heuristic", wayComputer.search.max_next_heuristic, 3.0);
  nh->param<float>(ns + "/heur_dist_weight", wayComputer.search.heur_dist_weight, 0.6);
  nh->param<float>(ns + "/heur_track_width_diff_weight", wayComputer.search.heur_track_width_diff_weight, 0.2);
  nh->param<bool>(ns + "/allow_intersection", wayComputer.search.allow_intersection, false);
  nh->param<double>(ns + "/min_dist_between_midpoints", wayComputer.search.min_dist_between_midpoints, 0.09);
  nh->param<int>(ns + "/extra_tree_height_closure", wayComputer.search.extra_tree_height_closure, 7);

  // WayComputer::Way
  nh->param<double>(ns + "/max_dist_loop_closure", wayComputer.way.max_dist_loop_closure, 1.0);
  nh->param<double>(ns + "/max_angle_diff_loop_closure", wayComputer.way.max_angle_diff_loop_closure, 0.6);
  nh->param<int>(ns + "/vital_num_midpoints", wayComputer.way.vital_num_midpoints, 5);

  // Visualization
  nh->param<bool>(ns + "/publish_markers", visualization.publish_markers, false);
  nh->param<std::string>(ns + "/marker_topics/triangulation", visualization.triangulation_topic, "/AS/P/urinay/markers/triangulation");
  nh->param<std::string>(ns + "/marker_topics/way", visualization.way_topic, "/AS/P/urinay/markers/way");
  nh->param<std::string>(ns + "/marker_topics/traceBuffer", visualization.traceBuffer_topic, "/AS/P/urinay/markers/traceBuffer");
}