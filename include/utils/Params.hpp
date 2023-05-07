/**
 * @file Params.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Params class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ros/package.h>
#include <ros/ros.h>

/**
 * @brief Represents all the parameters that the program needs divided into
 * modules and structures.
 * The class pretends to be a bridge between the .yaml and the program modules
 * and structures.
 * When constructed, it fills all parameters using ROS param.
 */
class Params {
 public:
  Params(ros::NodeHandle *const nh);
  struct Main {
    std::string input_cones_topic, input_pose_topic, output_full_topic, output_partial_topic;
    std::string markers_full_topic, markers_partial_topic;
    std::string package_path;
    bool shutdown_on_loop_closure;
    float min_cone_confidence;
  } main;
  struct WayComputer {
    double max_triangle_edge_len, min_triangle_angle, max_dist_circum_midPoint;
    int failsafe_max_way_horizon_size;
    bool general_failsafe;
    double general_failsafe_safetyFactor;
    struct Search {
      int max_way_horizon_size;
      int max_search_tree_height;
      double search_radius, max_angle_diff, edge_len_diff_factor;
      int max_search_options;
      double max_next_heuristic;
      float heur_dist_ponderation;
      bool allow_intersection;
      float max_treeSearch_time;
    } search;
    struct Way {
      double max_dist_loop_closure;
      double max_angle_diff_loop_closure;
      int vital_num_midpoints;
    } way;
  } wayComputer;
  struct Visualization {
    bool publish_markers;
    std::string triangulation_topic;
    std::string midpoints_topic;
    std::string way_topic;
  } visualization;
};