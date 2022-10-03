#pragma once

#include <ros/package.h>
#include <ros/ros.h>

class Params {
 public:
  Params(ros::NodeHandle *const nh);
  struct Main {
    std::string input_topic, input_pose_topic, output_full_topic, output_partial_topic;
    std::string markers_full_topic, markers_partial_topic;
    std::string package_path;
    bool shutdown_on_loop_closure;
  } main;
  struct WayComputer {
    double max_triangle_edge_len, min_triangle_angle, max_dist_circum_midPoint;
    int max_search_tree_height, max_search_options;
    double search_radius, max_angle_diff;
    float max_next_heuristic, heur_dist_ponderation;
    float max_treeSearch_time;
    struct Way {
      int min_loop_size;
      double max_dist_loop_closure;
      double max_angle_diff_loop_closure;
    } way;
  } wayComputer;
  struct Visualization {
    bool publish_markers;
    std::string triangulation_topic;
    std::string midpoints_topic;
    std::string way_topic;
  } visualization;
};