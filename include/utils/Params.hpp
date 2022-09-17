#pragma once

#include <ros/ros.h>

class Params {
 public:
  Params(ros::NodeHandle *const nh);
  struct Main {
    std::string input_topic, output_full_topic, output_partial_topic;
    std::string markers_full_topic, markers_partial_topic;
  } main;
  struct Way {
    double max_triangle_edge_len, max_dist_circum_midPoint;
  } way;
  struct Visualization {
    bool publish_markers;
    std::string triangulation_topic;
    std::string midpoints_topic;
  } visualization;
};