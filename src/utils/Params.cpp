#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(ros::NodeHandle *const nh) {
  std::string ns = ros::this_node::getName();
  // Main
  nh->param<std::string>(ns + "/input_topic", main.input_topic, "/AS/P/ccat/cones");
  nh->param<std::string>(ns + "/output_full_topic", main.output_full_topic, "/AS/P/tracklimits/full");
  nh->param<std::string>(ns + "/output_partial_topic", main.output_partial_topic, "/AS/P/tracklimits/partial");

  // Visualization
  nh->param<bool>(ns + "/publish_markers", visualization.publish_markers, false);
  nh->param<std::string>(ns + "/marker_topics/triangulation", visualization.triangulation_topic, "/AS/P/urinay/markers/triangulation");
  nh->param<std::string>(ns + "/marker_topics/midpoints", visualization.midpoints_topic, "/AS/P/urinay/markers/midpoints");
  // nh->param<std::string>(ns + "/marker_topics/markers_full_topic", visualization.markers_full_topic, "/AS/P/urinay/markers/full");
  // nh->param<std::string>(ns + "/marker_topics/markers_partial_topic", visualization.markers_partial_topic, "/AS/P/urinay/markers/partial");

  // Way
  nh->param<double>(ns + "/max_triangle_edge_len", way.max_triangle_edge_len, 9.0);
  nh->param<double>(ns + "/max_dist_circum_midPoint", way.max_dist_circum_midPoint, 1.0);
}