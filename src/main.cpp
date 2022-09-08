#include <as_msgs/ConeArray.h>
#include <as_msgs/Tracklimits.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/TLs.hpp"
#include "modules/Visualization.hpp"
#include "utils/Time.hpp"

// Publishers are initialized here
ros::Publisher tlPub;
ros::Publisher lapPub;
ros::Publisher fullMarkersPub, partialMarkersPub;
bool publish_markers;
Visualization *vis;

// This is the map callback
void callback_ccat(const as_msgs::ConeArray::ConstPtr &data) {
  if (not data->cones.empty()) {
    Time::tick("DelaunayTri");

    // Convert to Node vector
    std::vector<Node> nodes;
    nodes.reserve(data->cones.size());
    for (const as_msgs::Cone &c : data->cones) {
      nodes.emplace_back(c.position_global.x, c.position_global.y, c.id);
    }

    // Delaunay triangulation
    TriangleSet triangles = DelaunayTri::compute(nodes);

    Time::tock("DelaunayTri");
    
    if (publish_markers) {
      vis->triangulation(triangles);
    }

  }
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "urinay");

  ros::NodeHandle *const nh = new ros::NodeHandle;

  // nh.param<bool>("urinay/debug", urinay.debug, false);

  std::string input_topic, output_full_topic, output_partial_topic;
  nh->param<std::string>("urinay/input_topic", input_topic, "/AS/P/ccat/cones");
  nh->param<std::string>("urinay/output_full_topic", output_full_topic, "/AS/P/tracklimits/full");
  nh->param<std::string>("urinay/output_partial_topic", output_partial_topic, "/AS/P/tracklimits/partial");

  nh->param<bool>("urinay/publish_markers", publish_markers, false);

  std::string triangulation_topic, markers_full_topic, markers_partial_topic;
  nh->param<std::string>("urinay/triangulation_topic", triangulation_topic, "/AS/P/urinay/markers/triangulation");
  nh->param<std::string>("urinay/markers_full_topic", markers_full_topic, "/AS/P/urinay/markers/full");
  nh->param<std::string>("urinay/markers_partial_topic", markers_partial_topic, "/AS/P/urinay/markers/partial");

  // Publishers & Subscriber
  ros::Subscriber subMap = nh->subscribe(input_topic, 1, callback_ccat);

  tlPub = nh->advertise<as_msgs::Tracklimits>(output_partial_topic, 1);
  lapPub = nh->advertise<as_msgs::Tracklimits>(output_full_topic, 1);

  if (publish_markers) {
    vis = new Visualization(nh, triangulation_topic);
    fullMarkersPub = nh->advertise<visualization_msgs::MarkerArray>(markers_full_topic, 1);
    partialMarkersPub = nh->advertise<visualization_msgs::MarkerArray>(markers_partial_topic, 1);
  }

  ros::spin();
}