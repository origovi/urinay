#include <as_msgs/ConeArray.h>
#include <as_msgs/Tracklimits.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Way.hpp"
#include "modules/Visualization.hpp"
#include "utils/Time.hpp"

// Publishers are initialized here
ros::Publisher tlPub;
ros::Publisher lapPub;
Visualization *vis;

Way *way;
Params *params;

// This is the map callback
void callback_ccat(const as_msgs::ConeArray::ConstPtr &data) {
  if (not data->cones.empty()) {
    // Convert to Node vector
    std::vector<Node> nodes;
    nodes.reserve(data->cones.size());
    for (const as_msgs::Cone &c : data->cones) {
      nodes.emplace_back(c);
    }

    // Delaunay triangulation
    Time::tick("Triangulation");
    TriangleSet triangles = DelaunayTri::compute(nodes);
    Time::tock("Triangulation");


    // Update the way with the new triangulation
    Time::tick("Way update");
    way->update(triangles, *vis);
    Time::tock("Way update");

    if (params->visualization.publish_markers) {
      vis->visualize(triangles);
    }

    std::cout << std::endl;
  }
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "urinay");

  ros::NodeHandle *const nh = new ros::NodeHandle;

  params = new Params(nh);
  way = new Way(params->way);
  vis = new Visualization(nh, params->visualization);

  // Publishers & Subscriber
  ros::Subscriber subMap = nh->subscribe(params->main.input_topic, 1, callback_ccat);

  tlPub = nh->advertise<as_msgs::Tracklimits>(params->main.output_partial_topic, 1);
  lapPub = nh->advertise<as_msgs::Tracklimits>(params->main.output_full_topic, 1);

  ros::spin();
}