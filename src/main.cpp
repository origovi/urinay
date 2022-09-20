#include <as_msgs/ConeArray.h>
#include <as_msgs/PathLimits.h>
#include <ros/ros.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"

// Publishers are initialized here
ros::Publisher partialPub;
ros::Publisher loopPub;

WayComputer *wayComputer;
Visualization *vis;
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
    wayComputer->update(triangles, *vis);
    Time::tock("Way update");

    if (params->visualization.publish_markers) {
      vis->visualize(triangles);
    }

    // Publish loop
    if (wayComputer->way().closesLoop()) {
      loopPub.publish(wayComputer->way().getPathLimits());
    }
    // Publish partial
    else {
      partialPub.publish(wayComputer->way().getPathLimits());
    }

    std::cout << std::endl;
  }
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "urinay");

  ros::NodeHandle *const nh = new ros::NodeHandle;

  params = new Params(nh);
  wayComputer = new WayComputer(params->wayComputer);
  vis = new Visualization(nh, params->visualization);

  // Publishers & Subscriber
  ros::Subscriber subMap = nh->subscribe(params->main.input_topic, 1, callback_ccat);

  partialPub = nh->advertise<as_msgs::PathLimits>(params->main.output_partial_topic, 1);
  loopPub = nh->advertise<as_msgs::PathLimits>(params->main.output_full_topic, 1);

  ros::spin();
}