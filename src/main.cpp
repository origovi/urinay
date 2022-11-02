/**
 * @file main.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Main file of Urinay, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include <as_msgs/ConeArray.h>
#include <as_msgs/PathLimits.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/stat.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"

// Publishers are initialized here
ros::Publisher partialPub;
ros::Publisher loopPub;

WayComputer *wayComputer;
Params *params;

// This is the map callback
void callback_ccat(const as_msgs::ConeArray::ConstPtr &data) {
  if (data->cones.empty()) {
    ROS_WARN("[urinay] reading empty set of cones.");
    return;
  }

  Time::tick("computation");  // Start measuring time

  // Convert to Node vector
  std::vector<Node> nodes;
  nodes.reserve(data->cones.size());
  for (const as_msgs::Cone &c : data->cones) {
    nodes.emplace_back(c);
  }

  // Delaunay triangulation
  TriangleSet triangles = DelaunayTri::compute(nodes);

  // Update the way with the new triangulation
  wayComputer->update(triangles);

  // Publish loop and write tracklimits to a file
  if (wayComputer->isLoopClosed()) {
    loopPub.publish(wayComputer->getPathLimits());
    ROS_INFO("[urinay] Tanco loop");
    std::string loopDir = params->main.package_path + "/loops";
    mkdir(loopDir.c_str(), 0777);
    wayComputer->writeWayToFile(loopDir + "/loop.unay");
    if (params->main.shutdown_on_loop_closure) {
      Time::tock("computation");  // End measuring time
      ROS_INFO("[urinay] Tingui bon dia :)");
      ros::shutdown();
    }
  }
  // Publish partial
  else {
    partialPub.publish(wayComputer->getPathLimits());
  }

  Time::tock("computation");  // End measuring time

  std::cout << std::endl;
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "urinay");

  ros::NodeHandle *const nh = new ros::NodeHandle;

  params = new Params(nh);
  wayComputer = new WayComputer(params->wayComputer);
  Visualization::getInstance().init(nh, params->visualization);

  // Subscribers & Publishers
  ros::Subscriber subMap = nh->subscribe(params->main.input_topic, 1, callback_ccat);
  ros::Subscriber subPose = nh->subscribe(params->main.input_pose_topic, 1, &WayComputer::stateCallback, wayComputer);

  partialPub = nh->advertise<as_msgs::PathLimits>(params->main.output_partial_topic, 1);
  loopPub = nh->advertise<as_msgs::PathLimits>(params->main.output_full_topic, 1);

  ros::spin();
}