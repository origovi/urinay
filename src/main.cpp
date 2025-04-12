/**
 * @file main.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Main file of Urinay, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include <custom_msgs/ConeWithIdArray.h>
#include <custom_msgs/ConeWithId.h>
#include <custom_msgs/PathLimits.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/stat.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Logger.hpp"

// Publishers are initialized here
ros::Publisher pubPartial;
ros::Publisher pubFull;

WayComputer *wayComputer;
Params *params;

// This is the map callback
void mapCallback(const custom_msgs::ConeWithIdArray::ConstPtr &data) {
  if (not wayComputer->isLocalTfValid()) {
    Logger::logwarn("CarState not being received.");
    if (params->main.verbose)
      Logger::print_report();
    return;
  }
  if (data->cones.empty()) {
    Logger::logwarn("Reading empty set of cones.");
    if (params->main.verbose)
      Logger::print_report();
    return;
  }

  Logger::tick("total");

  // Convert to Node vector
  std::vector<Node> nodes;
  nodes.reserve(data->cones.size());
  for (const custom_msgs::ConeWithId &c : data->cones) {
    nodes.emplace_back(c);
  }

  // Update local coordinates of Nodes (makes original local coords unnecessary)
  for (const Node &n : nodes) {
    n.updateLocal(wayComputer->getLocalTf());
  }

  // Delaunay triangulation
  TriangleSet triangles = DelaunayTri::compute(nodes);

  // Update the way with the new triangulation
  wayComputer->update(triangles, data->header);

  // Publish loop and write tracklimits to a file
  if (wayComputer->isLoopClosed()) {
    pubFull.publish(wayComputer->getPathLimits());
    Logger::loginfo("Loop closed!");
    std::string loopDir = params->main.package_path + "/loops";
    mkdir(loopDir.c_str(), 0777);
    wayComputer->writeWayToFile(loopDir + "/loop.unay");
    if (params->main.shutdown_on_loop_closure) {
      Logger::tock("total");
      Logger::loginfo("Have a good day :)");
      if (params->main.verbose)
        Logger::print_report();
      ros::shutdown();
    }
  }
  // Publish partial
  else {
    pubPartial.publish(wayComputer->getPathLimits());
  }

  Logger::tock("total");
  if (params->main.verbose)
    Logger::print_report();
}

// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "urinay");

  ros::NodeHandle *const nh = new ros::NodeHandle;

  params = new Params(nh);
  wayComputer = new WayComputer(params->wayComputer);
  Logger::print_immediately = false;
  Visualization::getInstance().init(nh, params->visualization);

  // Subscribers & Publishers
  ros::Subscriber subCones = nh->subscribe(params->main.input_cones_topic, 1, mapCallback);
  ros::Subscriber subPose = nh->subscribe(params->main.input_pose_topic, 1, &WayComputer::stateCallback, wayComputer);

  pubPartial = nh->advertise<custom_msgs::PathLimits>(params->main.output_partial_topic, 1);
  pubFull = nh->advertise<custom_msgs::PathLimits>(params->main.output_full_topic, 1, true);  // Latch message

  ros::spin();
}