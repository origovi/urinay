/**
 * @file Visualization.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Visualization class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/Visualization.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ------------------------------ Public Methods ---------------------------- */

Visualization &Visualization::getInstance() {
  static Visualization vis;
  return vis;
}

void Visualization::init(ros::NodeHandle *const nh, const Params::Visualization &params) {
  params_ = params;
  if (params.publish_markers) {
    trianglesPub = nh->advertise<visualization_msgs::MarkerArray>(params_.triangulation_topic, 1);
    wayPub = nh->advertise<visualization_msgs::MarkerArray>(params_.way_topic, 1);
    traceBufferPub = nh->advertise<visualization_msgs::MarkerArray>(params_.treeSearch_topic, 1);
  }
}

void Visualization::setHeader(const std_msgs::Header &header) {
  this->lastHeader_ = header;
}

void Visualization::visualize(const EdgeSet &edgeSet) const {
  if (not this->params_.publish_markers) return;
  if (trianglesPub.getNumSubscribers() <= 0) return;

  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(1 + 1 + 1);  // delete, lines & midpoints
  visualization_msgs::Marker mLines, mMidpoint;
  size_t id = 0;
  mLines.header = this->lastHeader_;
  mLines.color.a = 1.0;
  mLines.color.r = 1.0;
  mLines.pose.orientation.w = 1.0;
  mLines.scale.x = 0.1;
  mLines.id = id++;
  mLines.action = visualization_msgs::Marker::DELETEALL;
  mLines.type = visualization_msgs::Marker::LINE_LIST;
  ma.markers.push_back(mLines);
  mLines.action = visualization_msgs::Marker::ADD;

  mMidpoint = mLines;
  mMidpoint.type = visualization_msgs::Marker::POINTS;
  mMidpoint.scale.x = 0.1;
  mMidpoint.scale.y = 0.1;
  mMidpoint.color.r = 0.0;
  mMidpoint.color.g = 1.0;
  mMidpoint.color.b = 0.0;

  mLines.points.reserve(2 * edgeSet.size());
  mLines.id = id++;

  mMidpoint.points.reserve(edgeSet.size());
  mMidpoint.id = id++;
  for (const Edge &e : edgeSet) {
    // Edge (per se)
    mLines.points.push_back(e.n0.pointGlobal().gmPoint());
    mLines.points.push_back(e.n1.pointGlobal().gmPoint());
    
    // Midpoint
    mMidpoint.points.push_back(e.midPointGlobal().gmPoint());
    mMidpoint.points.back().z += 0.05;  // To make it visible and avoid overlap with triangulation in vis
  }
  ma.markers.push_back(mLines);
  ma.markers.push_back(mMidpoint);
  trianglesPub.publish(ma);
}

void Visualization::visualize(const Trace &way) const {
  if (not this->params_.publish_markers) return;
  if (wayPub.getNumSubscribers() <= 0) return;

  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(3 * way.size() + 1);
  visualization_msgs::Marker mMidpoints, mLeft, mRight;
  size_t id = 0;
  mMidpoints.header = this->lastHeader_;
  mMidpoints.color.a = 1.0;
  mMidpoints.color.g = 1.0;
  mMidpoints.pose.orientation.w = 1.0;
  mMidpoints.scale.x = 0.15;
  mMidpoints.scale.y = 0.15;
  mMidpoints.scale.z = 0.15;
  mMidpoints.type = visualization_msgs::Marker::LINE_STRIP;
  mMidpoints.id = id++;
  mMidpoints.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(mMidpoints);
  mMidpoints.action = visualization_msgs::Marker::ADD;
  mLeft = mMidpoints;
  mLeft.color.g = 0.0;
  mLeft.color.b = 0.7;
  mLeft.id = id++;
  mRight = mMidpoints;
  mRight.color.r = 0.7;
  mRight.color.g = 0.7;

  mMidpoints.color.a = 0.5;
  mMidpoints.id = id++;
  for (const Point &p : way.getPath()) {
    mMidpoints.points.push_back(p.gmPoint());
    mMidpoints.points.back().z += 0.05;  // To make it visible and avoid overlap with triangulation in vis
  }
  ma.markers.push_back(mMidpoints);

  mRight.id = id++;
  Tracklimits tracklimits = way.getTracklimits();

  for (const Node &n : tracklimits.first) {
    mLeft.points.push_back(n.pointGlobal().gmPoint());
    mLeft.points.back().z += 0.05;  // To make it visible and avoid overlap with triangulation in vis
  }
  ma.markers.push_back(mLeft);

  for (const Node &n : tracklimits.second) {
    mRight.points.push_back(n.pointGlobal().gmPoint());
    mRight.points.back().z += 0.05;  // To make it visible and avoid overlap with triangulation in vis
  }
  ma.markers.push_back(mRight);

  wayPub.publish(ma);
}

void Visualization::visualize(const TraceBuffer &traceBuffer) const {
  if (!this->params_.publish_markers or !this->params_.visualize_treeSearch) return;
  if (traceBufferPub.getNumSubscribers() <= 0) return;

  ROS_WARN_STREAM("Visualizing " << traceBuffer.size() << " traces...");

  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(traceBuffer.size() + 1);
  visualization_msgs::Marker mTrace;
  size_t id = 0;
  mTrace.header = this->lastHeader_;
  mTrace.color.a = 1.0;
  mTrace.color.g = 1.0;
  mTrace.pose.orientation.w = 1.0;
  mTrace.color.a = 1.0;
  mTrace.scale.x = 0.15;
  mTrace.scale.y = 0.15;
  mTrace.scale.z = 0.15;
  mTrace.type = visualization_msgs::Marker::LINE_STRIP;
  mTrace.id = id++;
  mTrace.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(mTrace);
  mTrace.action = visualization_msgs::Marker::ADD;

  for (const TraceWithBuffer &twb : traceBuffer) {
    mTrace.points.clear();
    mTrace.id = id++;
    mTrace.color.r = rand() / double(RAND_MAX);
    mTrace.color.g = rand() / double(RAND_MAX);
    mTrace.color.b = rand() / double(RAND_MAX);

    Trace t = twb.trace;
    while (!t.empty()) {
      mTrace.points.push_back(t.edge().midPointGlobal().gmPoint());
      t = t.before();
    }
    ma.markers.push_back(mTrace);
  }

  traceBufferPub.publish(ma);
  ros::WallDuration(0.2).sleep();
}