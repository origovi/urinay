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
    midpointsPub = nh->advertise<visualization_msgs::MarkerArray>(params_.midpoints_topic, 1);
    wayPub = nh->advertise<visualization_msgs::MarkerArray>(params_.way_topic, 1);
  }
}

void Visualization::setHeader(const std_msgs::Header &header) {
  this->lastHeader_ = header;
}

void Visualization::visualize(const TriangleSet &triSet) const {
  if (not this->params_.publish_markers) return;
  if (trianglesPub.getNumSubscribers() <= 0) return;

  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(1 + 5 * triSet.size());
  visualization_msgs::Marker mTriangulation, mCircumCenter, mMidpoint;
  size_t id = 0;
  mTriangulation.header = this->lastHeader_;
  mTriangulation.color.a = 1.0;
  mTriangulation.color.r = 1.0;
  mTriangulation.pose.orientation.w = 1.0;
  mTriangulation.scale.x = 0.1;
  mTriangulation.scale.y = 0.1;
  mTriangulation.scale.z = 0.01;
  mTriangulation.id = id++;
  mTriangulation.action = visualization_msgs::Marker::DELETEALL;
  mTriangulation.type = visualization_msgs::Marker::LINE_STRIP;
  ma.markers.push_back(mTriangulation);
  mTriangulation.action = visualization_msgs::Marker::ADD;

  mCircumCenter = mTriangulation;
  mCircumCenter.type = visualization_msgs::Marker::CYLINDER;
  mCircumCenter.scale.x = 0.1;
  mCircumCenter.scale.y = 0.1;
  mCircumCenter.scale.z = 0.05;
  mCircumCenter.color.r = 0.0;
  mCircumCenter.color.g = 0.0;
  mCircumCenter.color.b = 1.0;

  mMidpoint = mCircumCenter;
  mMidpoint.type = visualization_msgs::Marker::CUBE;
  mMidpoint.color.r = 0.0;
  mMidpoint.color.g = 1.0;
  mMidpoint.color.b = 0.0;
  for (const Triangle &t : triSet) {
    // Triangle itself
    mTriangulation.points.clear();
    mTriangulation.points.reserve(4);
    mTriangulation.id = id++;
    mTriangulation.points.push_back(t.nodes[0].pointGlobal().gmPoint());
    mTriangulation.points.push_back(t.nodes[1].pointGlobal().gmPoint());
    mTriangulation.points.push_back(t.nodes[2].pointGlobal().gmPoint());
    mTriangulation.points.push_back(t.nodes[0].pointGlobal().gmPoint());
    ma.markers.push_back(mTriangulation);

    // Circumcenter
    mCircumCenter.pose.position = t.circumCenterGlobal().gmPoint();
    mCircumCenter.id = id++;
    ma.markers.push_back(mCircumCenter);

    // Edges midpoints
    for (const Edge &e : t.edges) {
      mMidpoint.pose.position = e.midPointGlobal().gmPoint();
      mMidpoint.id = id++;
      ma.markers.push_back(mMidpoint);
    }
  }
  trianglesPub.publish(ma);
}

void Visualization::visualize(const EdgeSet &edgeSet) const {
  if (not this->params_.publish_markers) return;
  if (midpointsPub.getNumSubscribers() <= 0) return;

  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(edgeSet.size() + 1);
  visualization_msgs::Marker mMidpoint;
  size_t id = 0;
  mMidpoint.header = this->lastHeader_;
  mMidpoint.color.a = 1.0;
  mMidpoint.color.r = 1.0;
  mMidpoint.pose.orientation.w = 1.0;
  mMidpoint.scale.x = 0.1;
  mMidpoint.scale.y = 0.1;
  mMidpoint.scale.z = 0.1;
  mMidpoint.type = visualization_msgs::Marker::CYLINDER;
  mMidpoint.id = id++;
  mMidpoint.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(mMidpoint);
  mMidpoint.action = visualization_msgs::Marker::ADD;

  for (const Edge &e : edgeSet) {
    mMidpoint.pose.position = e.midPointGlobal().gmPoint();
    mMidpoint.id = id++;
    ma.markers.push_back(mMidpoint);
  }
  midpointsPub.publish(ma);
}

void Visualization::visualize(const Way &way) const {
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
  }
  ma.markers.push_back(mMidpoints);

  mRight.id = id++;
  Tracklimits tracklimits = way.getTracklimits();

  for (const Node &n : tracklimits.first) {
    mLeft.points.push_back(n.pointGlobal().gmPoint());
  }
  ma.markers.push_back(mLeft);

  for (const Node &n : tracklimits.second) {
    mRight.points.push_back(n.pointGlobal().gmPoint());
  }
  ma.markers.push_back(mRight);

  wayPub.publish(ma);
}