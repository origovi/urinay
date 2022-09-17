#include "modules/Visualization.hpp"

Visualization::Visualization(ros::NodeHandle *const nh, const Params::Visualization &params) : nh(nh), params_(params) {
  trianglesPub = nh->advertise<visualization_msgs::MarkerArray>(params_.triangulation_topic, 1);
  midpointsPub = nh->advertise<visualization_msgs::MarkerArray>(params_.midpoints_topic, 1);
}

void Visualization::visualize(const TriangleSet &triSet) const {
  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(2 * triSet.size() + 3 * triSet.size() + 1);
  visualization_msgs::Marker mTriangulation, mCircumCenter, mMidpoint;
  size_t id = 0;
  mTriangulation.header.stamp = ros::Time::now();
  mTriangulation.header.frame_id = "global";
  mTriangulation.color.a = 1.0;
  mTriangulation.color.r = 1.0;
  mTriangulation.pose.orientation.w = 1.0;
  mTriangulation.scale.x = 0.1;
  mTriangulation.scale.y = 0.1;
  mTriangulation.scale.z = 0.0;
  mTriangulation.type = visualization_msgs::Marker::CYLINDER;
  mTriangulation.id = id++;
  mTriangulation.action = visualization_msgs::Marker::DELETEALL;
  mTriangulation.type = visualization_msgs::Marker::LINE_STRIP;
  ma.markers.push_back(mTriangulation);
  mTriangulation.action = visualization_msgs::Marker::ADD;

  mCircumCenter = mTriangulation;
  mCircumCenter.scale.x = 0.2;
  mCircumCenter.scale.y = 0.2;
  mCircumCenter.scale.z = 0.1;
  mCircumCenter.color.r = 0.0;
  mCircumCenter.color.g = 0.0;
  mCircumCenter.color.b = 1.0;
  mCircumCenter.type = visualization_msgs::Marker::CYLINDER;

  mMidpoint = mCircumCenter;
  mMidpoint.type = visualization_msgs::Marker::CUBE;
  mMidpoint.color.r = 0.0;
  mMidpoint.color.g = 1.0;
  mMidpoint.color.b = 0.0;
  for (const Triangle &t : triSet) {
    // Triangle itself
    mTriangulation.pose.position.x = 0.0;
    mTriangulation.pose.position.y = 0.0;
    mTriangulation.pose.position.z = 0.0;
    mTriangulation.points.clear();
    mTriangulation.points.reserve(triSet.size());
    mTriangulation.id = id++;
    mTriangulation.points.push_back(t.nodes[0].gmPoint());
    mTriangulation.points.push_back(t.nodes[1].gmPoint());
    mTriangulation.points.push_back(t.nodes[2].gmPoint());
    mTriangulation.points.push_back(t.nodes[0].gmPoint());
    ma.markers.push_back(mTriangulation);

    // Circumcenter
    mCircumCenter.pose.position = t.circumCenter().gmPoint();
    mCircumCenter.id = id++;
    ma.markers.push_back(mCircumCenter);

    // Edges midpoints
    for (const Edge &e : t.edges) {
      mMidpoint.pose.position = e.midPoint().gmPoint();
      mMidpoint.id = id++;
      ma.markers.push_back(mMidpoint);
    }
  }
  trianglesPub.publish(ma);
}

void Visualization::visualize(const EdgeSet &edgeSet) const {
  visualization_msgs::MarkerArray ma;
  ma.markers.reserve(edgeSet.size() + 1);
  visualization_msgs::Marker mMidpoint;
  size_t id = 0;
  mMidpoint.header.stamp = ros::Time::now();
  mMidpoint.header.frame_id = "global";
  mMidpoint.color.a = 1.0;
  mMidpoint.color.r = 1.0;
  mMidpoint.pose.orientation.w = 1.0;
  mMidpoint.scale.x = 0.08;
  mMidpoint.scale.y = 0.08;
  mMidpoint.scale.z = 0.2;
  mMidpoint.type = visualization_msgs::Marker::CYLINDER;
  mMidpoint.id = id++;
  mMidpoint.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.push_back(mMidpoint);
  mMidpoint.action = visualization_msgs::Marker::ADD;

  for (const Edge &e : edgeSet) {
    mMidpoint.pose.position = e.midPoint().gmPoint();
    mMidpoint.id = id++;
    ma.markers.push_back(mMidpoint);
  }
  midpointsPub.publish(ma);
}