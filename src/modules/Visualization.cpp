#include "modules/Visualization.hpp"

Visualization::Visualization(ros::NodeHandle *const nh, const std::string &triangulation_topic) : nh(nh) {
  trianglesPub = nh->advertise<visualization_msgs::MarkerArray>(triangulation_topic, 1);
}

void Visualization::triangulation(const TriangleSet &triSet) const {
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker m;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "global";
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.0;
  m.type = visualization_msgs::Marker::CYLINDER;
  m.id = 0;
  m.action = visualization_msgs::Marker::DELETEALL;
  ma.markers.reserve(2*triSet.size()+1);
  ma.markers.push_back(m);
  m.action = visualization_msgs::Marker::ADD;
  for (const Triangle &t : triSet) {
    // Triangle itself
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.points.clear();
    m.points.reserve(triSet.size());
    m.id++;
    m.points.push_back(t.n0.gmPoint());
    m.points.push_back(t.n1.gmPoint());
    m.points.push_back(t.n2.gmPoint());
    m.points.push_back(t.n0.gmPoint());
    m.type = visualization_msgs::Marker::LINE_STRIP;
    ma.markers.push_back(m);

    // Circumcenter
    m.type = visualization_msgs::Marker::CYLINDER;
    m.pose.position = t.circumCenter();
    m.id++;
    ma.markers.push_back(m);
  }
  trianglesPub.publish(ma);
}