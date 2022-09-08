#include "structures/Node.hpp"

const uint32_t Node::SUPERTRIANGLE_BASEID;
uint32_t Node::superTriangleNodeNum = 0;

/* ----------------------------- Private Methods ---------------------------- */

Node::Node(const double &x, const double &y) : x(x), y(y), id(SUPERTRIANGLE_BASEID + superTriangleNodeNum), belongsToSuperTriangle_(true) {
  superTriangleNodeNum++;
  superTriangleNodeNum %= 3;
}

/* ----------------------------- Public Methods ----------------------------- */

Node::Node(const double &x, const double &y, const uint32_t &id) : x(x), y(y), id(id), belongsToSuperTriangle_(false) {}

bool Node::operator==(const Node &n) const {
  return n.id == this->id and n.x == this->x and n.y == this->y;
}

Node Node::superTriangleNode(const double &x, const double &y) {
  return Node(x, y);
}

const bool &Node::belongsToSuperTriangle() const {
  return belongsToSuperTriangle_;
}

geometry_msgs::Point Node::gmPoint() const {
  geometry_msgs::Point res;
  res.x = this->x;
  res.y = this->y;
  res.z = 0.0;
  return res;
}

std::ostream &operator<<(std::ostream &os, const Node &n) {
  os << "N(" << n.x << ", " << n.y << ")";
  return os;
}