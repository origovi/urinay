#include "structures/Node.hpp"

const uint32_t Node::SUPERTRIANGLE_BASEID;
uint32_t Node::superTriangleNodeNum = 0;

/* ----------------------------- Private Methods ---------------------------- */

Node::Node(const double &x, const double &y)
    : point_(x, y), id(SUPERTRIANGLE_BASEID + superTriangleNodeNum), belongsToSuperTriangle_(true) {
  superTriangleNodeNum++;
  superTriangleNodeNum %= 3;
}

/* ----------------------------- Public Methods ----------------------------- */

Node::Node(const double &x, const double &y, const double &xGlobal, const double &yGlobal, const uint32_t &id)
    : point_(x, y), pointGlobal_(xGlobal, yGlobal), id(id), belongsToSuperTriangle_(false) {}

Node::Node(const as_msgs::Cone &c)
    : Node(c.position_baseLink.x, c.position_baseLink.y, c.position_global.x, c.position_global.y, c.id) {}

const double &Node::x() const {
  return this->point_.x;
}

const double &Node::y() const {
  return this->point_.y;
}

bool Node::operator==(const Node &n) const {
  return n.id == this->id;
}

bool Node::operator!=(const Node &n) const {
  return not(*this == n);
}

Node Node::superTriangleNode(const double &x, const double &y) {
  return Node(x, y);
}

const bool &Node::belongsToSuperTriangle() const {
  return belongsToSuperTriangle_;
}

void Node::updateLocal(const Eigen::Affine3d &tf) {
  this->point_ = this->pointGlobal().transformed(tf);
}

const Point &Node::point() const {
  return this->point_;
}

const Point &Node::pointGlobal() const {
  return this->pointGlobal_;
}

double Node::distSq(const Point &p) const {
  return (this->x() - p.x) * (this->x() - p.x) + (this->y() - p.y) * (this->y() - p.y);
}

as_msgs::Cone Node::cone() const {
  as_msgs::Cone res;
  res.id = this->id;
  res.position_global = this->pointGlobal().gmPoint();
  res.type = 4;  // None
  return res;
}

std::ostream &operator<<(std::ostream &os, const Node &n) {
  os << "N(" << n.x() << ", " << n.y() << ")";
  return os;
}