/**
 * @file Point.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Point class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Point.hpp"

/**
 * CONSTRUCTORS
 */
Point::Point() : x(0.0), y(0.0) {}
Point::Point(const double &x, const double &y) : x(x), y(y) {}

template <typename T>
Point::Point(const T &point) : x(point.x), y(point.y) {}
template Point::Point<geometry_msgs::Point>(const geometry_msgs::Point &);

/**
 * PRIVATE METHODS
 */

/**
 * PUBLIC METHODS
 */
Point Point::operator+(const Point &p) const { return Point(this->x + p.x, this->y + p.y); }

Point Point::operator-(const Point &p) const { return Point(this->x - p.x, this->y - p.y); }

template <typename T>
Point Point::operator*(const T &num) const {
  return Point(this->x * num, this->y * num);
}
template Point Point::operator*<int>(const int &) const;
template Point Point::operator*<float>(const float &) const;
template Point Point::operator*<double>(const double &) const;
template Point Point::operator*<size_t>(const size_t &) const;

template <typename T>
Point Point::operator/(const T &num) const {
  return Point(this->x / num, this->y / num);
}
template Point Point::operator/<int>(const int &) const;
template Point Point::operator/<float>(const float &) const;
template Point Point::operator/<double>(const double &) const;
template Point Point::operator/<size_t>(const size_t &) const;

Point &Point::operator+=(const Point &p) {
  this->x += p.x;
  this->y += p.y;
  return *this;
}

Point &Point::operator-=(const Point &p) {
  this->x -= p.x;
  this->y -= p.y;
  return *this;
}

template <typename T>
Point &Point::operator*=(const T &num) {
  this->x *= num;
  this->y *= num;
  return *this;
}
template Point &Point::operator*=<int>(const int &);
template Point &Point::operator*=<float>(const float &);
template Point &Point::operator*=<double>(const double &);
template Point &Point::operator*=<size_t>(const size_t &);

template <typename T>
Point &Point::operator/=(const T &num) {
  x /= num;
  y /= num;
  return *this;
}
template Point &Point::operator/=<int>(const int &);
template Point &Point::operator/=<float>(const float &);
template Point &Point::operator/=<double>(const double &);
template Point &Point::operator/=<size_t>(const size_t &);

std::ostream &operator<<(std::ostream &os, const Point &p) {
  return os << "P(" << p.x << ", " << p.y << ")\n";
}

Point Point::transformed(const Eigen::Affine3d &tf) const {
  Eigen::Vector3d product = tf * Eigen::Vector3d(this->x, this->y, 0.0);
  return Point(product.x(), product.y());
}

geometry_msgs::Point Point::gmPoint() const {
  geometry_msgs::Point res;
  res.x = this->x;
  res.y = this->y;
  res.z = 0.0;
  return res;
}

const double &Point::at(const size_t &ind) const {
  switch (ind) {
    case 0:
      return this->x;
    default:
      return this->y;
  }
}

size_t Point::size() const { return 2; }