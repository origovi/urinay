/**
 * @file Node.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Node class specification
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <as_msgs/Cone.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "structures/Point.hpp"
#include "structures/Vector.hpp"
#include "utils/constants.hpp"

/**
 * @brief Represents a node, i.e. the useful part of an as_msgs::Cone
 * (positions and id).
 */
class Node {
 private:
  /**
   * @brief Supertriangle's Node(s) must have an id, this id must not interfere
   * with existing Node's id, to solve this issue a huge id will be assigned to
   * all Node(s) that are part of the supertriangle.
   * See utils/constants.hpp/HASH_SHIFT_NUM to know why this value.
   */
  static const uint32_t SUPERTRIANGLE_BASEID = (1 << HASH_SHIFT_NUM) - 3;

  /**
   * @brief Current supertriangle node number.
   */
  static uint32_t superTriangleNodeNum;

  /**
   * @brief Whether or not this Node belong to a supertriangle.
   */
  const bool belongsToSuperTriangle_;

  /**
   * @brief Cone point in local coordinates.
   * **Note** that it is mutable.
   */
  mutable Point point_;

  /**
   * @brief Cone point in global coordinates.
   */
  const Point pointGlobal_;

  /**
   * @brief Construct a new Node object that will be a supertriangle.
   *
   * @param[in] x
   * @param[in] y
   */
  Node(const double &x, const double &y);

 public:
  /**
   * @brief Node's id, in principle given by the cone tracker.
   */
  const uint32_t id;

  /**
   * @brief Construct a new Node object.
   *
   * @param[in] x
   * @param[in] y
   * @param[in] xGlobal
   * @param[in] yGlobal
   * @param[in] id
   */
  Node(const double &x, const double &y, const double &xGlobal, const double &yGlobal, const uint32_t &id);

  /**
   * @brief Construct a new Node object from an as_msgs::Cone.
   *
   * @param[in] c
   */
  Node(const as_msgs::Cone &c);

  /**
   * @brief Returns the Node x local coordinate.
   */
  const double &x() const;

  /**
   * @brief Returns the Node y local coordinate.
   */
  const double &y() const;

  /**
   * @brief Comparison operator. Two nodes will be equal iff their id is same.
   *
   * @param[in] n
   */
  bool operator==(const Node &n) const;

  /**
   * @brief Negation of the comparison operator.
   *
   * @param[in] n
   */
  bool operator!=(const Node &n) const;

  /**
   * @brief Returns a supertriangle Node with local coordinates \a (x, y).
   *
   * @param[in] x
   * @param[in] y
   */
  static Node superTriangleNode(const double &x, const double &y);

  /**
   * @brief Checks if the Node belongs to a supertriangle.
   */
  const bool &belongsToSuperTriangle() const;

  /**
   * @brief Updates the local coordinates of the Node.
   *
   * @param[in] tf
   */
  void updateLocal(const Eigen::Affine3d &tf) const;

  /**
   * @brief Returns the point in local coordinates.
   */
  const Point &point() const;

  /**
   * @brief Returns the point in global coordinates.
   */
  const Point &pointGlobal() const;

  /**
   * @brief Returns the distance squared from the Node's local point to \a p.
   *
   * @param[in] p
   */
  double distSq(const Point &p) const;

  /**
   * @brief Returns the angle that the Node makes with the Node(s) \a n0
   * and \a n1.
   *
   * @param[in] n0
   * @param[in] n1
   */
  double angleWith(const Node &n0, const Node &n1) const;

  /**
   * @brief Converts (and returns) the Node as an as_msgs::Cone.
   */
  as_msgs::Cone cone() const;

  /**
   * @brief Cout operator.
   *
   * @param[in,out] os
   * @param[in] n
   */
  friend std::ostream &operator<<(std::ostream &os, const Node &n);
};