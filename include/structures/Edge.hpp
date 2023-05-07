/**
 * @file Edge.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Edge class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include "structures/Node.hpp"
#include "utils/constants.hpp"

/**
 * @brief Represents a triangle edge and has all information related to it.
 */
class Edge {
 private:
  /**
   * @brief Edge's unique hash. The hash is calculated using the Node(s)' ids.
   */
  const uint64_t hash_;

  /**
   * @brief Returns the hash that would have an Edge having \a n0 and \a n1
   * as its Node(s).
   * 
   * @param[in] n0 
   * @param[in] n1 
   */
  static uint64_t computeHash(const Node &n0, const Node &n1);
  
  /**
   * @brief Returns the length that would have an Edge defined by \a n0
   * and \a n1.
   * 
   * @param[in] n0 
   * @param[in] n1 
   */
  static double computeLen(const Node &n0, const Node &n1);
  friend class std::hash<Edge>;

 public:
  /**
   * @brief An Edge is defined by two Node(s) (whose points create an edge).
   */
  const Node n0, n1;

  /**
   * @brief The length of the Edge.
   */
  const double len;

  /**
   * @brief Construct a new Edge object.
   * 
   * @param[in] n0 
   * @param[in] n1 
   */
  Edge(const Node &n0, const Node &n1);

  /**
   * @brief Comparison operator. Two Edge(s) will be equal if they have
   * the same hash.
   * 
   * @param[in] e 
   */
  bool operator==(const Edge &e) const;

  /**
   * @brief Negation of the comparison operator.
   * 
   * @param[in] e 
   */
  bool operator!=(const Edge &e) const;

  /**
   * @brief Updates the local coordinates of both Node(s).
   * 
   * @param[in] tf 
   */
  void updateLocal(const Eigen::Affine3d &tf) const;
  
  /**
   * @brief Returns the midpoint of the Edge in local coordinates.
   */
  Point midPoint() const;

  /**
   * @brief Returns the midpoint of the Edge in global coordinates.
   */
  Point midPointGlobal() const;

  /**
   * @brief Returns a normal vector to the Edge.
   */
  Vector normal() const;

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[int] e 
   */
  friend std::ostream &operator<<(std::ostream &os, const Edge &e);
};

template <>
struct std::hash<Edge> {
  uint64_t operator()(const Edge &e) const {
    return e.hash_;
  }
};