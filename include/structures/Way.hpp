/**
 * @file Way.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Way class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <as_msgs/PathLimits.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <list>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

/**
 * @brief Represents a way, i.e. the centerline and track limits.
 * It is the result of the WayComputer module.
 */
class Way {
 private:
  /**
   * @brief All parameters related to the Way class.
   */
  static Params::WayComputer::Way params_;

  /**
   * @brief The Way is represented with a list of Edge(s), these are directly
   * obtained from the Delaunay triangulation. In the list, the Edge(s) are
   * sorted as their position in the centerline. Track limits are intrinsically
   * here as well.
   */
  std::list<Edge> path_;

  /**
   * @brief The average length of all the Edge(s) stored in \a path_.
   * This attribute will be used by WayComputer to find next Edge(s).
   */
  double avgEdgeLen_;

 public:
  /**
   * @brief Method to initialize the Singleton.
   * 
   * @param[in] params 
   */
  static void init(const Params::WayComputer::Way &params);

  /**
   * @brief Construct a new Way object.
   */
  Way();

  /**
   * @brief Consults if the Way is empty.
   */
  bool empty() const;

  /**
   * @brief Consults the number of midpoints of the Way.
   */
  size_t size() const;

  /**
   * @brief Returns last Edge.
   */
  const Edge &back() const;

  /**
   * @brief Returns the penultimate Edge.
   */
  const Edge &beforeBack() const;

  /**
   * @brief Returns first Edge.
   */
  const Edge &front() const;

  /**
   * @brief Updates the local position of all Edge(s) in the Way.
   * 
   * @param[in] tf 
   */
  void updateLocal(const Eigen::Affine3d &tf);

  /**
   * @brief Appends an Edge to the back of the Way.
   * 
   * @param[in] edge 
   */
  void addEdge(const Edge &edge);

  /**
   * @brief Trims the Way so that all Edge(s) coming after the closest to the
   * car are removed.
   */
  void trimByLocal();

  /**
   * @brief Checks if the Way closes loop when \a e is appended to the Way.
   * If \a lastPosInTrace is not NULL, it is considered last Way midpoint.
   * 
   * @param[in] e 
   * @param[in] lastPosInTrace 
   */
  bool closesLoopWith(const Edge &e, const Point *lastPosInTrace = nullptr) const;

  /**
   * @brief Makes sure that first and last Edge(s) coincide.
   * **Note** that first and last global coordinates may not coincide due to
   * cone movement over the run.
   */
  void restructureClosure();

  /**
   * @brief Checks if the loop is closed. This means:
   * - Size is longer than a threshold
   * - Distance between first and last midpoints exceeds a threshold
   */
  bool closesLoop() const;

  /**
   * @brief Checks if the Way contains a specific Edge \a e.
   * 
   * @param[in] e
   */
  bool containsEdge(const Edge &e) const;

  /**
   * @brief Assignment operator.
   * 
   * @param[in] way 
   */
  Way &operator=(const Way &way);

  /**
   * @brief Comparison operator. Two Way(s) will be equal if both contain
   * the same Edge(s).
   * **Note** that the midpoints (and track limits) positions may not be equal.
   * 
   * @param[in] way 
   */
  bool operator==(const Way &way) const;

  /**
   * @brief Negation of the comparison operator.
   * 
   * @param[in] way 
   */
  bool operator!=(const Way &way) const;

  /**
   * @brief Returns the average Edge length.
   */
  const double &getAvgEdgeLen() const;

  /**
   * @brief Returns a vector with all the midpoints in global coordinates.
   */
  std::vector<Point> getPath() const;

  /**
   * @brief Returns the track limits. First element is left track limit
   * and second is right.
   */
  Tracklimits getTracklimits() const;

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] way 
   */
  friend std::ostream &operator<<(std::ostream &os, const Way &way);
};