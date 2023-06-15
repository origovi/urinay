/**
 * @file WayComputer.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the WayComputer class specification
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <as_msgs/CarState.h>
#include <as_msgs/Tracklimits.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <queue>

#include "modules/Visualization.hpp"
#include "structures/Trace.hpp"
#include "structures/Vector.hpp"
#include "structures/Way.hpp"
#include "utils/KDTree.hpp"
#include "utils/Params.hpp"
#include "utils/Failsafe.hpp"
#include "utils/constants.hpp"
#include "utils/definitions.hpp"

/**
 * @brief A class that has all tools and functions to compute the Way.
 * It takes the Delaunay set and the car position to do so.
 */
class WayComputer {
 private:
  /**
   * @brief All parameters related to the WayComputer class.
   */
  const Params::WayComputer params_;

  /**
   * @brief Failsafe of search parameters.
   */
  Failsafe<Params::WayComputer::Search> colorFailsafe_, generalFailsafe_;

  /**
   * @brief The result of the computation and last iteration's result.
   */
  Way way_, lastWay_;

  /**
   * @brief This Way object had to be created to solve the non-stop loop
   * calculation. It is the Way that will be published every time (for both
   * full & partial).
   */
  Way wayToPublish_;

  /**
   * @brief Last data timestamp.
   */
  ros::Time lastStamp_;

  /**
   * @brief Whether or not \a way_ has its loop closed.
   */
  bool isLoopClosed_ = false;

  /**
   * @brief The transform between global and local frame.
   */
  Eigen::Affine3d localTf_;

  /**
   * @brief Whether or not \a localTf_ is valid.
   */
  bool localTfValid_ = false;

  /**
   * @brief Filters the TriangleSet and removes all unwanted triangles.
   *
   * @param[in,out] triangulation
   */
  void filterTriangulation(TriangleSet &triangulation) const;

  /**
   * @brief Filters the Edges by their midpoints and removes all unwanted Edge(s).
   *
   * @param[in,out] edges
   * @param[in] triangulation
   */
  void filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const;

  /**
   * @brief Computes and returns the heuristic based on angle and distance.
   * Uses \a params as its parameters.
   *
   * @param[in] actPos
   * @param[in] nextPos
   * @param[in] dir
   * @param[in] params
   */
  double getHeuristic(const Point &actPos,
                      const Point &nextPos,
                      const Vector &dir,
                      const Params::WayComputer::Search &params) const;

  /**
   * @brief Returns the average edge length of the Way and the Trace \a *trace
   * (if any).
   *
   * @param[in] trace
   */
  inline double avgEdgeLen(const Trace *trace) const;

  /**
   * @brief Decides whether or not the color of the cones of \a edge is correct.
   *
   * @param edge
   * @param dir
   * @param oneside specifies which criteria will be used, to allow one mismatch or not.
   * @return true
   * @return false
   */
  bool edgeWithCorrectColor(const Edge &edge, const Vector &dir, const bool &oneside) const;

  /**
   * @brief Finds all possible next Edges according to all metrics and thresholds.
   * Uses \a params as its parameters.
   *
   * @param[out] nextEdges
   * @param[in] actTrace
   * @param[in] midpointsKDT
   * @param[in] edges
   * @param[in] params
   */
  void findNextEdges(std::vector<HeurInd> &nextEdges,
                     const Trace *actTrace,
                     const KDTree &midpointsKDT,
                     const std::vector<Edge> &edges,
                     const Params::WayComputer::Search &params) const;

  /**
   * @brief Computes which is the best Trace from the best previous best Trace
   * and the candidate.
   *
   * @param best is the best previous Trace
   * @param t is the candidate to best Trace
   * @return Trace is the new best Trace
   */
  Trace computeBestTraceWithFinishedT(const Trace &best, const Trace &t) const;

  /**
   * @brief Performs a limited-height heuristic-ponderated tree search and
   * returns the index of the best possible next Edge. Uses \a params as its
   * parameters.
   *
   * @param[in] nextEdges
   * @param[in] midpointsKDT
   * @param[in] edges
   * @param[in] params
   */
  size_t treeSearch(std::vector<HeurInd> &nextEdges,
                    const KDTree &midpointsKDT,
                    const std::vector<Edge> &edges,
                    const Params::WayComputer::Search &params) const;

  /**
   * @brief Main function of the class, it takes all Edges and computes the best
   * possible centerline (Way). Uses \a params as its parameters.
   *
   * @param[in] edges
   * @param[in] params
   */
  void computeWay(const std::vector<Edge> &edges, const Params::WayComputer::Search &params);

 public:
  /**
   * @brief Construct a new Way Computer object.
   *
   * @param[in] params
   */
  WayComputer(const Params::WayComputer &params);

  /**
   * @brief Callback of the car's state.
   *
   * @param[in] data
   */
  void stateCallback(const as_msgs::CarState::ConstPtr &data);

  /**
   * @brief Takes the Delaunay triangle set and computes the Way.
   *
   * @param[in,out] triangulation
   * @param[in] stamp
   */
  void update(TriangleSet &triangulation, const ros::Time &stamp);

  /**
   * @brief Returns if the loop has been closed.
   */
  const bool &isLoopClosed() const;

  /**
   * @brief Writes the Way to the file path specified.
   *
   * @param[in] file_path
   */
  void writeWayToFile(const std::string &file_path) const;

  /**
   * @brief Returns if the attribute localTf is valid.
   */
  const bool &isLocalTfValid() const;

  /**
   * @brief Returns the transformation from global to local.
   */
  const Eigen::Affine3d &getLocalTf() const;

  /**
   * @brief Returns the centerline vector in global coordinates.
   */
  std::vector<Point> getPath() const;

  /**
   * @brief Returns the track limits in global coordinates.
   */
  Tracklimits getTracklimits() const;

  /**
   * @brief Returns the centerline and track limits in as_msgs format
   * in global coordinates.
   */
  as_msgs::PathLimits getPathLimits() const;
};