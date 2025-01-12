/**
 * @file Trace.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Trace class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <Eigen/Geometry>

#include "structures/Edge.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"

/**
 * @brief Represents a trace, i.e. an edge path in the tree search.
 * It is implemented using shared_ptr, this way we can clone an object and
 * append an Edge with O(1) time.
 */
class Trace {
 private:
  /**
   * @brief Inner class of Trace, contains information about last append
   * and a pointer to the Connection before.
   */
  struct Connection {
    /**
     * @brief A pointer to the Edge it represents.
     */
    const Edge edge;

    /**
     * @brief A shared pointer to the Connection that goes before this.
     */
    std::shared_ptr<Connection> before;

    /**
     * @brief Size of the Trace, i.e. how long is the Connection chain.
     */
    const size_t size;

    /**
     * @brief The heuristic of this append.
     */
    const double heur;

    /**
     * @brief The sum of heuristics of the Connection chain.
     */
    const double sumHeur;

    /**
     * @brief The average track width of all the Trace.
     */
    const double avgTrackWidth;

    /**
     * @brief Whether or not the Trace has closed the loop
     * (and thus it contains (or may contain) repeated Edge(s)).
     */
    const bool loopClosed;

    /**
     * @brief Number of Connections since the loop was closed.
     */
    const uint32_t connectionsSinceLoopClosed;

    /**
     * @brief Construct a new Connection object.
     * 
     * @param[in] edge 
     * @param[in] heur 
     * @param[in] loopClosed 
     * @param[in] before 
     */
    Connection(const Edge &edge, const double &heur, const bool &loopClosed, std::shared_ptr<Connection> before);

    /**
     * @brief Returns if there is \a _edge in the Connection chain.
     * 
     * @param[in] _edge 
     */
    bool containsEdge(const Edge &_edge) const;

    /**
     * @brief Cout operator.
     * 
     * @param[in,out] os 
     * @param[in] conn 
     */
    friend std::ostream &operator<<(std::ostream &os, const Connection &conn) {
      if (conn.before != nullptr) {
        os << conn.before;
      }
      return os << conn.edge << " -> ";
    }
  };

  /**
   * @brief All parameters related to the Trace class.
   */
  static Params::WayComputer::Way params_;

  /**
   * @brief IMPORTANT ATTRIBUTE, a shared pointer to a Connection.
   */
  std::shared_ptr<Connection> p;

  /**
   * @brief From the first, number of midpoints until the car is reached.
   * Set after trimming by local.
   */
  size_t sizeToCar_;

  /**
   * @brief Construct a new Trace object having \a p as a pointer to the
   * Connection chain.
   * 
   * @param[in] p 
   * @param[in] sizeToCar 
   */
  Trace(const std::shared_ptr<Connection> &p, const size_t &sizeToCar);

 public:
  /**
   * @brief Method to initialize the Class.
   * 
   * @param[in] params 
   */
  static void init(const Params::WayComputer::Way &params);
  /**
   * @brief Construct a new Trace object.
   */
  Trace();

  /**
   * @brief Construct a new Trace object with a possible previous Trace.
   * 
   * @param[in] edge 
   * @param[in] heur 
   * @param[in] previous represents the previous Trace, this new will extend
   */
  Trace(const Edge &edge, const double &heur, const Trace &previous = Trace());

  /**
   * @brief Appends an Edge as a Connection with the following data.
   * 
   * @param[in] edge 
   * @param[in] heur 
   */
  void addEdge(const Edge &edge, const double &heur);

  /**
   * @brief Consults if the Trace is empty.
   */
  bool empty() const;

  /**
   * @brief Returns the size of the Trace, i.e. how long is the Connection chain.
   * Complexity: O(1).
   */
  size_t size() const;

  /**
   * @brief Returns the number of midpoints ahead of the car, until end of Trace.
   * Does not take loop closure into account.
   */
  size_t sizeAheadOfCar() const;

  /**
   * @brief Returns a Trace containing the Connection chain of the Connection
   * before.
   * Complexity: O(1).
   * 
   * @param[in] num is the number of elements to iterate back
   */
  Trace before(const uint64_t &num = 1) const;

  /**
   * @brief Returns a Trace containing the first (initial) Connection.
   * Complexity: O(n), being n the number of Connections in the chain.
   */
  Trace first() const;

  /**
   * @brief Returns a Trace containing the second Connection.
   * Complexity: O(n), being n the number of Connections in the chain.
   */
  Trace second() const;

  /**
   * @brief Returns the last Edge.
   * Complexity: O(1).
   */
  const Edge &edge() const;

  /**
   * @brief Returns the penultimate Edge.
   */
  const Edge &beforeBack() const;

  /**
   * @brief Returns the last Connection heuristic.
   * Complexity: O(1).
   */
  const double &heur() const;

  /**
   * @brief Returns the last Connection's average track width.
   * If empty, returns 0.
   * Complexity: O(1).
   */
  double avgTrackWidth() const;

  /**
   * @brief Returns whether of not the loop is closed from last
   * Connection or before.
   * Complexity: O(1).
   */
  bool isLoopClosed() const;

  /**
   * @brief Returns the sum of heuristics of the Connection chain.
   * If empty, returns infinity.
   * Complexity: O(1).
   */
  double sumHeur() const;

  /**
   * @brief Checks if there is any Connection with Edge \a edge.
   * Complexity: O(n), being n the number of Connections in the chain.
   * 
   * @param[in] edge 
   */
  bool containsEdge(const Edge &edge) const;

  /**
   * @brief Checks if the Trace closes loop when \a e is appended.
   * 
   * @param[in] e 
   */
  bool closesLoopWith(const Edge &e) const;

  /**
   * @brief Returns the number of Connections in the chain since the loop was
   * first closed.
   * Complexity: O(1).
   */
  uint32_t connectionsSinceLoopClosed() const;

  /**
   * @brief Trims the Trace so that all Edge(s) coming after the closest to the
   * car are removed.
   * Note: distance to car is known by dist to (0,0).
   */
  void trimByLocal(const Eigen::Vector3d &carPosition);

  /**
   * @brief Returns the same Trace but trims the 'extra' Edges after the loop
   * has been closed.
   */
  Trace trimLoopClosure() const;

  /**
   * @brief Makes a copy making sure that:
   * - Possible spare points are removed. (e.g. when the loop closes with the second point).
   * - First and last Edge(s) coincide (EXACTLY equals).
   * WARNING: The resultant Trace has not correct Connection parameters and
   * shall only be used for publish purposes.
   */
  Trace restructureClosure() const;

  /**
   * @brief Checks if Edge \a e creates an intersection (a loop) on the path.
   * O(n), n=this->size().
   * 
   * @param[in] e 
   */
  bool intersectsWith(const Edge &e) const;

  /**
   * @brief Clears the Connection chain. O(1).
   */
  void clear();

  /**
   * @brief Detaches (breaks) the Connection chain and makes the current
   * Connection the first of the chain. O(1).
   */
  void detach() const;

    /**
   * @brief Updates the local position of all Edge(s) in the Connection chain.
   * 
   * @param[in] tf 
   */
  void updateLocal(const Eigen::Affine3d &tf) const;

  /**
   * @brief Comparison operator used to see if the implicit object is a "better"
   * Trace compared to parameter.
   * The method of choosing the best trace is as follows:
   * 1. The longest trace wins.
   * 2. If the size is equal, then the trace with smallest accum heuristic wins.
   */
  bool operator<(const Trace &t) const;

  /**
   * @brief Comparison operator. Two Trace(s) will be equal if both contain
   * the same Edge(s) in the Connection chain.
   * **Note** that the midpoints (and track limits) positions may not be equal.
   * 
   * @param[in] t 
   */
  bool operator==(const Trace &t) const;

  /**
   * @brief Negation of the comparison operator.
   * 
   * @param[in] t 
   */
  bool operator!=(const Trace &t) const;

  /**
   * @brief Checks if the vital_num_midpoints (the n midpoints after car's position)
   * are equal in both \a *this and \a t.
   * 
   * @param[in] t 
   */
  bool vitalMidpointsChanged(const Trace &t) const;

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] trace 
   */
  friend std::ostream &operator<<(std::ostream &os, const Trace &trace);

  /**
   * @brief Returns a vector with all the midpoints in global coordinates.
   */
  std::vector<Point> getPath() const;

  /**
   * @brief Returns the track limits. First element is left track limit
   * and second is right.
   */
  Tracklimits getTracklimits() const;
};