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

#include "structures/Edge.hpp"

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
    const Edge* const edge;

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
     * @brief The average Edge length of all the Trace.
     */
    const double avgEdgeLen;

    /**
     * @brief Whether or not the Trace has closed the loop
     * (and thus it contains (or may contain) repeated Edge(s)).
     */
    const bool loopClosed;

    /**
     * @brief Construct a new Connection object.
     * 
     * @param[in] edge 
     * @param[in] heur 
     * @param[in] loopClosed 
     * @param[in] before 
     */
    Connection(const Edge* const edge, const double &heur, const bool &loopClosed, std::shared_ptr<Connection> before);

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
        os << *(conn.before);
      }
      return os << *conn.edge << " -> ";
    }
  };

  /**
   * @brief ONLY ATTRIBUTE, a shared pointer to a Connection.
   */
  std::shared_ptr<Connection> p;

  /**
   * @brief Construct a new Trace object having \a p as a pointer to the
   * Connection chain.
   * 
   * @param[in] p 
   */
  Trace(std::shared_ptr<Connection> p);

 public:
  /**
   * @brief Construct a new Trace object.
   */
  Trace();

  /**
   * @brief Construct a new Trace object and the first Connection object.
   * 
   * @param[in] edge 
   * @param[in] heur 
   * @param[in] loopClosed 
   * @param[in] previous represents the previous Trace, this new will extend
   */
  Trace(const Edge &edge, const double &heur, const bool &loopClosed, const Trace &previous = Trace());

  /**
   * @brief Appends an Edge as a Connection with the following data.
   * 
   * @param[in] edge 
   * @param[in] heur 
   * @param[in] loopClosed 
   */
  void addEdge(const Edge &edge, const double &heur, const bool &loopClosed = false);

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
   * @brief Returns a Trace containing the Connection chain of the Connection
   * before.
   * Complexity: O(1).
   */
  Trace before() const;

  /**
   * @brief Returns a Trace containing the first (initial) Connection.
   * Complexity: O(n), being n the number of Connections in the chain.
   */
  Trace first() const;

  /**
   * @brief Returns the last Edge.
   * Complexity: O(1).
   */
  const Edge &edge() const;

  /**
   * @brief Returns the last Connection heuristic.
   * Complexity: O(1).
   */
  const double &heur() const;

  /**
   * @brief Returns the last Connection's average Edge length.
   * If empty, returns 0.
   * Complexity: O(1).
   */
  double avgEdgeLen() const;

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
   * @brief Clears the Connection chain. O(1).
   */
  void clear();

  /**
   * @brief Comparison operator used to see if the implicit object is a "better"
   * Trace compared to parameter.
   * The method of choosing the best trace is as follows:
   * 1. The longest trace wins.
   * 2. If the size is equal, then the trace with smallest accum heuristic wins.
   */
  bool operator<(const Trace &t) const;

  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] trace 
   */
  friend std::ostream &operator<<(std::ostream &os, const Trace &trace);
};