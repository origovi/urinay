/**
 * @file definitions.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the definitions (aliases) needed throughout the program.
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <unordered_set>

#include "structures/Edge.hpp"
#include "structures/Triangle.hpp"


/**
 * Represents a set of triangles.
 */
using TriangleSet = std::unordered_set<Triangle>;

/**
 * Represents a set of triangle edges.
 */
using EdgeSet = std::unordered_set<Edge>;

/**
 * Represents the heuristics and index of the correspondant Edge.
 * The heuristics are a pair of the actual heuristic value, and the angle
 * deviation in rad.
 */
using HeurInd = std::pair<std::pair<double, double>, size_t>;

/**
 * Represents the two traces of track limits (two vectors of points).
 */
using Tracklimits = std::pair<std::vector<Node>, std::vector<Node>>;