#pragma once

#include <list>
#include <unordered_set>

#include "structures/Edge.hpp"
#include "structures/Triangle.hpp"

using TriangleSet = std::unordered_set<Triangle>;
using EdgeSet = std::unordered_set<Edge>;

using Tracklimits = std::pair<std::vector<Node>, std::vector<Node>>;