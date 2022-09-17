#pragma once

#include <unordered_set>
#include <list>
#include "structures/Triangle.hpp"
#include "structures/Edge.hpp"

using TriangleSet = std::unordered_set<Triangle>;
using EdgeSet = std::unordered_set<Edge>;

using Path = std::list<Edge>;