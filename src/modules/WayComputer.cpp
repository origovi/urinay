#include "modules/WayComputer.hpp"

/* ----------------------------- Private Methods ---------------------------- */

void WayComputer::filterTriangulation(TriangleSet &triangulation) const {
  auto it = triangulation.begin();
  while (it != triangulation.end()) {
    bool removeTriangle = false;
    for (const Edge &e : it->edges) {
      if (e.len > params_.max_triangle_edge_len) {
        removeTriangle = true;
        break;
      }
    }
    if (removeTriangle)
      it = triangulation.erase(it);
    else
      it++;
  }
}

void WayComputer::filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const {
  // Build a k-d tree of all circumcenters so finding matches is O(logn)
  std::vector<Point> circums(triangulation.size());
  std::transform(triangulation.begin(), triangulation.end(), circums.begin(),
                 [](const Triangle &t) -> const Point & { return t.circumCenter(); });
  KDTree circumKDTree(circums);

  // Perform the filtering
  auto it = edges.begin();
  while (it != edges.end()) {
    Point midPoint = it->midPoint();
    KDTData<size_t> nearestCC = circumKDTree.nearest_index(midPoint);
    if (bool(nearestCC) and Point::distSq(circums[*nearestCC], midPoint) > pow(params_.max_dist_circum_midPoint, 2)) {
      it = edges.erase(it);
    } else {
      it++;
    }
  }
}

float WayComputer::getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir) const {
  double distHeur = Point::dist(actPos, nextPos);

  double angle = Vector(actPos, nextPos).angle(dir);
  double angleHeur = -log(std::max(0.0, ((M_PI_2 - abs(angle)) / M_PI_2) - 0.2));

  return params_.heur_dist_ponderation * distHeur + (1 - params_.heur_dist_ponderation) * angleHeur;
}

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Edge *actEdge, const Vector &dir, const KDTree &midpointsKDT, const std::vector<const Edge *> &edges) const {
  nextEdges.clear();
  Point actPos = actEdge ? actEdge->midPoint() : Point(0, 0);

  // Find all possible edges in a specified radius
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params_.search_radius);

  // Discard:
  // - all Edges whose midpoint is behind actPos
  // - the one that corresponds to actPos
  // - all Edges already contained in the Way(s)
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {
    if (edges[*it] == actEdge or Vector::pointBehind(edges[*it]->midPoint(), actPos, dir))
      it = nextPossibleEdges.erase(it);
    else
      it++;
  }

  // Get the heuristics for all possible next edges and filter them (only the
  // ones having a heuristic small enough will prevail)
  std::vector<HeurInd> privilege_runner;
  privilege_runner.reserve(nextPossibleEdges.size());
  for (const size_t &nextPossibleEdgeInd : nextPossibleEdges) {
    double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd]->midPoint(), dir);
    if (heuristic <= params_.max_next_heuristic) privilege_runner.emplace_back(heuristic, nextPossibleEdgeInd);
  }

  // Copy the n best HeurInd(s) into the nextEdges vector, according to
  // cppreference.com, this is O(nlogn)
  nextEdges.resize(std::min(privilege_runner.size(), (size_t)params_.max_search_options));
  std::partial_sort_copy(privilege_runner.begin(), privilege_runner.end(), nextEdges.begin(), nextEdges.end());
}

void WayComputer::computeWay(Way &way, const std::vector<const Edge *> &edges) const {
  // Build a k-d tree of all midpoints
  std::vector<Point> midpoints(edges.size());
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge *e) -> Point { return e->midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. The tree height will be limited, at that point the first
  // element of the highest with lower heuristic Trace will be added to the
  // way. The search finishes when no element can be added to the way.
  std::vector<HeurInd> nextEdges;
  this->findNextEdges(nextEdges, nullptr, Vector(1, 0), midpointsKDT, edges);

  while (not nextEdges.empty() and ros::ok()) {
    Trace best;
    std::queue<Trace> cua;
    for (const HeurInd &nextEdge : nextEdges) {
      cua.emplace(nextEdge.second, nextEdge.first);
    }

    while (not cua.empty()) {
      Trace t = cua.front();
      cua.pop();

      bool trace_at_max_height = false;

      if (t.size() >= params_.max_search_tree_height)
        trace_at_max_height = true;
      else {
        Point actPos = edges[t.edgeInd()]->midPoint();
        Point antPos(0, 0);
        if (t.size() >= 2)
          antPos = edges[t.before().edgeInd()]->midPoint();
        else if (not way.empty())
          antPos = way.back().midPoint();
        this->findNextEdges(nextEdges, edges[t.edgeInd()], Vector(antPos, actPos), midpointsKDT, edges);
      }

      if (trace_at_max_height or nextEdges.empty()) {
        // Means that this trace is finished, should be considered as the "best"
        // trace. The method of choosing the best trace is as follows:
        // 1. The longest trace wins.
        // 2. If the size is equal, then the trace with smaller heuristic wins.
        // Note that here, no trace is added to the queue.
        if (t.size() > best.size() or (t.size() == best.size() and t.sumHeur() < best.sumHeur())) {
          best = t;
        }
      } else {
        // Add new possible traces to the queue
        for (const HeurInd &nextEdge : nextEdges) {
          Trace aux = t;
          aux.addEdge(nextEdge.second, nextEdge.first);
          cua.push(aux);
        }
      }
    }

    // Find the direction the way is now pointing at. 2 cases:
    // - If the way is empty, the dir vector actPos->nextPos
    // - Otherwise, the dir vector will be B-A
    const Edge *edgeToAppend = edges[best.first().edgeInd()];
    Point nextPos = edgeToAppend->midPoint();
    Point actPos(0, 0);
    if (not way.empty())
      actPos = way.back().midPoint();

    way.addEdge(*edgeToAppend);

    // Check for loop closure
    if (historic_.closesLoopWith(way) or way.closesLoop()) {
      std::cout << "TANQUEM LOOP!" << std::endl;
      return;
    }

    // for (const Edge &e : way) {
    //   std::cout << e.midPoint() << std::endl;
    // }

    this->findNextEdges(nextEdges, edgeToAppend, Vector(actPos, nextPos), midpointsKDT, edges);
  }
  std::cout << "sortim" << std::endl;
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params) {}

void WayComputer::update(TriangleSet &triangulation, const Visualization &vis) {
  // #1: Remove all triangles which we know will not be part of the track.
  this->filterTriangulation(triangulation);

  // #2: Extract all midpoints without repetitions, do that through an EdgeSet
  // so no midpoint is got twice.
  EdgeSet edgeSet;
  for (const Triangle &t : triangulation) {
    for (const Edge &e : t.edges) {
      edgeSet.insert(e);
    }
  }

  // #3: Filter the midpoints. Only the ones having a circumcenter near, will be
  // left.
  this->filterMidpoints(edgeSet, triangulation);
  vis.visualize(edgeSet);

  // Convert this set to a vector
  std::vector<const Edge *> edgeVec;
  edgeVec.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    edgeVec.push_back(&e);
  }

  // #4: Perform the search through the midpoints in order to obtain a way.
  Way way;
  this->computeWay(way, edgeVec);

  // vis.visualize(way);
  // #5: Update the historical way
  historic_.mergeWith(way);  // Destroys way

  vis.visualize(historic_);
}

const Way &WayComputer::way() const {
  return this->historic_;
}