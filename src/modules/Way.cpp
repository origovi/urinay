#include "modules/Way.hpp"

/* ----------------------------- Private Methods ---------------------------- */

void Way::filterTriangulation(TriangleSet &triangulation) const {
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

void Way::filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const {
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

void Way::findNextPossibleEdges(std::vector<std::pair<size_t, float>> &nextPossibleEdges, const Point &point, const Point &dir, const KDTree &midpointsKDT) const {
  nextPossibleEdges.clear();
}

void Way::computePath(Path &path, const std::vector<const Edge *> &edges) const {
  // Build a k-d tree of all midpoints
  std::vector<Point> midpoints(edges.size());
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge *e) -> Point { return e->midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. The tree height will be limited, at that point the first
  // element of the highest with lower heuristic Trace will be added to the
  // path. The search finishes when no element can be added to the path.
  std::vector<std::pair<size_t, float>> nextPossibleEdges;
  findNextPossibleEdges(nextPossibleEdges, Point(0, 0), Point(1, 0), midpointsKDT);
  while (not nextPossibleEdges.empty()) {
    Trace best;
    std::queue<Trace> cua;
    for (const std::pair<size_t, float> &nextEdge : nextPossibleEdges) {
      cua.emplace(nextEdge.first, nextEdge.second);
    }

    while (not cua.empty()) {
      Trace t = cua.front();
      cua.pop();

      bool trace_at_max_height = false;
      if (t.size() >= params_.max_search_tree_height)
        trace_at_max_height = true;
      else {
        Point dir = edges[t.edgeInd()]->midPoint();
        if (t.size() >= 2)
          dir -= edges[t.before().edgeInd()]->midPoint();
        else if (not path.empty())
          dir -= path.back().midPoint();
        findNextPossibleEdges(nextPossibleEdges, edges[t.edgeInd()]->midPoint(), dir, midpointsKDT);
      }

      if (trace_at_max_height or nextPossibleEdges.empty()) {
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
        for (const std::pair<size_t, float> &nextEdge : nextPossibleEdges) {
          Trace aux = t;
          aux.addEdge(nextEdge.first, nextEdge.second);
          cua.push(aux);
        }
      }
    }

    // Find the direction the path is now pointing at. 2 cases:
    // - If the path is empty, the dir vector A->B (B-A) will be B (A=(0,0))
    // - Otherwise, the dir vector will be B-A
    Point dir = edges[best.first().edgeInd()]->midPoint();
    if (not path.empty())
      dir -= path.back().midPoint();

    path.push_back(*edges[best.first().edgeInd()]);
    findNextPossibleEdges(nextPossibleEdges, path.back().midPoint(), dir, midpointsKDT);
  }
}

void Way::mergeToHistoric(const Path &path) {
  historic_ = std::list<Edge>(path);
}

/* ----------------------------- Public Methods ----------------------------- */

Way::Way(const Params::Way &params) : params_(params) {}

void Way::update(TriangleSet &triangulation, const Visualization &vis) {
  // #1: Remove all triangles which we know will not be part of the track.
  filterTriangulation(triangulation);

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
  filterMidpoints(edgeSet, triangulation);
  vis.visualize(edgeSet);

  // Convert this set to a vector
  std::vector<const Edge *> edgeVec;
  edgeVec.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    edgeVec.push_back(&e);
  }

  // #4: Perform the search through the midpoints in order to obtain a path.
  Path path;
  computePath(path, edgeVec);

  // #5: Update the historical path
  mergeToHistoric(path);
}