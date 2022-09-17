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

/* ----------------------------- Public Methods ----------------------------- */

Way::Way(const Params::Way &params) : params_(params) {
}

void Way::update(TriangleSet &triangulation, const Visualization &vis) {
  // #1: Remove all triangles which we know will not be part of the track.
  filterTriangulation(triangulation);

  // #2: Extract all midpoints without repetitions, do that through an EdgeSet
  // so no midpoint is got twice.
  EdgeSet edges;
  for (const Triangle &t : triangulation) {
    for (const Edge &e : t.edges) {
      edges.insert(e);
    }
  }

  // #3: Filter the midpoints. Only the ones having a circumcenter near, will be
  // considered. To do so:
  // - #3.1 Build a k-d tree of all circumcenters so finding matches is O(logn).
  // - #3.2 Perform the filtering.

  // #3.1: Building the tree
  std::vector<Point> circums(triangulation.size());
  std::transform(triangulation.begin(), triangulation.end(), circums.begin(),
                 [](const Triangle &t) -> const Point & { return t.circumCenter(); });
  KDTree circumKDTree(circums);

  // #3.2: Filtering
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

  vis.visualize(edges);  

}