#include "modules/DelaunayTri.hpp"

/* ----------------------------- Private Methods ---------------------------- */

Triangle DelaunayTri::superTriangle(const std::vector<Node> &nodes) {
  // Find the coords maxs and mins
  double xmax = nodes.front().x();
  double xmin = nodes.front().x();
  double ymax = nodes.front().y();
  double ymin = nodes.front().y();
  for (const Node &n : nodes) {
    xmax = std::max(xmax, n.x());
    xmin = std::min(xmin, n.x());
    ymax = std::max(ymax, n.y());
    ymin = std::min(ymin, n.y());
  }

  const double dx = xmax - xmin;
  const double dy = ymax - ymin;
  const double dmax = std::max(dx, dy);
  const double midx = (xmin + xmax) / 2.0;
  const double midy = (ymin + ymax) / 2.0;

  const Node n0 = Node::superTriangleNode(midx - 20 * dmax, midy - dmax);
  const Node n1 = Node::superTriangleNode(midx, midy + 20 * dmax);
  const Node n2 = Node::superTriangleNode(midx + 20 * dmax, midy - dmax);

  return {n0, n1, n2};
}

/* ----------------------------- Public Methods ----------------------------- */

/**
 * @brief Implementation of the Bowyer-Watson algorithm to find the Delaunay
 * triangulation given a set of points. O(nlogn).
 * 
 * @param nodes 
 * @param vis 
 * @return TriangleSet 
 */
TriangleSet DelaunayTri::compute(const std::vector<Node> &nodes) {
  if (nodes.size() < 3) return {};
  TriangleSet triangulation;

  // Add a triangle large enough to contain all the points
  triangulation.insert(superTriangle(nodes));

  // Add all the points one at a time to the triangulation
  for (const Node &n : nodes) {
    TriangleSet badTriangles;

    // First find all the triangles that are no longer valid due to the insertion
    for (const Triangle &t : triangulation) {
      if (t.circleContainsNode(n)) {
        badTriangles.insert(t);
      }
    }

    EdgeSet polygon;

    // Find the boundary of the polygonal hole
    for (const Triangle &t : badTriangles) {
      for (const Edge &e : t.edges) {
        bool shared = false;
        for (const Triangle &t2 : badTriangles) {
          if (&t != &t2 and t2.containsEdge(e)) {
            shared = true;
            break;
          }
        }
        if (not shared) {
          polygon.insert(e);
        }
      }
    }

    // Remove every bad triangle from the triangulation data structure
    for (const Triangle &t : badTriangles) {
      triangulation.erase(t);
    }

    // Re-triangulate the polygonal hole
    for (const Edge &e : polygon) {
      triangulation.emplace(e, n);
    }
  }

  // Remove every triangle that belonged to the super triangle
  auto it = triangulation.begin();
  while (it != triangulation.end()) {
    if (it->anyNodeInSuperTriangle()) {
      it = triangulation.erase(it);
    } else {
      it++;
    }
  }

  return triangulation;
}