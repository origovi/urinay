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

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Edge *actEdge, const Vector &dir, const KDTree &midpointsKDT, const std::vector<Edge> &edges) const {
  nextEdges.clear();
  Point actPos = actEdge ? actEdge->midPoint() : Point(0, 0);

  // Find all possible edges in a specified radius
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params_.search_radius);

  // Discard:
  // - all Edges whose midpoint is behind actPos
  // - the one that corresponds to actPos
  // - (not in first iteration) all Edges already contained in the Way that do not close the loop
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {
    const Edge &nextPossibleEdge = edges[*it];
    if (&edges[*it] == actEdge or Vector::pointBehind(nextPossibleEdge.midPoint(), actPos, dir) or (actEdge and not this->way_.closesLoopWith(nextPossibleEdge) and this->way_.containsEdge(nextPossibleEdge)))
      it = nextPossibleEdges.erase(it);
    else
      it++;
  }

  // Get the heuristics for all possible next edges and filter them (only the
  // ones having a heuristic small enough will prevail)
  std::vector<HeurInd> privilege_runner;
  privilege_runner.reserve(nextPossibleEdges.size());
  for (const size_t &nextPossibleEdgeInd : nextPossibleEdges) {
    double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd].midPoint(), dir);
    if (heuristic <= params_.max_next_heuristic) privilege_runner.emplace_back(heuristic, nextPossibleEdgeInd);
  }

  // Copy the n best HeurInd(s) into the nextEdges vector, according to
  // cppreference.com, this is O(nlogn)
  nextEdges.resize(std::min(privilege_runner.size(), (size_t)params_.max_search_options));
  std::partial_sort_copy(privilege_runner.begin(), privilege_runner.end(), nextEdges.begin(), nextEdges.end());
}

void WayComputer::computeWay(const std::vector<Edge> &edges) {
  // Get rid of all edges from closest to car (included) to last
  this->way_.trimByLocal();

  // Build a k-d tree of all midpoints
  std::vector<Point> midpoints(edges.size());
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge &e) -> Point { return e.midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. The tree height will be limited, at that point the first
  // element of the longest with lower heuristic Trace will be added to the
  // way. The search finishes when no element can be added to the way.
  std::vector<HeurInd> nextEdges;
  const Edge *actEdge = nullptr;
  Point actPos = Point(1, 0);
  Point antPos = Point(0, 0);
  if (not this->way_.empty()) {
    actEdge = &this->way_.back();
    if (this->way_.size() >= 2) {
      actPos = this->way_.back().midPoint();
      antPos = this->way_.beforeBack().midPoint();
    }
  }
  this->findNextEdges(nextEdges, actEdge, Vector(antPos, actPos), midpointsKDT, edges);

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
        Point actPos = edges[t.edgeInd()].midPoint();
        Point antPos(0, 0);
        if (t.size() >= 2)
          antPos = edges[t.before().edgeInd()].midPoint();
        else if (not this->way_.empty())
          antPos = this->way_.back().midPoint();
        this->findNextEdges(nextEdges, &edges[t.edgeInd()], Vector(antPos, actPos), midpointsKDT, edges);
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

    const Edge &edgeToAppend = edges[best.first().edgeInd()];
    Point nextPos = edgeToAppend.midPoint();
    actPos = this->way_.back().midPoint();

    this->way_.addEdge(edgeToAppend);

    // Check for loop closure
    if (this->way_.closesLoop()) {
      this->way_.restructureClosure();
      this->isLoopClosed_ = true;
      std::cout << "TANQUEM LOOP!" << std::endl;
      return;
    }

    this->findNextEdges(nextEdges, &edgeToAppend, Vector(actPos, nextPos), midpointsKDT, edges);
  }
  std::cout << "sortim  " << this->way_.size() << std::endl;
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params) {}

void WayComputer::poseCallback(const nav_msgs::Odometry::ConstPtr &data) {
  tf::poseMsgToEigen(data->pose.pose, this->localTf_);

  // Invert y and z axis
  static const Eigen::Matrix4d aux = (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0,
                                      0.0, -1.0, 0.0, 0.0,
                                      0.0, 0.0, -1.0, 0.0,
                                      0.0, 0.0, 0.0, 1.0)
                                         .finished();
  this->localTf_ = this->localTf_.inverse();
  this->localTf_.matrix() = this->localTf_.matrix() * aux;

  this->localTfValid_ = true;
}

void WayComputer::update(TriangleSet &triangulation, const Visualization &vis) {
  if (not this->localTfValid_) return;

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
  std::vector<Edge> edgeVec;
  edgeVec.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    edgeVec.push_back(e);
  }
  
  // #4: Update all local positions (way and edges) with car tf
  this->way_.updateLocal(this->localTf_);
  for (Edge &e : edgeVec) {
    e.updateLocal(this->localTf_);
  }

  // #5: Perform the search through the midpoints in order to obtain a way.
  this->computeWay(edgeVec);

  vis.visualize(this->way_);
}

const bool &WayComputer::isLoopClosed() const {
  return this->isLoopClosed_;
}

void WayComputer::writeWayToFile(const std::string &file_path) const {
  std::ofstream oStreamToWrite(file_path);
  oStreamToWrite << this->way_;
  oStreamToWrite.close();
}

std::vector<Point> WayComputer::getPath() const {
  return this->way_.getPath();
}

Tracklimits WayComputer::getTracklimits() const {
  return this->way_.getTracklimits();
}

as_msgs::PathLimits WayComputer::getPathLimits() const {
  return this->way_.getPathLimits();
}