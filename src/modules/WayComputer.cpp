/**
 * @file WayComputer.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the WayComputer class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/WayComputer.hpp"

/* ----------------------------- Private Methods ---------------------------- */

void WayComputer::filterTriangulation(TriangleSet &triangulation) const {
  auto it = triangulation.begin();
  while (it != triangulation.end()) {
    bool removeTriangle = false;
    // Look for edges longer than accepted
    for (const Edge &e : it->edges) {
      if (e.len > params_.max_triangle_edge_len) {
        removeTriangle = true;
        break;
      }
    }

    // Look for angles smaller than accepted
    if (!removeTriangle) {
      for (const double &angle : it->angles()) {
        if (angle < this->params_.min_triangle_angle) {
          removeTriangle = true;
          break;
        }
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

double WayComputer::getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir) const {
  double distHeur = Point::dist(actPos, nextPos);

  double angle = Vector(actPos, nextPos).angleWith(dir);
  double angleHeur = -log(std::max(0.0, ((M_PI_2 - abs(angle)) / M_PI_2) - 0.2));

  return params_.heur_dist_ponderation * distHeur + (1 - params_.heur_dist_ponderation) * angleHeur;
}

inline double WayComputer::avgEdgeLen(const Trace *trace) const {
  if (not trace or trace->empty())
    return this->way_.getAvgEdgeLen();
  else if (this->way_.empty())
    return trace->avgEdgeLen();
  else
    return ((trace->avgEdgeLen() * trace->size()) / (trace->size() + this->way_.size())) + ((this->way_.getAvgEdgeLen() * this->way_.size()) / (trace->size() + this->way_.size()));
}

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Trace *actTrace, const KDTree &midpointsKDT, const std::vector<Edge> &edges) const {
  nextEdges.clear();

  const Edge *actEdge = nullptr;
  Point actPos(0, 0);
  Point lastPos(0, 0);

  // Set actual and last position
  if (actTrace and not actTrace->empty()) {
    actEdge = &edges[actTrace->edgeInd()];
    actPos = edges[actTrace->edgeInd()].midPoint();
    if (actTrace->size() >= 2) {
      lastPos = edges[actTrace->before().edgeInd()].midPoint();
    }
  }
  if (not this->way_.empty()) {
    if (not actEdge) {
      actEdge = &this->way_.back();
      actPos = this->way_.back().midPoint();
      if (this->way_.size() >= 2) {
        lastPos = this->way_.beforeBack().midPoint();
      }
    } else if (actTrace->size() < 2) {
      lastPos = this->way_.back().midPoint();
    }
  }

  // Set dir vector
  Vector dir;
  if (actEdge and (this->way_.size() >= 2 or actTrace))  // This condition avoids setting a vector pointing backwards
    dir = Vector(lastPos, actPos);
  else
    dir = Vector(1, 0);

  // Find all possible edges in a specified radius
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params_.search_radius);

  // Discard edges by specifications
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {
    const Edge &nextPossibleEdge = edges[*it];
    // The currently-iterated Edge will be removed if any of the below conditions is true
    bool removeConditions = actEdge and (
      // Remove itself from being the next one
      nextPossibleEdge == *actEdge or

      // Remove any edge whose midpoint create an angle too closed with last one
      abs(dir.angleWith(Vector(actPos, nextPossibleEdge.midPoint()))) > this->params_.max_angle_diff or

      // [Only before appending the edge that closes the loop] Remove any edge that is already contained in the path but is not the one that closes the loop
      (not this->way_.closesLoopWith(nextPossibleEdge) and (not actTrace or not actTrace->isLoopClosed()) and this->way_.containsEdge(nextPossibleEdge)) or

      // Remove any edge whose midpoint and lastPos are in the same side of actEdge (avoid bouncing on a track limit)
      ((this->way_.size() >= 2 or actTrace) and Vector::pointBehind(actEdge->midPoint(), lastPos, actEdge->normal()) == Vector::pointBehind(actEdge->midPoint(), nextPossibleEdge.midPoint(), actEdge->normal())) or

      // Remove any edge whose length is too big or too small compared to the average edge length of the way
      (nextPossibleEdge.len < (1 - this->params_.edge_len_diff_factor) * this->avgEdgeLen(actTrace) or nextPossibleEdge.len > (1 + this->params_.edge_len_diff_factor) * this->avgEdgeLen(actTrace)) or

      // [If not allow_intersection, only before closing the loop] Remove any Edge which appended would create an intersection
      (not this->params_.allow_intersection and (not actTrace or not actTrace->isLoopClosed()) and not this->way_.closesLoopWith(nextPossibleEdge) and this->way_.intersectsWith(nextPossibleEdge))
    );

    if (removeConditions)
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

Trace WayComputer::computeBestTraceWithFinishedT(const Trace &best, const Trace &t) const {
  // The method of choosing the best trace is as follows:
  // 1. The longest trace wins.
  // 2. If the size is equal, then the trace with smallest accum heuristic wins.
  // Note that here, no trace is added to the queue.
  if (t.size() > best.size() or (t.size() == best.size() and t.sumHeur() < best.sumHeur())) {
    return t;
  }
  else return best;
}

size_t WayComputer::treeSearch(std::vector<HeurInd> &nextEdges, const KDTree &midpointsKDT, const std::vector<Edge> &edges) const {
  std::queue<Trace> cua;
  for (const HeurInd &nextEdge : nextEdges) {
    bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second]);
    cua.emplace(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
  }
  // The provisional best is the Trace with smaller heuristic, i.e. the front one
  Trace best = cua.front();

  // This loop will realize the tree search and get the longest (& best) path.
  // The loop will stop if the tree search is completed or a time limit has
  // been exceeded.
  ros::WallTime searchBeginTime = ros::WallTime::now();
  while (not cua.empty()) {
    if (ros::WallTime::now() - searchBeginTime > ros::WallDuration(params_.max_treeSearch_time)) {
      ROS_WARN("[urinay] Time limit exceeded in tree search.");
      break;
    }
    Trace t = cua.front();
    cua.pop();

    bool trace_at_max_height = false;

    if (t.size() >= params_.max_search_tree_height)
      trace_at_max_height = true;
    else
      this->findNextEdges(nextEdges, &t, midpointsKDT, edges);

    if (trace_at_max_height or nextEdges.empty()) {
      // Means that this trace is finished, should be considered as the "best"
      // trace.
      best = this->computeBestTraceWithFinishedT(best, t);
    } else {
      // Add new possible traces to the queue
      for (const HeurInd &nextEdge : nextEdges) {
        Point actPos = edges[t.edgeInd()].midPoint();
        bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second], &actPos);
        Trace aux = t;
        aux.addEdge(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
        cua.push(aux);
      }
    }
  }
  // The next point will be the FIRST point of the best path
  return best.first().edgeInd();
}

void WayComputer::computeWay(const std::vector<Edge> &edges) {
  // Get rid of all edges from closest to car (included) to last
  this->way_.trimByLocal();

  // Build a k-d tree of all midpoints so is cheaper to find closest points
  std::vector<Point> midpoints(edges.size());
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge &e) -> Point { return e.midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. The tree height will be limited, at that point the first
  // element of the longest with lower heuristic Trace will be added to the
  // way. The search finishes when no element can be added to the way.
  std::vector<HeurInd> nextEdges;

  // Get first set of possible Edges
  this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges);

  // Main outer loop, every iteration of this loop will involve adding one
  // midpoint to the path.
  while (not nextEdges.empty() and ros::ok()) {
    size_t nextEdgeInd = this->treeSearch(nextEdges, midpointsKDT, edges);

    // Append the new Edge
    this->way_.addEdge(edges[nextEdgeInd]);

    // Check for loop closure
    if (this->way_.closesLoop()) {
      this->wayToPublish_ = this->way_.restructureClosure();
      this->isLoopClosed_ = true;
      return;
    }

    // Get next set of possible edges
    this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges);
  }
  this->wayToPublish_ = this->way_;
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params) {
  Way::init(params.way);
}

void WayComputer::stateCallback(const as_msgs::CarState::ConstPtr &data) {
  geometry_msgs::Pose pose;
  pose.position = data->odom.position;
  tf::Quaternion qAux;
  qAux.setRPY(0.0, 0.0, data->odom.heading);
  tf::quaternionTFToMsg(qAux, pose.orientation);
  tf::poseMsgToEigen(pose, this->localTf_);

  this->localTf_ = this->localTf_.inverse();

  this->localTfValid_ = true;
}

void WayComputer::update(TriangleSet &triangulation) {
  if (not this->localTfValid_) {
    ROS_WARN("[urinay] CarState not being received.");
    return;
  }

  // #0: Update last way (this will be used to calculate the raplan flag)
  this->lastWay_ = this->way_;

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

  // Convert this set to a vector
  std::vector<Edge> edgeVec;
  edgeVec.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    edgeVec.push_back(e);
  }

  // #4: Update all local positions (way and edges) with car tf
  this->way_.updateLocal(this->localTf_);
  for (const Edge &e : edgeVec) {
    e.updateLocal(this->localTf_);
  }

  // #5: Perform the search through the midpoints in order to obtain a way.
  this->computeWay(edgeVec);

  // #6: Visualize
  Visualization::getInstance().visualize(edgeSet);
  Visualization::getInstance().visualize(triangulation);
  Visualization::getInstance().visualize(this->wayToPublish_);
}

const bool &WayComputer::isLoopClosed() const {
  return this->isLoopClosed_;
}

void WayComputer::writeWayToFile(const std::string &file_path) const {
  std::ofstream oStreamToWrite(file_path);
  oStreamToWrite << this->wayToPublish_;
  oStreamToWrite.close();
}

std::vector<Point> WayComputer::getPath() const {
  return this->wayToPublish_.getPath();
}

Tracklimits WayComputer::getTracklimits() const {
  return this->wayToPublish_.getTracklimits();
}

as_msgs::PathLimits WayComputer::getPathLimits() const {
  as_msgs::PathLimits res;
  res.stamp = ros::Time::now();

  // res.replan indicates if the Way is different from last iteration's
  res.replan = this->way_ != this->lastWay_;

  // Fill path
  std::vector<Point> path = this->wayToPublish_.getPath();
  res.path.reserve(path.size());
  for (const Point &p : path) {
    res.path.push_back(p.gmPoint());
  }

  // Fill Tracklimits
  Tracklimits tracklimits = this->wayToPublish_.getTracklimits();
  res.tracklimits.stamp = res.stamp;
  res.tracklimits.left.reserve(tracklimits.first.size());
  for (const Node &n : tracklimits.first) {
    res.tracklimits.left.push_back(n.cone());
  }
  for (const Node &n : tracklimits.second) {
    res.tracklimits.right.push_back(n.cone());
  }

  // res.tracklimits.replan indicates if the n midpoints in front of the car
  // have varied from last iteration
  res.tracklimits.replan = this->way_.quinEhLobjetiuDeLaSevaDiresio(this->lastWay_);
  return res;
}