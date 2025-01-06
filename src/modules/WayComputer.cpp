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
      if (e.len > this->params_.max_triangle_edge_len) {
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
    if (bool(nearestCC) and Point::distSq(circums[*nearestCC], midPoint) > pow(this->params_.max_dist_circum_midPoint, 2)) {
      it = edges.erase(it);
    } else {
      it++;
    }
  }
}

double WayComputer::getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir, const Params::WayComputer::Search &params) const {
  // 1. Angle Heuristic
  double angle = Vector(actPos, nextPos).angleWith(dir);
  double angleHeur = sqrt(abs(angle) / M_PI_2);    // Range in [0, 1], 0->best, 1->worst

  // 2. Distance Heuristic
  double distHeur = Point::dist(actPos, nextPos);  // Range of meters, likely in [0.5, 5]

  // 3. Edge len heuristic
  // double edgeLenHeur = 

  return params.heur_dist_ponderation * distHeur + (1 - params.heur_dist_ponderation) * angleHeur;
}

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Trace *actTrace, const KDTree &midpointsKDT, const std::vector<Edge> &edges, const Params::WayComputer::Search &params) const {
  nextEdges.clear();

  const Edge *actEdge = nullptr;  // Last valid Edge (starting point)
  Point actPos(0, 0);
  Point lastPos(0, 0);

  // Set actual and last position
  if (actTrace and not actTrace->empty()) {
    actEdge = &actTrace->edge();
    actPos = actEdge->midPoint();
    if (actTrace->size() >= 2) {
      lastPos = actTrace->before().edge().midPoint();
    }
  }
  if (not this->way_.empty()) {
    if (not actEdge) {
      actEdge = &this->way_.back();
      actPos = actEdge->midPoint();
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
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params.search_radius);

  // Discard edges by specifications
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {
    const Edge &nextPossibleEdge = edges[*it];
    // The currently-iterated Edge will be removed if any of the below conditions is true
    bool removeConditions = actEdge and (
      // 1. Remove itself from being the next one
      nextPossibleEdge == *actEdge or

      // 2. Remove any edge whose distance with the last is too small
      // Point::distSq(actPos, nextPossibleEdge.midPoint()) < params.min_distSq_between_midpoints or

      // 3. Remove any edge whose midpoint create an angle too closed with last one
      abs(dir.angleWith(Vector(actPos, nextPossibleEdge.midPoint()))) > params.max_angle_diff or

      // 4. [Only before appending the edge that closes the loop] Remove any edge that is already contained in the path but is not the one that closes the loop
      (not this->way_.closesLoopWith(nextPossibleEdge, actTrace) and (not actTrace or not actTrace->isLoopClosed()) and ((actTrace and actTrace->containsEdge(nextPossibleEdge)) or this->way_.containsEdge(nextPossibleEdge))) or

      // 5. Remove any edge whose midpoint and lastPos are in the same side of actEdge (avoid bouncing on a track limit)
      ((this->way_.size() >= 2 or actTrace) and Vector::pointBehind(actEdge->midPoint(), lastPos, actEdge->normal()) == Vector::pointBehind(actEdge->midPoint(), nextPossibleEdge.midPoint(), actEdge->normal())) or

      // 6. Remove any edge whose length is too big or too small compared to the average edge length of the way
      // (actTrace and (nextPossibleEdge.len < (1 - params.edge_len_diff_factor) * actTrace->avgEdgeLen() or nextPossibleEdge.len > (1 + params.edge_len_diff_factor) * actTrace->avgEdgeLen())) or

      // 7. [If not allow_intersection, only before closing the loop] Remove any Edge which appended would create an intersection
      (not params.allow_intersection and (not actTrace or not actTrace->isLoopClosed()) and not this->way_.closesLoopWith(nextPossibleEdge, actTrace) and this->way_.intersectsWith(nextPossibleEdge, actTrace)));

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
    double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd].midPoint(), dir, params);
    if (heuristic <= params.max_next_heuristic) privilege_runner.emplace_back(heuristic, nextPossibleEdgeInd);
  }

  // Copy the n best HeurInd(s) into the nextEdges vector, according to
  // cppreference.com, this is O(nlogn)
  nextEdges.resize(std::min(privilege_runner.size(), (size_t)params.max_search_options));
  std::partial_sort_copy(privilege_runner.begin(), privilege_runner.end(), nextEdges.begin(), nextEdges.end());
}

bool WayComputer::treeSearch(TraceBuffer &traceBuffer, const KDTree &midpointsKDT, const std::vector<Edge> &edges, const Params::WayComputer::Search &params) {
  std::vector<HeurInd> nextEdges;
  
  if (traceBuffer.empty()) {
    // Get first set of possible Edges
    this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);
    if (nextEdges.empty()) return false;
    for (const HeurInd &nextEdge : nextEdges) {
      bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second]);
      traceBuffer.emplace_back(edges[nextEdge.second], nextEdge.first, closesLoop);
    }
  }

  // // This loop will realize the tree search and get the longest (& best) path.
  // // The loop will stop if the tree search is completed or a time limit has
  // // been exceeded.
  // ros::WallTime searchBeginTime = ros::WallTime::now();
  bool didSth = false;
  auto it_buff = traceBuffer.begin();
  std::cout << "size " << traceBuffer.size() << std::endl;
  while (it_buff != traceBuffer.end()) {
    Trace &t = *it_buff;  // Reference to current iterated trace

    this->findNextEdges(nextEdges, &t, midpointsKDT, edges, params);

    for (const HeurInd &nextEdge : nextEdges) {
      const Edge &e = edges[nextEdge.second];
      traceBuffer.emplace(it_buff, e, nextEdge.first, this->way_.closesLoopWith(e, &t), t);
    }

    if (!nextEdges.empty()) {
      it_buff = traceBuffer.erase(it_buff);
      didSth = true;
    } else it_buff++;
  }
  return didSth;

  /*
  while (not cua.empty()) {
    if (ros::WallTime::now() - searchBeginTime > ros::WallDuration(params.max_treeSearch_time)) {
      ROS_WARN("[urinay] Time limit exceeded in tree search.");
      // Save current search state to buffer & break
      while (not cua.empty()) {
        this->treeBuffer_.insert(cua.front());
        cua.pop();
      }
      best = this->treeBuffer_.getBestTrace();
      break;
    }

    Trace t = cua.front();
    cua.pop();

    bool trace_at_max_height = false;

    if (t.size() >= params.max_search_tree_height)
      trace_at_max_height = true;
    else
      this->findNextEdges(nextEdges, &t, midpointsKDT, edges, params);

    if (trace_at_max_height or nextEdges.empty()) {
      // Means that this trace is finished, should be considered as the "best"
      // trace AND added to tree buffer.
      this->treeBuffer_.insert(t);
      best = t < best ? t : best;  // See Trace::operator<() for more info on criteria
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
  size_t nextEdgeInd = best.first().edgeInd();
  this->treeBuffer_.update(nextEdgeInd);
  return {true, nextEdgeInd};
  */
}

void WayComputer::computeWay(const std::vector<Edge> &edges, const Params::WayComputer::Search &params) {
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

  // Main outer loop, every iteration of this loop will involve adding one
  // midpoint to the path.
  TraceBuffer traceBuffer(params);
  while (ros::ok() and (!params.max_way_horizon_size or traceBuffer.height() <= params.max_way_horizon_size)) {
    // Perform tree search and break the loop if no next edges are found
    // std::cout << "NEW ITER" << std::endl;
    bool canContinue = this->treeSearch(traceBuffer, midpointsKDT, edges, params);
    if (not canContinue) break;

    // // Append the new Edge
    // this->way_.addEdge(edges[nextEdgeIndP.second]);

    // Check for loop closure
    // if (this->way_.closesLoop()) {
    //   this->wayToPublish_ = this->way_.restructureClosure();
    //   this->isLoopClosed_ = true;
    //   return;
    // }

    // Prune buffer
    traceBuffer.prune();

    traceBuffer.newStep();
  }
  this->way_.addTrace(traceBuffer.bestTrace());
  this->isLoopClosed_ = false;
  this->wayToPublish_ = this->way_;
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params) {
  Way::init(params.way);
  this->generalFailsafe_.initGeneral(this->params_.search, this->params_.general_failsafe_safetyFactor, this->params_.failsafe_max_way_horizon_size);
}

void WayComputer::stateCallback(const nav_msgs::Odometry::ConstPtr &data) {
  tf::poseMsgToEigen(data->pose.pose, this->localTf_);

  this->localTf_ = this->localTf_.inverse();

  this->localTfValid_ = true;
}

void WayComputer::update(TriangleSet &triangulation, const std_msgs::Header &header) {
  if (not this->localTfValid_) {
    ROS_WARN("[urinay] CarState not being received.");
    return;
  }

  // #0: Update last way (this will be used to calculate the raplan flag).
  //     And update stamp.
  this->lastWay_ = this->way_;
  this->lastHeader_ = header;

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

  // #5: Perform the search through the midpoints in order to obtain a way
  //     using normal parameters.
  this->computeWay(edgeVec, this->params_.search);

  // #6: Check failsafe(s)
  if (this->params_.general_failsafe and this->way_.sizeAheadOfCar() < MIN_FAILSAFE_WAY_SIZE and !this->isLoopClosed_) {
    ROS_WARN("[urinay] GENERAL FAILSAFE ACTIVATED!");
    this->computeWay(edgeVec, this->generalFailsafe_);
  }

  // #7: Visualize
  Visualization::getInstance().setHeader(header);
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

const bool &WayComputer::isLocalTfValid() const {
  return this->localTfValid_;
}

const Eigen::Affine3d &WayComputer::getLocalTf() const {
  return this->localTf_;
}

std::vector<Point> WayComputer::getPath() const {
  return this->wayToPublish_.getPath();
}

Tracklimits WayComputer::getTracklimits() const {
  return this->wayToPublish_.getTracklimits();
}

custom_msgs::PathLimits WayComputer::getPathLimits() const {
  custom_msgs::PathLimits res;
  res.header = this->lastHeader_;

  // res.replan indicates if the Way is different from last iteration's
  res.new_path = this->way_ != this->lastWay_;

  // res.tracklimits.replan indicates if the n midpoints in front of the car
  // have varied from last iteration
  res.new_close_midpoints = this->way_.vitalMidpointsChanged(this->lastWay_);

  // Fill path
  std::vector<Point> path = this->wayToPublish_.getPath();
  res.path.reserve(path.size());
  for (const Point &p : path) {
    res.path.push_back(p.gmPoint());
  }

  // Fill Tracklimits
  Tracklimits tracklimits = this->wayToPublish_.getTracklimits();
  res.tracklimits_left.reserve(tracklimits.first.size());
  for (const Node &n : tracklimits.first) {
    res.tracklimits_left.push_back(n.point().gmPoint());
  }
  res.tracklimits_right.reserve(tracklimits.second.size());
  for (const Node &n : tracklimits.second) {
    res.tracklimits_right.push_back(n.point().gmPoint());
  }

  return res;
}