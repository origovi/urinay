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

double WayComputer::getHeuristic(const Point &actPos, const Edge &nextEdge, const Vector &actDir, const Trace *actTrace, const Params::WayComputer::Search &params) const {
  const Point &nextPos = nextEdge.midPoint();
  Vector nextDir(actPos, nextPos);

  // 1. Angle Heuristic:
  // This heuristic is given by the sqrt func, the range of abs(angle)/(PI/2)
  // is [0,1], because points should not have an abs(angle) > PI/2 (or max_angle_diff).
  // In practise, we can think that the input of sqrt does not exceed 0.5.
  // Thus the range of angleHeur is in [0,0.7]
  double angle_rad = abs(nextDir.angleWith(actDir));
  double angleHeur = (1 - params.heur_dist_weight) * sqrt(angle_rad / M_PI_2);    // Range in [0,1], 0->best, 1->worst

  // 2. Distance Heuristic
  double distHeur = params.heur_dist_weight * Point::dist(actPos, nextPos);  // Dist in [sqrt(min_distSq_between_midpoints), search_radius]

  // 3. Track width heuristic
  double trackWidthDiff = actTrace ? abs(nextEdge.trackWidth(nextDir) - actTrace->avgTrackWidth()) / actTrace->avgTrackWidth() : 0.0;
  double trackWidthDiffHeur = params.heur_track_width_diff_weight * trackWidthDiff;

  // FINAL HEURISTIC VALUE:
  // This value is a linear interpolation between distance and angle heuristics
  // plus a weighted track width difference heuristic.
  double heur = distHeur + angleHeur + trackWidthDiffHeur;

  // FOR DEBUG
  // std::cout << " a: " << angle_rad << " ah: " << angleHeur;
  // std::cout << " dh: " << distHeur;
  // std::cout << " twd: " << trackWidthDiff << " twdh: " << trackWidthDiffHeur;
  // std::cout << " h: " << heur << std::endl;
  return heur;
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

  // Set actual dir vector
  Vector actDir;
  if (actTrace and actTrace->size() >= 2)
    actDir = Vector(lastPos, actPos);
  else
    actDir = Vector(1, 0);

  // Find all possible edges in a specified radius
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params.search_radius);

  // Discard edges by specifications
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {
    const Edge &nextPossibleEdge = edges[*it];
    const Vector nextDir(actPos, nextPossibleEdge.midPoint());
    // The currently-iterated Edge will be removed if any of the below conditions is true
    bool removeConditions = actEdge and (
      // 1. Remove itself from being the next one
      nextPossibleEdge == *actEdge or

      // 2. Remove any edge whose distance with the last is too small
      Point::distSq(actPos, nextPossibleEdge.midPoint()) < params.min_distSq_between_midpoints or

      // 3. Remove any edge whose midpoint create an angle too closed with last one
      abs(actDir.angleWith(Vector(actPos, nextPossibleEdge.midPoint()))) > params.max_angle_diff or

      // 4. [Only before appending the edge that closes the loop] Remove any edge that is already contained in the path but is not the one that closes the loop
      (!actTrace->isLoopClosed() and !actTrace->closesLoopWith(nextPossibleEdge) and actTrace->containsEdge(nextPossibleEdge)) or

      // 5. Remove any edge whose midpoint and lastPos are in the same side of actEdge (avoid bouncing on a track limit)
      (actTrace->size() >= 2 and Vector::pointBehind(actEdge->midPoint(), lastPos, actEdge->normal()) == Vector::pointBehind(actEdge->midPoint(), nextPossibleEdge.midPoint(), actEdge->normal())) or

      // 6. Remove any edge whose track width is too big or too small compared to the average track width
      (nextPossibleEdge.trackWidth(nextDir) < params.min_track_width) or

      // 7. [If not allow_intersection, only before closing the loop] Remove any Edge which appended would create an intersection
      (!params.allow_intersection and !actTrace->isLoopClosed() and actTrace->intersectsWith(nextPossibleEdge)));

    if (removeConditions) {
      it = nextPossibleEdges.erase(it);
    }
    else
      it++;
  }

  // Get the heuristics for all possible next edges and filter them (only the
  // ones having a heuristic small enough will prevail)
  std::vector<HeurInd> privilege_runner;
  privilege_runner.reserve(nextPossibleEdges.size());
  for (const size_t &nextPossibleEdgeInd : nextPossibleEdges) {
    // FOR DEBUG
    // std::cout << "FROM: ";
    // if (actEdge) std::cout << *actEdge;
    // else std::cout << "NONE";
    // std::cout << " TO: " << edges[nextPossibleEdgeInd];
    double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd], actDir, actTrace, params);
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
      traceBuffer.emplace_back(edges[nextEdge.second], nextEdge.first);
    }
  }

  // This loop will realize one tree search step, i.e. extend each Trace
  // by one and add the extensions to the 'queue' list.
  bool didSth = false;
  auto it_buff = traceBuffer.begin();
  while (it_buff != traceBuffer.end()) {
    Trace &t = *it_buff;  // Reference to current iterated trace

    this->findNextEdges(nextEdges, &t, midpointsKDT, edges, params);

    for (const HeurInd &nextEdge : nextEdges) {
      traceBuffer.emplace(it_buff, edges[nextEdge.second], nextEdge.first, t);
    }

    if (!nextEdges.empty()) {
      it_buff = traceBuffer.erase(it_buff);
      didSth = true;
    } else it_buff++;
  }
  return didSth;
}

void WayComputer::computeWay(const std::vector<Edge> &edges, const Params::WayComputer::Search &params) {
  // Get rid of all edges from closest to car (included) to last
  this->way_.trimByLocal(this->localTf_.translation());

  if (this->way_.isLoopClosed()) {
    ROS_WARN("[urinay] Loop already closed, this should only happen if the "
             "car is just passing by the starting point and the closest "
             "midpoint is the one closing the loop, hence no computation "
             "is needed.");
    this->isLoopClosed_ = true;
    this->wayToPublish_ = this->way_.restructureClosure();
    return;
  }

  // Build a k-d tree of all midpoints so is cheaper to find closest points
  std::vector<Point> midpoints(edges.size());
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge &e) -> Point { return e.midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. All traces will be stored in a trace buffer and only the
  // n best traces will be maintained.

  // Main outer loop, every iteration of this loop will involve adding one
  // midpoint to the path.
  TraceBuffer traceBuffer(params);
  if (!this->way_.empty()) traceBuffer.push_back(this->way_);
  while (ros::ok() and (!params.max_way_horizon_size or traceBuffer.bestTrace().sizeAheadOfCar() <= params.max_way_horizon_size)) {
    // Perform tree search and break the loop if no next edges are found
    bool canContinue = this->treeSearch(traceBuffer, midpointsKDT, edges, params);
    
    // Stop search because no more Traces can be extended
    if (!canContinue) break;

    // Stop search becayse loop has been closed with enough confidence
    if (traceBuffer.bestTrace().connectionsSinceLoopClosed() >= params.extra_tree_height_closure + 1) {
      break;
    }

    // Prune buffer
    traceBuffer.prune();
  }
  // Now append best trace to way
  this->way_ = traceBuffer.bestTrace().trimLoopClosure();
  this->isLoopClosed_ = this->way_.isLoopClosed();
  this->wayToPublish_ = this->isLoopClosed_ ? this->way_.restructureClosure() : this->way_;
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params) {
  Trace::init(params.way);
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