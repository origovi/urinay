/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 *
 * It is a reimplementation of the C code using C++. It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 *
 */

#include "utils/KDTree.hpp"

KDNode::KDNode() = default;

KDNode::KDNode(const Point &pt, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_) {
  x = pt;
  index = idx_;
  left = left_;
  right = right_;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_) {
  x = pi.first;
  index = pi.second;
  left = left_;
  right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return x->at(idx); }
KDNode::operator bool() const { return bool(index); }
KDNode::operator size_t() { return *index; }
KDNode::operator pointIndex() { return pointIndex(*x, *index); }

KDNodePtr NewKDNodePtr() {
  KDNodePtr mynode = std::make_shared<KDNode>();
  return mynode;
}

inline double dist2(const Point &a, const Point &b) {
  double distc = 0;
  for (size_t i = 0; i < a.size(); i++) {
    double di = a.at(i) - b.at(i);
    distc += di * di;
  }
  return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b) {
  return dist2(*(a->x), *(b->x));
}

inline double dist(const Point &a, const Point &b) {
  return std::sqrt(dist2(a, b));
}

inline double dist(const KDNodePtr &a, const KDNodePtr &b) {
  return std::sqrt(dist2(a, b));
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx(const pointIndex &a,  //
                                  const pointIndex &b   //
) {
  return (a.first.at(idx) < b.first.at(idx));  //
}

inline void sort_on_idx(const pointIndexArr::iterator &begin,  //
                        const pointIndexArr::iterator &end,    //
                        size_t idx) {
  comparer comp(idx);
  comp.idx = idx;

  using std::placeholders::_1;
  using std::placeholders::_2;

  std::nth_element(begin, begin + std::distance(begin, end) / 2,
                   end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

using pointVec = std::vector<Point>;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
                            const pointIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
  if (begin == end) {
    return NewKDNodePtr();  // empty tree
  }

  size_t dim = begin->first.size();

  if (length > 1) {
    sort_on_idx(begin, end, level);
  }

  auto middle = begin + (length / 2);

  auto l_begin = begin;
  auto l_end = middle;
  auto r_begin = middle + 1;
  auto r_end = end;

  size_t l_len = length / 2;
  size_t r_len = length - l_len - 1;

  KDNodePtr left;
  if (l_len > 0 && dim > 0) {
    left = make_tree(l_begin, l_end, l_len, (level + 1) % dim);
  } else {
    left = leaf;
  }
  KDNodePtr right;
  if (r_len > 0 && dim > 0) {
    right = make_tree(r_begin, r_end, r_len, (level + 1) % dim);
  } else {
    right = leaf;
  }

  // KDNode result = KDNode();
  return std::make_shared<KDNode>(*middle, left, right);
}

KDTree::KDTree(const pointVec &point_array) {
  leaf = std::make_shared<KDNode>();
  // iterators
  pointIndexArr arr;
  for (size_t i = 0; i < point_array.size(); i++) {
    arr.push_back(pointIndex(point_array.at(i), i));
  }

  auto begin = arr.begin();
  auto end = arr.end();

  size_t length = arr.size();
  size_t level = 0;  // starting

  root = KDTree::make_tree(begin, end, length, level);
}

KDTree::KDTree(const std::list<Point> &point_list) {
  leaf = std::make_shared<KDNode>();
  // iterators
  pointIndexArr arr;
  int i = 0;
  for (std::list<Point>::const_iterator it = point_list.begin(); it != point_list.end(); it++) {
    arr.push_back(pointIndex(*it, i++));
  }

  auto begin = arr.begin();
  auto end = arr.end();

  size_t length = arr.size();
  size_t level = 0;  // starting

  root = KDTree::make_tree(begin, end, length, level);
}

KDNodePtr KDTree::nearest_(   //
    const KDNodePtr &branch,  //
    const Point &pt,          //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist,  //
    const std::set<size_t> &excs) const {
  double d, dx, dx2;

  if (!bool(*branch)) {
    return NewKDNodePtr();  // basically, null
  }

  const Point &branch_pt = *(branch->x);
  size_t dim = branch_pt.size();

  d = dist2(branch_pt, pt);
  dx = branch_pt.at(level) - pt.at(level);
  dx2 = dx * dx;

  KDNodePtr best_l = best;
  double best_dist_l = best_dist;

  if ((!bool(*best_l) or d < best_dist) and excs.find(*(branch->index)) == excs.end()) {
    best_dist_l = d;
    best_l = branch;
  }

  size_t next_lv = (level + 1) % dim;
  KDNodePtr section;
  KDNodePtr other;

  // select which branch makes sense to check
  if (dx > 0) {
    section = branch->left;
    other = branch->right;
  } else {
    section = branch->right;
    other = branch->left;
  }

  // keep nearest neighbor from further down the tree
  KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l, excs);
  double dl = dist2(*(further->x), pt);
  if (bool(*further) and (dl < best_dist_l or !bool(*best_l)) and excs.find(*(further->index)) == excs.end()) {
    best_dist_l = dl;
    best_l = further;
  }
  // only check the other branch if it makes sense to do so
  if (!bool(*best_l) or dx2 < best_dist_l) {
    further = nearest_(other, pt, next_lv, best_l, best_dist_l, excs);
    double dl = dist2(*(further->x), pt);
    if (bool(*further) and (dl < best_dist_l or !bool(*best_l)) and excs.find(*(further->index)) == excs.end()) {
      best_dist_l = dl;
      best_l = further;
    }
  }

  return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const Point &pt, const std::set<size_t> &excs) const {
  size_t level = 0;
  // KDNodePtr best = branch;
  double branch_dist = dist2(*(root->x), pt);
  return nearest_(root,   // beginning of tree
                  pt,     // point we are querying
                  level,  // start from level 0
                  (excs.find(*(root->index)) == excs.end()) ? root : NewKDNodePtr(),
                  branch_dist,  // best_dist = branch_dist
                  excs);
};

KDTData<Point> KDTree::nearest_point(const Point &pt, const std::set<size_t> &excs) const {
  return nearest_(pt, excs)->x;
};

KDTData<size_t> KDTree::nearest_index(const Point &pt, const std::set<size_t> &excs) const {
  return nearest_(pt, excs)->index;
};

pointIndexV KDTree::nearest_pointIndex(const Point &pt, const std::set<size_t> &excs) const {
  KDNodePtr Nearest = nearest_(pt, excs);
  return (bool(*Nearest)) ? pointIndexV(pointIndex(*Nearest->x, *Nearest->index)) : pointIndexV();
}

pointIndexArr KDTree::neighborhood_(  //
    const KDNodePtr &branch,          //
    const Point &pt,                  //
    const double &rad,                //
    const size_t &level               //
) const {
  double d, dx, dx2;
  if (!bool(*branch)) {
    // branch has no point, means it is a leaf,
    // no points to add
    return pointIndexArr();
  }

  size_t dim = pt.size();

  double r2 = rad * rad;
  d = dist2(*(branch->x), pt);
  dx = branch->x->at(level) - pt.at(level);
  dx2 = dx * dx;

  pointIndexArr nbh, nbh_s, nbh_o;
  if (d <= r2) {
    nbh.push_back(pointIndex(*branch));
  }

  KDNodePtr section;
  KDNodePtr other;
  if (dx > 0) {
    section = branch->left;
    other = branch->right;
  } else {
    section = branch->right;
    other = branch->left;
  }

  nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
  nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
  if (dx2 < r2) {
    nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
    nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
  }

  return nbh;
};

pointIndexArr KDTree::neighborhood(  //
    const Point &pt,                 //
    const double &rad) const {
  size_t level = 0;
  return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(  //
    const Point &pt,                   //
    const double &rad) const {
  size_t level = 0;
  pointIndexArr nbh = neighborhood_(root, pt, rad, level);
  pointVec nbhp;
  nbhp.resize(nbh.size());
  std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                 [](pointIndex x) { return x.first; });
  return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
    const Point &pt,                    //
    const double &rad) const {
  size_t level = 0;
  pointIndexArr nbh = neighborhood_(root, pt, rad, level);
  indexArr nbhi;
  nbhi.resize(nbh.size());
  std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                 [](pointIndex x) { return x.second; });
  return nbhi;
}
