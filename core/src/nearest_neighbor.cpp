/// @file nearest_neighbor.cpp
/// @brief Brute-force and k-d tree nearest-neighbour search.
///
/// Algorithm summary (k-d tree)
/// ----------------------------
/// Every node holds one point and splits space on axis (depth mod d) at that point's
/// coordinate. A query descends first into the side containing the query (most likely to
/// hold the answer), then visits the other side only if the splitting plane is closer than
/// the best distance so far. On a continuous joint the "other side" can also be reached by
/// going around the circle, so the lower bound is the smaller of the two ways.
///
/// Details: docs/ALGORITHMS.md#nearest-neighbour-search
#include "amp/nearest_neighbor.hpp"

#include "amp/cspace.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace amp {

namespace {

/// (distance, id) lexicographic comparison: the tie-break that keeps both strategies identical.
bool better(double d, std::size_t id, double bestD, std::size_t bestId) {
    return d < bestD || (d == bestD && id < bestId);
}

}  // namespace

// ---------------------------------------------------------------------------
// Brute force
// ---------------------------------------------------------------------------

BruteForceNN::BruteForceNN(ArmModel arm) : arm_(std::move(arm)) {}

void BruteForceNN::add(const Config& q) {
    points_.push_back(q);
}

std::size_t BruteForceNN::nearest(const Config& q) const {
    std::size_t best = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const double d = configDistance(arm_, q, points_[i]);
        if (better(d, i, bestDist, best)) {
            best = i;
            bestDist = d;
        }
    }
    return best;
}

std::vector<std::size_t> BruteForceNN::withinRadius(const Config& q, double radius) const {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        if (configDistance(arm_, q, points_[i]) <= radius) {
            out.push_back(i);
        }
    }
    return out;  // already ascending
}

// ---------------------------------------------------------------------------
// k-d tree
// ---------------------------------------------------------------------------

KdTreeNN::KdTreeNN(ArmModel arm) : arm_(std::move(arm)) {}

void KdTreeNN::add(const Config& q) {
    const int id = static_cast<int>(points_.size());
    points_.push_back(q);
    nodes_.push_back(Node{});
    if (id == 0) {
        return;  // root
    }
    // Walk down from the root and attach as a leaf.
    int node = 0;
    int depth = 0;
    const int d = arm_.dof();
    while (true) {
        const int axis = depth % d;
        const auto un = static_cast<std::size_t>(node);
        int& child = q[axis] < points_[un][axis] ? nodes_[un].left : nodes_[un].right;
        if (child < 0) {
            child = id;
            return;
        }
        node = child;
        ++depth;
    }
}

double KdTreeNN::planeLowerBound(const Config& q, int axis, double split, bool farIsRight) const {
    const double direct = farIsRight ? split - q[axis] : q[axis] - split;  // >= 0 by construction
    if (!arm_.limits(axis).continuous) {
        return std::max(0.0, direct);
    }
    // Continuous joint, values in (-pi, pi]. The far side [split, pi] (or [-pi, split)) can also
    // be reached by wrapping through the +/-pi seam, at distance q + pi (or pi - q).
    const double wrapped = farIsRight ? q[axis] + kPi : kPi - q[axis];
    return std::max(0.0, std::min(direct, wrapped));
}

void KdTreeNN::nearestRecursive(int node, int depth, const Config& q, std::size_t& best, double& bestDist) const {
    if (node < 0) {
        return;
    }
    const auto un = static_cast<std::size_t>(node);
    const double d = configDistance(arm_, q, points_[un]);
    if (better(d, un, bestDist, best)) {
        best = un;
        bestDist = d;
    }
    const int axis = depth % arm_.dof();
    const double split = points_[un][axis];
    const bool goLeft = q[axis] < split;
    const int nearChild = goLeft ? nodes_[un].left : nodes_[un].right;
    const int farChild = goLeft ? nodes_[un].right : nodes_[un].left;

    nearestRecursive(nearChild, depth + 1, q, best, bestDist);
    // "<=" (not "<") so an equally distant point with a lower id on the far side is still found.
    if (planeLowerBound(q, axis, split, /*farIsRight=*/goLeft) <= bestDist) {
        nearestRecursive(farChild, depth + 1, q, best, bestDist);
    }
}

std::size_t KdTreeNN::nearest(const Config& q) const {
    std::size_t best = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    nearestRecursive(points_.empty() ? -1 : 0, 0, q, best, bestDist);
    return best;
}

void KdTreeNN::radiusRecursive(int node, int depth, const Config& q, double radius,
                               std::vector<std::size_t>& out) const {
    if (node < 0) {
        return;
    }
    const auto un = static_cast<std::size_t>(node);
    if (configDistance(arm_, q, points_[un]) <= radius) {
        out.push_back(un);
    }
    const int axis = depth % arm_.dof();
    const double split = points_[un][axis];
    const bool goLeft = q[axis] < split;
    radiusRecursive(goLeft ? nodes_[un].left : nodes_[un].right, depth + 1, q, radius, out);
    if (planeLowerBound(q, axis, split, goLeft) <= radius) {
        radiusRecursive(goLeft ? nodes_[un].right : nodes_[un].left, depth + 1, q, radius, out);
    }
}

std::vector<std::size_t> KdTreeNN::withinRadius(const Config& q, double radius) const {
    std::vector<std::size_t> out;
    radiusRecursive(points_.empty() ? -1 : 0, 0, q, radius, out);
    std::sort(out.begin(), out.end());  // deterministic order, identical to brute force
    return out;
}

std::unique_ptr<NearestNeighbor> makeNearestNeighbor(NnStrategy strategy, const ArmModel& arm) {
    if (strategy == NnStrategy::BruteForce) {
        return std::make_unique<BruteForceNN>(arm);
    }
    return std::make_unique<KdTreeNN>(arm);
}

}  // namespace amp
