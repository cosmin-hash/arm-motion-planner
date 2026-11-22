/// @file nearest_neighbor.hpp
/// @brief Nearest-neighbour search over configurations: brute force and an incremental k-d tree.
///
/// Sampling planners spend much of their time asking "which tree node is closest to this
/// sample?". Brute force is O(n) per query. A k-d tree is O(log n) on average in low
/// dimensions. Both implement the same interface, so a planner can switch with one parameter,
/// and the tests require them to return *identical* answers (ties broken by the lower index),
/// which keeps planning deterministic regardless of the strategy.
///
/// Distances use configDistance(), including wraparound on continuous joints.
///
/// See docs/ALGORITHMS.md#nearest-neighbour-search.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/types.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace amp {

/// Which nearest-neighbour implementation to use.
enum class NnStrategy { BruteForce, KdTree };

/// Abstract nearest-neighbour index. Points are identified by insertion order (0, 1, 2, ...).
class NearestNeighbor {
public:
    virtual ~NearestNeighbor() = default;

    /// Insert a point; its id is the number of points inserted before it.
    virtual void add(const Config& q) = 0;
    /// Id of the closest point to `q` (lowest id on ties). @pre size() > 0
    [[nodiscard]] virtual std::size_t nearest(const Config& q) const = 0;
    /// Ids of all points within distance `radius` of `q` (inclusive), sorted ascending.
    [[nodiscard]] virtual std::vector<std::size_t> withinRadius(const Config& q, double radius) const = 0;
    /// Number of points stored.
    [[nodiscard]] virtual std::size_t size() const = 0;
};

/// Linear scan. Simple, obviously correct: the reference the k-d tree is tested against.
class BruteForceNN final : public NearestNeighbor {
public:
    /// Empty index for configurations of `arm` (used for its distance metric).
    explicit BruteForceNN(ArmModel arm);
    /// Insert a point; see NearestNeighbor::add.
    void add(const Config& q) override;
    /// Closest point id; see NearestNeighbor::nearest.
    [[nodiscard]] std::size_t nearest(const Config& q) const override;
    /// Ids within a radius; see NearestNeighbor::withinRadius.
    [[nodiscard]] std::vector<std::size_t> withinRadius(const Config& q, double radius) const override;
    /// Number of points stored.
    [[nodiscard]] std::size_t size() const override { return points_.size(); }

private:
    ArmModel arm_;
    std::vector<Config> points_;
};

/// Incremental (insert-only) k-d tree.
///
/// Each node splits on axis depth % d at its own point. Insertion is O(depth). Random
/// sample order (as in RRT) keeps the expected depth O(log n) without rebalancing.
/// Continuous joints are handled by a wrap-aware lower bound on the distance to the far
/// side of each splitting plane.
class KdTreeNN final : public NearestNeighbor {
public:
    /// Empty index for configurations of `arm` (used for its distance metric).
    explicit KdTreeNN(ArmModel arm);
    /// Insert a point; see NearestNeighbor::add.
    void add(const Config& q) override;
    /// Closest point id; see NearestNeighbor::nearest.
    [[nodiscard]] std::size_t nearest(const Config& q) const override;
    /// Ids within a radius; see NearestNeighbor::withinRadius.
    [[nodiscard]] std::vector<std::size_t> withinRadius(const Config& q, double radius) const override;
    /// Number of points stored.
    [[nodiscard]] std::size_t size() const override { return points_.size(); }

private:
    /// Child links of one tree node (node i stores points_[i]).
    struct Node {
        int left = -1;   ///< Child with coordinate < split value.
        int right = -1;  ///< Child with coordinate >= split value.
    };

    /// Smallest possible distance along `axis` from `q` to any point on the far side of the
    /// plane x[axis] = split (accounts for wraparound on continuous joints).
    [[nodiscard]] double planeLowerBound(const Config& q, int axis, double split, bool farIsRight) const;
    void nearestRecursive(int node, int depth, const Config& q, std::size_t& best, double& bestDist) const;
    void radiusRecursive(int node, int depth, const Config& q, double radius, std::vector<std::size_t>& out) const;

    ArmModel arm_;
    std::vector<Config> points_;  // node i stores points_[i]
    std::vector<Node> nodes_;
};

/// Create an index of the requested kind for configurations of `arm`.
[[nodiscard]] std::unique_ptr<NearestNeighbor> makeNearestNeighbor(NnStrategy strategy, const ArmModel& arm);

}  // namespace amp
