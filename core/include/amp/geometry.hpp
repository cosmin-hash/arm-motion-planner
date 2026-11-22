/// @file geometry.hpp
/// @brief 2D geometric primitives and distance queries used by collision checking.
///
/// Conventions: all distances are Euclidean, in metres. "Touching" (distance exactly equal
/// to the sum of radii) counts as collision everywhere in this library: for safety, an
/// ambiguous contact is treated as a collision.
///
/// See docs/ALGORITHMS.md#collision-checking.
#pragma once

#include "amp/types.hpp"

#include <vector>

namespace amp {

/// A disc obstacle.
struct Circle {
    Vec2 center = Vec2::Zero();
    double radius = 0.0;
};

/// A convex polygon, stored counter-clockwise.
///
/// The constructor validates convexity and normalises the winding, so every query can rely
/// on "inside = to the left of every edge".
class ConvexPolygon {
public:
    /// @param vertices At least 3 vertices in either winding order, forming a convex polygon.
    ///        Collinear consecutive vertices are allowed.
    /// @throws std::invalid_argument if there are fewer than 3 vertices, the area is ~zero,
    ///         or the polygon is not convex.
    explicit ConvexPolygon(std::vector<Vec2> vertices);

    /// Axis-aligned rectangle helper.
    [[nodiscard]] static ConvexPolygon rectangle(const Vec2& minCorner, const Vec2& maxCorner);

    /// Vertices in counter-clockwise order.
    [[nodiscard]] const std::vector<Vec2>& vertices() const { return vertices_; }
    /// Number of vertices (and edges).
    [[nodiscard]] std::size_t size() const { return vertices_.size(); }
    /// Vertex `i` (index taken modulo size(), so edges are (vertex(i), vertex(i+1))).
    [[nodiscard]] const Vec2& vertex(std::size_t i) const { return vertices_[i % vertices_.size()]; }
    /// Move every vertex by `delta`.
    void translate(const Vec2& delta);
    /// Arithmetic mean of the vertices (a point inside the polygon).
    [[nodiscard]] Vec2 centroid() const;

private:
    std::vector<Vec2> vertices_;
};

/// Axis-aligned rectangular workspace boundary. The arm must stay inside it.
struct Bounds {
    Vec2 min = Vec2::Zero();
    Vec2 max = Vec2::Zero();
};

/// 2D cross product (z component of the 3D cross product): positive if `b` is
/// counter-clockwise from `a`.
[[nodiscard]] inline double cross(const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
}

/// Distance from point `p` to the closed segment [a, b]. Handles a == b.
[[nodiscard]] double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b);

/// True if closed segments [a, b] and [c, d] share at least one point (including touching
/// endpoints and collinear overlap).
[[nodiscard]] bool segmentsIntersect(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);

/// Minimum distance between closed segments [a, b] and [c, d]; zero if they intersect.
[[nodiscard]] double segmentSegmentDistance(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);

/// True if `p` is inside or on the boundary of `poly`.
[[nodiscard]] bool pointInConvexPolygon(const Vec2& p, const ConvexPolygon& poly);

/// Minimum distance between segment [a, b] and the filled polygon; zero if they overlap
/// (including the segment lying entirely inside the polygon).
[[nodiscard]] double segmentPolygonDistance(const Vec2& a, const Vec2& b, const ConvexPolygon& poly);

/// Capsule (segment [a, b] swept by a disc of radius r) versus circle: true if they touch or overlap.
[[nodiscard]] bool capsuleIntersectsCircle(const Vec2& a, const Vec2& b, double r, const Circle& circle);

/// Capsule versus convex polygon: true if they touch or overlap.
[[nodiscard]] bool capsuleIntersectsPolygon(const Vec2& a, const Vec2& b, double r, const ConvexPolygon& poly);

/// Capsule versus capsule: true if they touch or overlap.
[[nodiscard]] bool capsulesIntersect(const Vec2& a, const Vec2& b, double ra, const Vec2& c, const Vec2& d, double rc);

/// True if the capsule lies entirely inside `bounds` (touching the boundary counts as outside).
[[nodiscard]] bool capsuleInsideBounds(const Vec2& a, const Vec2& b, double r, const Bounds& bounds);

}  // namespace amp
