/// @file geometry.cpp
/// @brief 2D distance and intersection queries.
///
/// Algorithm summary
/// -----------------
/// Every arm-versus-world test reduces to a few exact primitives:
/// * point-to-segment distance (project, clamp to the segment, measure);
/// * segment-segment intersection via orientation signs, and distance as the minimum of the
///   four endpoint-to-segment distances when they do not intersect;
/// * point-in-convex-polygon: inside iff on the left of every counter-clockwise edge.
/// A capsule of radius r hits a shape iff the distance from its core segment to the shape
/// is <= r. That is why links are modelled as capsules: one distance query per link.
///
/// Details: docs/ALGORITHMS.md#collision-checking
#include "amp/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace amp {

namespace {

/// Sign of the turn a -> b -> c with a tolerance relative to the segment lengths:
/// +1 counter-clockwise, -1 clockwise, 0 collinear.
int orientation(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double value = cross(b - a, c - a);
    const double scale = std::max({(b - a).squaredNorm(), (c - a).squaredNorm(), 1e-300});
    constexpr double kRelEps = 1e-12;
    if (value > kRelEps * scale) {
        return 1;
    }
    if (value < -kRelEps * scale) {
        return -1;
    }
    return 0;
}

/// For collinear a, b, p: is p within the bounding box of [a, b]?
bool onSegmentCollinear(const Vec2& a, const Vec2& b, const Vec2& p) {
    return p.x() <= std::max(a.x(), b.x()) && p.x() >= std::min(a.x(), b.x()) && p.y() <= std::max(a.y(), b.y()) &&
           p.y() >= std::min(a.y(), b.y());
}

double signedArea(const std::vector<Vec2>& v) {
    double twice = 0.0;
    for (std::size_t i = 0; i < v.size(); ++i) {
        twice += cross(v[i], v[(i + 1) % v.size()]);
    }
    return 0.5 * twice;
}

}  // namespace

// ---------------------------------------------------------------------------
// ConvexPolygon
// ---------------------------------------------------------------------------

ConvexPolygon::ConvexPolygon(std::vector<Vec2> vertices) : vertices_(std::move(vertices)) {
    if (vertices_.size() < 3) {
        throw std::invalid_argument("ConvexPolygon: need at least 3 vertices");
    }
    const double area = signedArea(vertices_);
    if (std::abs(area) < 1e-12) {
        throw std::invalid_argument("ConvexPolygon: polygon has zero area");
    }
    if (area < 0.0) {
        std::reverse(vertices_.begin(), vertices_.end());  // normalise to counter-clockwise
    }
    // Convex and simple iff every turn is a left turn (or straight).
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        if (orientation(vertex(i), vertex(i + 1), vertex(i + 2)) < 0) {
            throw std::invalid_argument("ConvexPolygon: polygon is not convex");
        }
    }
    // A star-shaped "polygon" can have only left turns yet wind around twice. For a simple
    // convex polygon, the exterior angles sum to exactly 2*pi.
    double turning = 0.0;
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        const Vec2 e1 = vertex(i + 1) - vertex(i);
        const Vec2 e2 = vertex(i + 2) - vertex(i + 1);
        turning += std::atan2(cross(e1, e2), e1.dot(e2));
    }
    if (std::abs(turning - kTwoPi) > 1e-6) {
        throw std::invalid_argument("ConvexPolygon: polygon is self-intersecting");
    }
}

ConvexPolygon ConvexPolygon::rectangle(const Vec2& minCorner, const Vec2& maxCorner) {
    return ConvexPolygon(
        {minCorner, Vec2(maxCorner.x(), minCorner.y()), maxCorner, Vec2(minCorner.x(), maxCorner.y())});
}

void ConvexPolygon::translate(const Vec2& delta) {
    for (Vec2& v : vertices_) {
        v += delta;
    }
}

Vec2 ConvexPolygon::centroid() const {
    Vec2 sum = Vec2::Zero();
    for (const Vec2& v : vertices_) {
        sum += v;
    }
    return sum / static_cast<double>(vertices_.size());
}

// ---------------------------------------------------------------------------
// Distance and intersection queries
// ---------------------------------------------------------------------------

double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.squaredNorm();
    if (len2 == 0.0) {
        return (p - a).norm();  // degenerate segment
    }
    // Parameter of the orthogonal projection of p onto the infinite line, clamped to [0, 1].
    const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

bool segmentsIntersect(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const int o1 = orientation(a, b, c);
    const int o2 = orientation(a, b, d);
    const int o3 = orientation(c, d, a);
    const int o4 = orientation(c, d, b);

    // General case: each segment's endpoints lie on opposite sides of the other segment.
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    // Special cases: an endpoint lies exactly on the other segment.
    return (o1 == 0 && onSegmentCollinear(a, b, c)) || (o2 == 0 && onSegmentCollinear(a, b, d)) ||
           (o3 == 0 && onSegmentCollinear(c, d, a)) || (o4 == 0 && onSegmentCollinear(c, d, b));
}

double segmentSegmentDistance(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    if (segmentsIntersect(a, b, c, d)) {
        return 0.0;
    }
    // In 2D, the closest points of two non-intersecting segments always include an endpoint.
    return std::min({pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d), pointSegmentDistance(c, a, b),
                     pointSegmentDistance(d, a, b)});
}

bool pointInConvexPolygon(const Vec2& p, const ConvexPolygon& poly) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
        if (cross(poly.vertex(i + 1) - poly.vertex(i), p - poly.vertex(i)) < 0.0) {
            return false;  // strictly right of a counter-clockwise edge: outside
        }
    }
    return true;
}

double segmentPolygonDistance(const Vec2& a, const Vec2& b, const ConvexPolygon& poly) {
    // Segment fully or partly inside: an endpoint is inside. (Testing only the edges would
    // miss a segment lying entirely within the polygon.)
    if (pointInConvexPolygon(a, poly) || pointInConvexPolygon(b, poly)) {
        return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < poly.size(); ++i) {
        best = std::min(best, segmentSegmentDistance(a, b, poly.vertex(i), poly.vertex(i + 1)));
        if (best == 0.0) {
            break;
        }
    }
    return best;
}

bool capsuleIntersectsCircle(const Vec2& a, const Vec2& b, double r, const Circle& circle) {
    return pointSegmentDistance(circle.center, a, b) <= r + circle.radius;
}

bool capsuleIntersectsPolygon(const Vec2& a, const Vec2& b, double r, const ConvexPolygon& poly) {
    return segmentPolygonDistance(a, b, poly) <= r;
}

bool capsulesIntersect(const Vec2& a, const Vec2& b, double ra, const Vec2& c, const Vec2& d, double rc) {
    return segmentSegmentDistance(a, b, c, d) <= ra + rc;
}

bool capsuleInsideBounds(const Vec2& a, const Vec2& b, double r, const Bounds& bounds) {
    // A capsule is the convex hull of two discs, and a rectangle is convex, so the capsule
    // is inside iff both end discs are inside (strictly, since touching counts as contact).
    const auto discInside = [&](const Vec2& p) {
        return p.x() - r > bounds.min.x() && p.x() + r < bounds.max.x() && p.y() - r > bounds.min.y() &&
               p.y() + r < bounds.max.y();
    };
    return discInside(a) && discInside(b);
}

}  // namespace amp
