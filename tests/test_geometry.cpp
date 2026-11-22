/// @file test_geometry.cpp
/// @brief Unit tests for the 2D collision primitives, including edge cases:
/// tangency, touching endpoints, collinear overlap, and "fully inside".
#include "amp/geometry.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace amp {
namespace {

const ConvexPolygon kUnitSquare = ConvexPolygon::rectangle(Vec2(0, 0), Vec2(1, 1));

// ---------------------------------------------------------------------------
// ConvexPolygon validation
// ---------------------------------------------------------------------------

// @verifies REQ-COL-001
TEST(ConvexPolygon, NormalisesClockwiseInputToCounterClockwise) {
    const ConvexPolygon cw({Vec2(0, 0), Vec2(0, 1), Vec2(1, 1), Vec2(1, 0)});
    // Counter-clockwise means every vertex is to the left of the previous edge.
    for (std::size_t i = 0; i < cw.size(); ++i) {
        EXPECT_GT(cross(cw.vertex(i + 1) - cw.vertex(i), cw.vertex(i + 2) - cw.vertex(i + 1)), 0.0);
    }
    EXPECT_TRUE(cw.centroid().isApprox(Vec2(0.5, 0.5)));
}

// @verifies REQ-COL-001
TEST(ConvexPolygon, RejectsInvalidShapes) {
    EXPECT_THROW(ConvexPolygon({Vec2(0, 0), Vec2(1, 0)}), std::invalid_argument);              // too few
    EXPECT_THROW(ConvexPolygon({Vec2(0, 0), Vec2(1, 0), Vec2(2, 0)}), std::invalid_argument);  // zero area
    EXPECT_THROW(ConvexPolygon({Vec2(0, 0), Vec2(2, 0), Vec2(1, 0.5), Vec2(1, 2)}),            // concave
                 std::invalid_argument);
    // Pentagram: only left turns, but winds around twice.
    EXPECT_THROW(
        ConvexPolygon({Vec2(0, 1), Vec2(0.588, -0.809), Vec2(-0.951, 0.309), Vec2(0.951, 0.309), Vec2(-0.588, -0.809)}),
        std::invalid_argument);
}

// @verifies REQ-COL-001
TEST(ConvexPolygon, TranslateMovesAllVertices) {
    ConvexPolygon p = kUnitSquare;
    p.translate(Vec2(2, -1));
    EXPECT_TRUE(p.centroid().isApprox(Vec2(2.5, -0.5)));
}

// ---------------------------------------------------------------------------
// Point / segment primitives
// ---------------------------------------------------------------------------

// @verifies REQ-COL-002
TEST(PointSegment, DistanceCases) {
    const Vec2 a(0, 0), b(2, 0);
    EXPECT_DOUBLE_EQ(pointSegmentDistance(Vec2(1, 1), a, b), 1.0);   // perpendicular foot inside
    EXPECT_DOUBLE_EQ(pointSegmentDistance(Vec2(-3, 4), a, b), 5.0);  // beyond endpoint a
    EXPECT_DOUBLE_EQ(pointSegmentDistance(Vec2(2, 0), a, b), 0.0);   // on endpoint
    EXPECT_DOUBLE_EQ(pointSegmentDistance(Vec2(3, 4), a, a), 5.0);   // degenerate segment
}

// @verifies REQ-COL-003
TEST(SegmentSegment, IntersectionCases) {
    EXPECT_TRUE(segmentsIntersect(Vec2(0, 0), Vec2(2, 2), Vec2(0, 2), Vec2(2, 0)));   // proper crossing
    EXPECT_TRUE(segmentsIntersect(Vec2(0, 0), Vec2(1, 0), Vec2(1, 0), Vec2(2, 5)));   // touching endpoints
    EXPECT_TRUE(segmentsIntersect(Vec2(0, 0), Vec2(2, 0), Vec2(1, 0), Vec2(1, 3)));   // T-junction
    EXPECT_TRUE(segmentsIntersect(Vec2(0, 0), Vec2(2, 0), Vec2(1, 0), Vec2(3, 0)));   // collinear overlap
    EXPECT_FALSE(segmentsIntersect(Vec2(0, 0), Vec2(1, 0), Vec2(2, 0), Vec2(3, 0)));  // collinear, disjoint
    EXPECT_FALSE(segmentsIntersect(Vec2(0, 0), Vec2(1, 0), Vec2(0, 1), Vec2(1, 1)));  // parallel
}

// @verifies REQ-COL-003
TEST(SegmentSegment, DistanceCases) {
    EXPECT_DOUBLE_EQ(segmentSegmentDistance(Vec2(0, 0), Vec2(1, 0), Vec2(0, 1), Vec2(1, 1)), 1.0);
    EXPECT_DOUBLE_EQ(segmentSegmentDistance(Vec2(0, 0), Vec2(2, 2), Vec2(0, 2), Vec2(2, 0)), 0.0);
    EXPECT_NEAR(segmentSegmentDistance(Vec2(0, 0), Vec2(1, 0), Vec2(2, 1), Vec2(3, 1)), std::sqrt(2.0), 1e-15);
}

// @verifies REQ-COL-003
TEST(PointInPolygon, InsideBoundaryOutside) {
    EXPECT_TRUE(pointInConvexPolygon(Vec2(0.5, 0.5), kUnitSquare));
    EXPECT_TRUE(pointInConvexPolygon(Vec2(1.0, 0.5), kUnitSquare));  // on an edge
    EXPECT_TRUE(pointInConvexPolygon(Vec2(1.0, 1.0), kUnitSquare));  // on a vertex
    EXPECT_FALSE(pointInConvexPolygon(Vec2(1.0001, 0.5), kUnitSquare));
}

// ---------------------------------------------------------------------------
// Capsule versus circle
// ---------------------------------------------------------------------------

// @verifies REQ-COL-002
TEST(CapsuleCircle, SeparatedTangentAndOverlapping) {
    const Circle c{Vec2(1, 1), 0.5};
    // Capsule core along the x axis, radius 0.25: gap to the circle = 1 - 0.5 - 0.25 = 0.25.
    EXPECT_FALSE(capsuleIntersectsCircle(Vec2(0, 0), Vec2(2, 0), 0.25, c));
    // Tangent: distance exactly equals r + R. Touching counts as collision.
    EXPECT_TRUE(capsuleIntersectsCircle(Vec2(0, 0), Vec2(2, 0), 0.5, c));
    EXPECT_TRUE(capsuleIntersectsCircle(Vec2(0, 0.6), Vec2(2, 0.6), 0.0, c));
}

// @verifies REQ-COL-002
TEST(CapsuleCircle, EndCapAndFullyInside) {
    const Circle c{Vec2(3, 0), 0.5};
    // Only the rounded end cap reaches the circle: core ends at x = 2.25, radius 0.25, so the
    // cap touches x = 2.5 exactly. (Values chosen to be exact in binary floating point.)
    EXPECT_TRUE(capsuleIntersectsCircle(Vec2(0, 0), Vec2(2.25, 0), 0.25, c));
    EXPECT_FALSE(capsuleIntersectsCircle(Vec2(0, 0), Vec2(2.24, 0), 0.25, c));
    // Capsule entirely inside a big circle.
    EXPECT_TRUE(capsuleIntersectsCircle(Vec2(2.9, 0), Vec2(3.1, 0), 0.01, Circle{Vec2(3, 0), 2.0}));
    // Circle entirely inside a fat capsule.
    EXPECT_TRUE(capsuleIntersectsCircle(Vec2(0, 0), Vec2(5, 0), 3.0, c));
}

// ---------------------------------------------------------------------------
// Capsule versus polygon
// ---------------------------------------------------------------------------

// @verifies REQ-COL-003
TEST(CapsulePolygon, SeparatedAndTouchingEdge) {
    // Horizontal capsule below the unit square, gap 0.2 - r.
    EXPECT_FALSE(capsuleIntersectsPolygon(Vec2(0, -0.2), Vec2(1, -0.2), 0.1, kUnitSquare));
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(0, -0.2), Vec2(1, -0.2), 0.2, kUnitSquare));  // tangent to edge
}

// @verifies REQ-COL-003
TEST(CapsulePolygon, TouchingVertexAndEndpoint) {
    // Diagonal approach to the (1, 1) vertex: core ends at (1.3, 1.4), distance 0.5 to the vertex.
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(2, 2), Vec2(1.3, 1.4), 0.5, kUnitSquare));
    EXPECT_FALSE(capsuleIntersectsPolygon(Vec2(2, 2), Vec2(1.3, 1.4), 0.49, kUnitSquare));
    // Core segment endpoint lies exactly on the polygon edge, zero radius.
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(1, 0.5), Vec2(3, 0.5), 0.0, kUnitSquare));
}

// @verifies REQ-COL-003
TEST(CapsulePolygon, CrossingWithBothEndpointsOutside) {
    // The core passes straight through the square; neither endpoint is inside.
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(-1, 0.5), Vec2(2, 0.5), 0.0, kUnitSquare));
}

// @verifies REQ-COL-003
TEST(CapsulePolygon, FullyInsideAndFullyContaining) {
    // Segment entirely inside the polygon: no edge crossing at all, must still collide.
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(0.2, 0.2), Vec2(0.8, 0.8), 0.01, kUnitSquare));
    // Polygon entirely inside a fat capsule.
    EXPECT_TRUE(capsuleIntersectsPolygon(Vec2(-5, 0.5), Vec2(5, 0.5), 3.0, kUnitSquare));
}

// ---------------------------------------------------------------------------
// Capsule versus capsule, and bounds
// ---------------------------------------------------------------------------

// @verifies REQ-COL-005
TEST(CapsuleCapsule, SeparatedTouchingCrossing) {
    EXPECT_FALSE(capsulesIntersect(Vec2(0, 0), Vec2(1, 0), 0.1, Vec2(0, 0.5), Vec2(1, 0.5), 0.1));
    EXPECT_TRUE(capsulesIntersect(Vec2(0, 0), Vec2(1, 0), 0.25, Vec2(0, 0.5), Vec2(1, 0.5), 0.25));  // tangent
    EXPECT_TRUE(capsulesIntersect(Vec2(0, 0), Vec2(1, 1), 0.0, Vec2(0, 1), Vec2(1, 0), 0.0));        // crossing
}

// @verifies REQ-COL-004
TEST(Bounds, CapsuleInsideTouchingAndOutside) {
    const Bounds b{Vec2(-1, -1), Vec2(1, 1)};
    EXPECT_TRUE(capsuleInsideBounds(Vec2(-0.5, 0), Vec2(0.5, 0), 0.1, b));
    EXPECT_FALSE(capsuleInsideBounds(Vec2(-0.5, 0), Vec2(0.9, 0), 0.1, b));  // end cap touches x = 1
    EXPECT_FALSE(capsuleInsideBounds(Vec2(-0.5, 0), Vec2(1.5, 0), 0.1, b));  // pokes out
}

}  // namespace
}  // namespace amp
