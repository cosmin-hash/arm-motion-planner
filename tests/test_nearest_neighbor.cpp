/// @file test_nearest_neighbor.cpp
/// @brief The k-d tree must return exactly what brute force returns, including on
/// continuous (wrapping) joints and on ties.
#include "amp/cspace.hpp"
#include "amp/nearest_neighbor.hpp"
#include "amp/rng.hpp"

#include <gtest/gtest.h>

namespace amp {
namespace {

/// Arm with a mix of limited and continuous joints.
ArmModel mixedArm(int dof) {
    std::vector<JointLimits> limits;
    for (int i = 0; i < dof; ++i) {
        limits.push_back(i % 2 == 0 ? JointLimits{0, 0, true} : JointLimits{-2.5, 2.5, false});
    }
    return ArmModel(std::vector<double>(static_cast<std::size_t>(dof), 0.2), limits);
}

// @verifies REQ-NN-001
TEST(NearestNeighbor, KdTreeMatchesBruteForce) {
    Rng rng(17);
    for (int dof = 2; dof <= 6; ++dof) {
        const ArmModel arm = mixedArm(dof);
        BruteForceNN brute(arm);
        KdTreeNN kd(arm);
        for (int i = 0; i < 2000; ++i) {
            const Config q = sampleUniform(arm, rng);
            brute.add(q);
            kd.add(q);
        }
        ASSERT_EQ(brute.size(), kd.size());
        for (int i = 0; i < 300; ++i) {
            const Config q = sampleUniform(arm, rng);
            EXPECT_EQ(kd.nearest(q), brute.nearest(q)) << "dof=" << dof;
            const double r = 0.3 + 0.2 * dof;
            EXPECT_EQ(kd.withinRadius(q, r), brute.withinRadius(q, r)) << "dof=" << dof;
        }
    }
}

// @verifies REQ-NN-001
TEST(NearestNeighbor, FindsNeighbourAcrossTheWrapSeam) {
    // Points at +3.1 and -1.0 on a continuous joint; query at -3.1. The true nearest is +3.1
    // (0.083 rad away through the seam), which a naive Euclidean k-d tree would miss.
    const ArmModel arm({1.0, 1.0}, {JointLimits{0, 0, true}, JointLimits{0, 0, true}});
    KdTreeNN kd(arm);
    Config a(2), b(2), q(2);
    a << -1.0, 0.0;
    b << 3.1, 0.0;
    q << -3.1, 0.0;
    kd.add(a);
    kd.add(b);
    EXPECT_EQ(kd.nearest(q), 1u);
    EXPECT_EQ(kd.withinRadius(q, 0.1), std::vector<std::size_t>{1u});
}

// @verifies REQ-NN-001
TEST(NearestNeighbor, TiesResolveToLowestIndexInBothStrategies) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    for (NnStrategy s : {NnStrategy::BruteForce, NnStrategy::KdTree}) {
        auto nn = makeNearestNeighbor(s, arm);
        Config p(2);
        p << 0.5, 0.5;
        Config far(2);
        far << 2.0, 2.0;
        nn->add(far);  // id 0
        nn->add(p);    // id 1
        nn->add(p);    // id 2: exact duplicate
        nn->add(p);    // id 3
        EXPECT_EQ(nn->nearest(p), 1u);
        Config mirror(2);  // equidistant from ids 4 and 5
        mirror << 0.0, 0.0;
        Config left(2), right(2);
        left << -0.1, 0.0;
        right << 0.1, 0.0;
        nn->add(right);  // id 4
        nn->add(left);   // id 5
        EXPECT_EQ(nn->nearest(mirror), 4u);
    }
}

}  // namespace
}  // namespace amp
