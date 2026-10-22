#include <gtest/gtest.h>

#include "fleetline/eta.hpp"
#include "test_util.hpp"

using namespace fleetline;
using fleetline::testutil::offset;

namespace {
// An L-shaped 2 km route: 1 km east at 10 m/s, then 1 km north at 20 m/s.
Route l_route() { return Route({offset(0, 0), offset(1000, 0), offset(1000, 1000)}, {10.0, 20.0}); }
}  // namespace

TEST(Route, LengthAndFreeFlowTime) {
    Route r = l_route();
    EXPECT_NEAR(r.length_m(), 2000.0, 1.0);
    auto m = r.match(offset(0, 0));
    EXPECT_NEAR(r.time_to_end_s(m), 100.0 + 50.0, 0.2);
}

TEST(Route, MatchSnapsToNearestSegment) {
    Route r = l_route();
    auto m = r.match(offset(500, 20));
    EXPECT_EQ(m.segment, 0u);
    EXPECT_NEAR(m.offset_m, 500.0, 0.5);
    EXPECT_NEAR(m.cross_track_m, 20.0, 0.5);
    m = r.match(offset(1010, 400));
    EXPECT_EQ(m.segment, 1u);
    EXPECT_NEAR(m.offset_m, 1400.0, 0.5);
}

TEST(Route, ProgressBiasOnRepeatedStreet) {
    // Out and back along the same street: without a progress hint the match
    // is ambiguous; with one, it stays on the return leg.
    Route r({offset(0, 0), offset(1000, 0), offset(0, 0)});
    auto naive = r.match(offset(300, 0));
    EXPECT_NEAR(naive.offset_m, 300.0, 0.5);
    auto biased = r.match(offset(300, 0), 1500.0);
    EXPECT_NEAR(biased.offset_m, 1700.0, 0.5);
}

TEST(Eta, ProfileOnlyAndObservedBlend) {
    Route r = l_route();
    auto e = r.eta(offset(500, 0));
    EXPECT_FALSE(e.off_route);
    EXPECT_NEAR(e.remaining_m, 1500.0, 1.0);
    EXPECT_NEAR(e.eta_s, 50.0 + 50.0, 0.2);

    // Heavy traffic: observed 3 m/s should lengthen the ETA a lot, but less
    // than assuming 3 m/s for the whole remaining distance.
    auto slow = r.eta(offset(500, 0), 3.0);
    EXPECT_GT(slow.eta_s, e.eta_s * 2.0);
    EXPECT_LT(slow.eta_s, 1500.0 / 3.0);

    // Free-flowing traffic matching the profile leaves the ETA roughly unchanged.
    auto fast = r.eta(offset(500, 0), 15.0);
    EXPECT_NEAR(fast.eta_s, e.eta_s, 15.0);
}

TEST(Eta, OffRouteAddsRejoinCost) {
    Route r = l_route();
    auto e = r.eta(offset(500, 400));
    EXPECT_TRUE(e.off_route);
    EXPECT_GT(e.eta_s, r.eta(offset(500, 0)).eta_s);
}

TEST(Eta, AtDestinationIsZero) {
    Route r = l_route();
    auto e = r.eta(offset(1000, 1000), 10.0);
    EXPECT_NEAR(e.remaining_m, 0.0, 0.5);
    EXPECT_NEAR(e.eta_s, 0.0, 0.5);
}

TEST(Route, RejectsBadInput) {
    EXPECT_THROW(Route({offset(0, 0)}), std::invalid_argument);
    EXPECT_THROW(Route({offset(0, 0), offset(1, 1)}, {1.0, 2.0}), std::invalid_argument);
    EXPECT_THROW(Route({offset(0, 0), offset(1, 1)}, {0.0}), std::invalid_argument);
}
