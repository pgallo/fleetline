#include <gtest/gtest.h>

#include "fleetline/geo.hpp"
#include "test_util.hpp"

using namespace fleetline;

TEST(Geo, HaversineOneDegreeLatitude) {
    // One degree of latitude on the mean-radius sphere is ~111.195 km.
    EXPECT_NEAR(haversine_m({0, 0}, {1, 0}), 111195.0, 5.0);
    EXPECT_DOUBLE_EQ(haversine_m({45, 9}, {45, 9}), 0.0);
}

TEST(Geo, HaversineIsSymmetric) {
    const LatLon a{45.1, 9.2}, b{45.3, 8.9};
    EXPECT_DOUBLE_EQ(haversine_m(a, b), haversine_m(b, a));
}

TEST(Geo, BearingCardinalDirections) {
    EXPECT_NEAR(bearing_deg({45, 9}, {46, 9}), 0.0, 1e-6);
    EXPECT_NEAR(bearing_deg({45, 9}, {44, 9}), 180.0, 1e-6);
    EXPECT_NEAR(bearing_deg({0, 0}, {0, 1}), 90.0, 1e-6);
    EXPECT_NEAR(bearing_deg({0, 1}, {0, 0}), 270.0, 1e-6);
}

TEST(Geo, ProjectionRoundTripAndScale) {
    LocalProjection proj(testutil::kOrigin);
    const LatLon p = proj.to_latlon({1200.0, -800.0});
    const auto xy = proj.to_xy(p);
    EXPECT_NEAR(xy.x, 1200.0, 1e-6);
    EXPECT_NEAR(xy.y, -800.0, 1e-6);
    // Planar distance agrees with haversine at city scale.
    EXPECT_NEAR(haversine_m(testutil::kOrigin, p), std::hypot(1200.0, 800.0), 0.5);
}

TEST(Geo, ProjectOnSegmentClamps) {
    using XY = LocalProjection::XY;
    auto r = project_on_segment(XY{5, 3}, XY{0, 0}, XY{10, 0});
    EXPECT_DOUBLE_EQ(r.t, 0.5);
    EXPECT_DOUBLE_EQ(r.distance, 3.0);
    r = project_on_segment(XY{-4, 3}, XY{0, 0}, XY{10, 0});
    EXPECT_DOUBLE_EQ(r.t, 0.0);
    EXPECT_DOUBLE_EQ(r.distance, 5.0);
    r = project_on_segment(XY{1, 1}, XY{0, 0}, XY{0, 0});  // degenerate segment
    EXPECT_NEAR(r.distance, std::sqrt(2.0), 1e-12);
}
