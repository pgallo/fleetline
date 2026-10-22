#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "fleetline/filter.hpp"
#include "test_util.hpp"

using namespace fleetline;
using fleetline::testutil::fix;
using fleetline::testutil::offset;

namespace {
double rms(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) s += x * x;
    return std::sqrt(s / static_cast<double>(v.size()));
}
}  // namespace

TEST(Filter, ReducesNoiseOnStraightLine) {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 8.0);
    PositionFilter f;
    std::vector<double> raw_err, filt_err;
    const double v = 12.0;
    for (int i = 0; i < 300; ++i) {
        const double x = v * i;
        const LatLon truth = offset(x, 0);
        const LatLon noisy = offset(x + noise(rng), noise(rng));
        // No device speed: the filter must estimate velocity itself.
        auto out = f.update(fix("v", i, noisy, -1.0, true));
        ASSERT_TRUE(out.has_value());
        if (i < 30) continue;  // warm-up
        raw_err.push_back(haversine_m(truth, noisy));
        filt_err.push_back(haversine_m(truth, out->pos));
    }
    EXPECT_LT(rms(filt_err), 0.6 * rms(raw_err));
}

TEST(Filter, EstimatesVelocityWhenDeviceSpeedMissing) {
    PositionFilter f;
    std::optional<FilteredPoint> out;
    for (int i = 0; i < 60; ++i) out = f.update(fix("v", i, offset(15.0 * i, 0), -1.0, true));
    ASSERT_TRUE(out);
    EXPECT_NEAR(out->filter_speed_mps, 15.0, 0.5);
    EXPECT_NEAR(out->speed_mps, 15.0, 0.5);
}

TEST(Filter, RejectsSingleJumpOutlier) {
    PositionFilter f;
    for (int i = 0; i < 20; ++i) ASSERT_TRUE(f.update(fix("v", i, offset(10.0 * i, 0), 10.0, true)));
    // A 3 km jump in one second is 3000 m/s.
    EXPECT_FALSE(f.update(fix("v", 20, offset(3000, 3000), 10.0, true)).has_value());
    auto back = f.update(fix("v", 21, offset(210, 0), 10.0, true));
    ASSERT_TRUE(back);
    EXPECT_LT(haversine_m(back->pos, offset(210, 0)), 15.0);
    EXPECT_EQ(f.rejected(), 1u);
}

TEST(Filter, ResetsAfterConsecutiveRejections) {
    FilterConfig cfg;
    cfg.reset_after_rejections = 3;
    PositionFilter f(cfg);
    // A bad cold-start fix far from the real track.
    ASSERT_TRUE(f.update(fix("v", 0, offset(5000, 5000), 0.0, true)));
    EXPECT_FALSE(f.update(fix("v", 1, offset(0, 0), 0.0, true)));
    EXPECT_FALSE(f.update(fix("v", 2, offset(0, 0), 0.0, true)));
    auto out = f.update(fix("v", 3, offset(0, 0), 0.0, true));
    ASSERT_TRUE(out);  // third consecutive "outlier" re-initialises on the new track
    EXPECT_LT(haversine_m(out->pos, offset(0, 0)), 1.0);
}

TEST(Filter, RestartsAfterLongGap) {
    PositionFilter f;
    for (int i = 0; i < 10; ++i) f.update(fix("v", i, offset(10.0 * i, 0), 10.0, true));
    // 20 minutes later, 8 km away: a towing or a long tunnel, not an outlier.
    auto out = f.update(fix("v", 1210, offset(8000, 0), 0.0, true));
    ASSERT_TRUE(out);
    EXPECT_LT(haversine_m(out->pos, offset(8000, 0)), 1.0);
}
