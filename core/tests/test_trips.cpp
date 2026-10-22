#include <gtest/gtest.h>

#include "fleetline/trips.hpp"
#include "test_util.hpp"

using namespace fleetline;
using fleetline::testutil::as_filtered;
using fleetline::testutil::fix;
using fleetline::testutil::offset;
using fleetline::testutil::scripted_trip;

namespace {
SegmentOutput run(TripSegmenter& seg, const std::vector<TelemetryPoint>& pts, bool flush = true) {
    SegmentOutput all;
    for (const auto& p : pts) {
        auto o = seg.update(as_filtered(p));
        all.trips.insert(all.trips.end(), o.trips.begin(), o.trips.end());
        all.stops.insert(all.stops.end(), o.stops.begin(), o.stops.end());
    }
    if (flush) {
        auto o = seg.flush();
        all.trips.insert(all.trips.end(), o.trips.begin(), o.trips.end());
        all.stops.insert(all.stops.end(), o.stops.begin(), o.stops.end());
    }
    return all;
}
}  // namespace

TEST(Trips, SingleTripDistanceDurationAndStop) {
    TripSegmenter seg("v1");
    auto out = run(seg, scripted_trip("v1", 1000, 10.0, 600, 300));
    ASSERT_EQ(out.trips.size(), 1u);
    const Trip& t = out.trips[0];
    EXPECT_NEAR(t.distance_m, 6000.0, 15.0);
    EXPECT_NEAR(t.duration_s(), 600.0, 15.0);
    EXPECT_NEAR(t.max_speed_mps, 10.0, 1e-9);
    // The engine-on stop after the trip is reported with its idle time.
    ASSERT_EQ(out.stops.size(), 1u);
    EXPECT_NEAR(out.stops[0].idle_s, 300.0, 2.0);
    EXPECT_NEAR(out.stops[0].start_ts, t.end_ts, 1e-9);
    EXPECT_EQ(seg.state(), MotionState::Parked);
}

TEST(Trips, ShortStopStaysInsideTripAsIdle) {
    std::vector<TelemetryPoint> pts;
    double t = 0, x = 0;
    for (int i = 0; i < 300; ++i, t += 1) pts.push_back(fix("v", t, offset(x += 12, 0), 12.0, true));
    for (int i = 0; i < 90; ++i, t += 1) pts.push_back(fix("v", t, offset(x, 0), 0.0, true));  // red light
    for (int i = 0; i < 300; ++i, t += 1) pts.push_back(fix("v", t, offset(x += 12, 0), 12.0, true));
    for (int i = 0; i < 200; ++i, t += 1) pts.push_back(fix("v", t, offset(x, 0), 0.0, false));
    TripSegmenter seg("v");
    auto out = run(seg, pts);
    ASSERT_EQ(out.trips.size(), 1u);
    EXPECT_NEAR(out.trips[0].idle_s, 89.0, 2.0);
    EXPECT_NEAR(out.trips[0].distance_m, 600 * 12.0, 30.0);
}

TEST(Trips, LongStopSplitsTrips) {
    std::vector<TelemetryPoint> pts;
    double t = 0, x = 0;
    for (int i = 0; i < 120; ++i, t += 1) pts.push_back(fix("v", t, offset(x += 10, 0), 10.0, true));
    for (int i = 0; i < 400; ++i, t += 1) pts.push_back(fix("v", t, offset(x, 0), 0.0, true));  // delivery, engine on
    for (int i = 0; i < 120; ++i, t += 1) pts.push_back(fix("v", t, offset(x += 10, 0), 10.0, true));
    TripSegmenter seg("v");
    auto out = run(seg, pts);
    ASSERT_EQ(out.trips.size(), 2u);
    ASSERT_EQ(out.stops.size(), 1u);
    EXPECT_NEAR(out.stops[0].duration_s(), 400.0, 2.0);
    EXPECT_NEAR(out.stops[0].idle_s, 399.0, 2.0);
}

TEST(Trips, IgnitionOffEndsTripQuickly) {
    std::vector<TelemetryPoint> pts;
    double t = 0, x = 0;
    for (int i = 0; i < 120; ++i, t += 1) pts.push_back(fix("v", t, offset(x += 10, 0), 10.0, true));
    for (int i = 0; i < 90; ++i, t += 1) pts.push_back(fix("v", t, offset(x, 0), 0.0, false));
    TripSegmenter seg("v");
    auto out = run(seg, pts, false);
    ASSERT_EQ(out.trips.size(), 1u);  // closed by ignition-off timeout, before min_stop_s
    EXPECT_NEAR(out.trips[0].end_ts, 120.0, 1e-9);
}

TEST(Trips, CreepingInTrafficDoesNotStartTrip) {
    std::vector<TelemetryPoint> pts;
    double x = 0;
    for (int i = 0; i < 600; ++i) pts.push_back(fix("v", i, offset(x += 1.5, 0), 1.5, true));
    TripSegmenter seg("v");
    auto out = run(seg, pts);
    EXPECT_TRUE(out.trips.empty());
}

TEST(Trips, TooShortTripIsDiscarded) {
    TripSegmenter seg("v");
    auto out = run(seg, scripted_trip("v", 0, 5.0, 30, 0));  // 150 m
    EXPECT_TRUE(out.trips.empty());
}

TEST(Trips, HarshEventsDebounced) {
    std::vector<TelemetryPoint> pts;
    double t = 0, x = 0;
    auto push = [&](double v) {
        x += v;
        pts.push_back(fix("v", t, offset(x, 0), v, true));
        t += 1;
    };
    for (int i = 0; i < 60; ++i) push(15.0);
    for (double v = 15.0; v > 0.5; v -= 4.5) push(v);  // braking at 4.5 m/s^2 for ~3 s
    for (int i = 0; i < 5; ++i) push(0.0);
    for (double v = 0.0; v < 14.0; v += 3.5) push(v);  // accelerating at 3.5 m/s^2
    for (int i = 0; i < 60; ++i) push(14.0);
    TripSegmenter seg("v");
    auto out = run(seg, pts);
    ASSERT_EQ(out.trips.size(), 1u);
    int brakes = 0, accels = 0;
    for (const auto& e : out.trips[0].harsh_events) (e.type == HarshType::Braking ? brakes : accels)++;
    EXPECT_EQ(brakes, 1);
    EXPECT_EQ(accels, 1);
}
