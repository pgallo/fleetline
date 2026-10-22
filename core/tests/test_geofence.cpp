#include <gtest/gtest.h>

#include "fleetline/geofence.hpp"
#include "test_util.hpp"

using namespace fleetline;
using fleetline::testutil::offset;

namespace {
// 200 m x 200 m square centred on (cx, cy) metres from the origin.
Geofence square(const std::string& id, double cx, double cy, double half = 100.0, double dwell = 60.0) {
    return Geofence::polygon(id, id,
                             {offset(cx - half, cy - half), offset(cx + half, cy - half), offset(cx + half, cy + half),
                              offset(cx - half, cy + half)},
                             dwell);
}
}  // namespace

TEST(PointInPolygon, ConvexAndConcave) {
    // L-shaped polygon (concave).
    const std::vector<LatLon> L = {{0, 0}, {0, 2}, {1, 2}, {1, 1}, {2, 1}, {2, 0}};
    EXPECT_TRUE(point_in_polygon({0.5, 0.5}, L));
    EXPECT_TRUE(point_in_polygon({0.5, 1.5}, L));
    EXPECT_TRUE(point_in_polygon({1.5, 0.5}, L));
    EXPECT_FALSE(point_in_polygon({1.5, 1.5}, L));  // the notch
    EXPECT_FALSE(point_in_polygon({3, 3}, L));
}

TEST(PointInPolygon, WindingOrderDoesNotMatter) {
    std::vector<LatLon> ccw = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
    std::vector<LatLon> cw(ccw.rbegin(), ccw.rend());
    EXPECT_EQ(std::abs(winding_number({0.5, 0.5}, ccw)), 1);
    EXPECT_EQ(std::abs(winding_number({0.5, 0.5}, cw)), 1);
    EXPECT_EQ(winding_number({0.5, 0.5}, ccw), -winding_number({0.5, 0.5}, cw));
}

TEST(PointInPolygon, SelfOverlappingCountsAsInside) {
    // A pentagram: the centre has winding number 2, which even-odd would call outside.
    std::vector<LatLon> star;
    for (int k = 0; k < 5; ++k) {
        const double a = deg2rad(90.0 + 144.0 * k);
        star.push_back({std::sin(a), std::cos(a)});
    }
    EXPECT_EQ(std::abs(winding_number({0, 0}, star)), 2);
    EXPECT_TRUE(point_in_polygon({0, 0}, star));
}

TEST(PreparedFence, SignedDistance) {
    PreparedFence sq(square("a", 0, 0));
    EXPECT_NEAR(sq.signed_distance_m(offset(0, 0)), -100.0, 0.5);
    EXPECT_NEAR(sq.signed_distance_m(offset(130, 0)), 30.0, 0.5);
    PreparedFence c(Geofence::circle("c", "c", offset(0, 0), 50.0));
    EXPECT_NEAR(c.signed_distance_m(offset(20, 0)), -30.0, 0.1);
    EXPECT_TRUE(c.contains(offset(0, 49)));
    EXPECT_FALSE(c.contains(offset(0, 51)));
}

TEST(GeofenceIndex, CandidatesAreLocal) {
    GeofenceIndex idx(0.01, 30.0);
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) idx.add(square("f" + std::to_string(i) + "_" + std::to_string(j), i * 2000.0, j * 2000.0));
    }
    EXPECT_EQ(idx.size(), 400u);
    auto c = idx.candidates(offset(4000, 6000));
    ASSERT_FALSE(c.empty());
    EXPECT_LE(c.size(), 4u);  // only fences near this cell, not all 400
    bool found = false;
    for (auto* pf : c) found |= pf->fence().id == "f2_3";
    EXPECT_TRUE(found);
    EXPECT_TRUE(idx.candidates(offset(-50000, -50000)).empty());
}

TEST(GeofenceIndex, RemoveAndReplace) {
    GeofenceIndex idx;
    idx.add(square("a", 0, 0));
    EXPECT_EQ(idx.candidates(offset(0, 0)).size(), 1u);
    idx.add(square("a", 5000, 0));  // same id replaces
    EXPECT_TRUE(idx.candidates(offset(0, 0)).empty());
    EXPECT_EQ(idx.candidates(offset(5000, 0)).size(), 1u);
    EXPECT_TRUE(idx.remove("a"));
    EXPECT_FALSE(idx.remove("a"));
    EXPECT_EQ(idx.size(), 0u);
}

TEST(GeofenceTracker, EnterDwellExit) {
    GeofenceIndex idx;
    idx.add(square("depot", 0, 0, 100.0, 60.0));
    GeofenceTracker tr(15.0);
    std::vector<GeofenceEvent> all;
    auto feed = [&](double ts, double x) {
        auto ev = tr.update("v1", ts, offset(x, 0), idx);
        all.insert(all.end(), ev.begin(), ev.end());
    };
    feed(0, -300);
    feed(10, -95);  // inside by 5 m: below hysteresis, no enter yet
    EXPECT_TRUE(all.empty());
    feed(20, -50);
    ASSERT_EQ(all.size(), 1u);
    EXPECT_EQ(all[0].type, GeofenceEventType::Enter);
    for (int t = 30; t <= 100; t += 10) feed(t, 0);
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[1].type, GeofenceEventType::Dwell);
    EXPECT_GE(all[1].dwell_s, 60.0);
    feed(110, 300);
    ASSERT_EQ(all.size(), 3u);
    EXPECT_EQ(all[2].type, GeofenceEventType::Exit);
    EXPECT_DOUBLE_EQ(all[2].dwell_s, 90.0);
    EXPECT_TRUE(tr.inside().empty());
}

TEST(GeofenceTracker, HysteresisSuppressesBoundaryJitter) {
    GeofenceIndex idx;
    idx.add(square("gate", 0, 0));
    GeofenceTracker tr(15.0);
    std::size_t events = 0;
    events += tr.update("v", 0, offset(-60, 0), idx).size();  // enter
    // Oscillate +-10 m around the east edge at x = 100.
    for (int i = 1; i <= 50; ++i) events += tr.update("v", i, offset(i % 2 ? 90 : 110, 0), idx).size();
    EXPECT_EQ(events, 1u);
    EXPECT_EQ(tr.inside(), std::vector<std::string>{"gate"});
}

TEST(GeofenceTracker, ExitWhenLeavingIndexedArea) {
    GeofenceIndex idx;
    idx.add(Geofence::circle("c", "c", offset(0, 0), 80.0));
    GeofenceTracker tr;
    EXPECT_EQ(tr.update("v", 0, offset(0, 0), idx).size(), 1u);
    // Next fix is far away (e.g. after a data gap): no candidates at all, still an exit.
    auto ev = tr.update("v", 600, offset(20000, 0), idx);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].type, GeofenceEventType::Exit);
}
