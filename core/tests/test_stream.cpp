#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <thread>

#include "fleetline/stream.hpp"
#include "test_util.hpp"

using namespace fleetline;
using fleetline::testutil::fix;
using fleetline::testutil::offset;
using fleetline::testutil::scripted_trip;

TEST(Stream, ReordersWithinWindow) {
    StreamConfig cfg;
    cfg.reorder_window_s = 60.0;
    StreamProcessor sp(cfg);
    auto pts = scripted_trip("v1", 0, 10.0, 300, 0, 300, 1.0);
    // Shuffle within blocks of 5 points (at most 50 s of event time, inside the window).
    std::mt19937 rng(3);
    for (std::size_t i = 0; i + 5 <= pts.size(); i += 5) std::shuffle(pts.begin() + i, pts.begin() + i + 5, rng);
    sp.ingest_batch(pts);
    sp.flush();
    const auto st = sp.stats();
    EXPECT_EQ(st.late_dropped, 0u);
    EXPECT_EQ(st.processed, pts.size());
    auto trips = sp.trips("v1");
    ASSERT_EQ(trips.size(), 1u);
    EXPECT_NEAR(trips[0].distance_m, 3000.0, 30.0);
}

TEST(Stream, DropsLateAndDuplicatePoints) {
    StreamConfig cfg;
    cfg.reorder_window_s = 5.0;
    StreamProcessor sp(cfg);
    for (int i = 0; i < 100; ++i) sp.ingest(fix("v", i, offset(10.0 * i, 0), 10.0, true));
    // Watermark is at 94, so everything up to ts 94 has been processed.
    sp.ingest(fix("v", 50.5, offset(505, 0), 10.0, true));  // too late
    sp.ingest(fix("v", 94, offset(940, 0), 10.0, true));    // re-sent last point
    sp.ingest(fix("v", 97.5, offset(975, 0), 10.0, true));  // late but inside the window: accepted
    const auto st = sp.stats();
    EXPECT_EQ(st.late_dropped, 1u);
    EXPECT_EQ(st.duplicates, 1u);
    sp.flush();
    EXPECT_EQ(sp.stats().processed, 101u);
}

TEST(Stream, BufferOverflowForcesRelease) {
    StreamConfig cfg;
    cfg.reorder_window_s = 1e9;  // watermark never passes
    cfg.max_buffer = 8;
    StreamProcessor sp(cfg);
    for (int i = 0; i < 20; ++i) sp.ingest(fix("v", i, offset(i, 0), 1.0, true));
    auto s = sp.status("v");
    ASSERT_TRUE(s);
    EXPECT_EQ(s->buffered, 8u);
    EXPECT_EQ(sp.stats().processed, 12u);
}

TEST(Stream, GeofenceEventsAndStatus) {
    StreamProcessor sp;
    sp.add_geofence(Geofence::circle("yard", "Yard", offset(3000, 0), 150.0, 60.0));
    sp.ingest_batch(scripted_trip("v", 0, 10.0, 300, 200));  // ends at x = 3000, idling
    sp.flush();
    auto ev = sp.geofence_events("v");
    ASSERT_GE(ev.size(), 2u);
    EXPECT_EQ(ev[0].type, GeofenceEventType::Enter);
    EXPECT_EQ(ev[1].type, GeofenceEventType::Dwell);
    auto s = sp.status("v");
    ASSERT_TRUE(s);
    EXPECT_EQ(s->inside_fences, std::vector<std::string>{"yard"});
    EXPECT_FALSE(s->ignition);
    EXPECT_FALSE(sp.status("nope").has_value());

    auto drained = sp.drain();
    EXPECT_EQ(drained.trips.size(), 1u);
    EXPECT_EQ(drained.geofence_events.size(), ev.size());
    EXPECT_TRUE(sp.drain().trips.empty());
}

TEST(Stream, ConcurrentIngestAcrossVehicles) {
    StreamProcessor sp;
    sp.add_geofence(Geofence::circle("hub", "Hub", offset(0, 0), 200.0));
    constexpr int kThreads = 8;
    constexpr int kVehiclesPerThread = 5;
    std::vector<std::thread> workers;
    std::size_t expected = 0;
    std::vector<std::vector<TelemetryPoint>> work(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        for (int v = 0; v < kVehiclesPerThread; ++v) {
            auto pts = scripted_trip("v" + std::to_string(t) + "_" + std::to_string(v), 0, 9.0 + v, 400, 60);
            expected += pts.size();
            work[t].insert(work[t].end(), pts.begin(), pts.end());
        }
    }
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t] {
            sp.ingest_batch(work[t]);
            // Readers racing with writers must be safe too.
            for (const auto& id : sp.vehicles()) (void)sp.status(id);
        });
    }
    // Fence updates racing with ingestion.
    workers.emplace_back([&] {
        for (int i = 0; i < 50; ++i) sp.add_geofence(Geofence::circle("tmp", "tmp", offset(i * 100.0, 0), 50.0));
        sp.remove_geofence("tmp");
    });
    for (auto& w : workers) w.join();
    sp.flush();
    const auto st = sp.stats();
    EXPECT_EQ(st.ingested, expected);
    EXPECT_EQ(st.processed, expected);
    EXPECT_EQ(st.vehicles, static_cast<std::size_t>(kThreads * kVehiclesPerThread));
    for (const auto& id : sp.vehicles()) EXPECT_EQ(sp.trips(id).size(), 1u) << id;
}

TEST(Stream, RecentSpeed) {
    StreamProcessor sp(StreamConfig{0.0});
    for (int i = 0; i < 100; ++i) sp.ingest(fix("v", i, offset(8.0 * i, 0), 8.0, true));
    EXPECT_NEAR(sp.recent_speed("v", 60.0), 8.0, 1e-9);
    EXPECT_LT(sp.recent_speed("unknown"), 0.0);
}
