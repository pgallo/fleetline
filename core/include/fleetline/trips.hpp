#pragma once

#include <optional>
#include <string>
#include <vector>

#include "fleetline/telemetry.hpp"

namespace fleetline {

struct TripConfig {
    double move_speed_mps = 2.5;       // speed that starts or resumes driving (~9 km/h)
    double stop_speed_mps = 1.0;       // below this the vehicle is considered stationary
    double min_stop_s = 180.0;         // stationary this long (ignition on) ends a trip
    double ignition_off_stop_s = 60.0; // ignition off this long ends a trip
    double min_trip_distance_m = 250.0;// shorter "trips" are parking-lot shuffles or drift
    double harsh_brake_mps2 = 3.5;
    double harsh_accel_mps2 = 3.0;
    double max_accel_dt_s = 4.0;       // only differentiate speed over short gaps
    double harsh_debounce_s = 5.0;     // one event per manoeuvre
    double max_interval_s = 300.0;     // longer gaps are not counted as idle or driving
};

enum class HarshType { Braking, Acceleration };
const char* to_string(HarshType t);

struct HarshEvent {
    HarshType type = HarshType::Braking;
    double ts = 0.0;
    LatLon pos{};
    double accel_mps2 = 0.0;  // signed: negative for braking
};

struct Trip {
    std::string vehicle_id;
    double start_ts = 0.0;
    double end_ts = 0.0;
    LatLon start_pos{};
    LatLon end_pos{};
    double distance_m = 0.0;
    double idle_s = 0.0;  // ignition on, stationary, while the trip was open
    double max_speed_mps = 0.0;
    std::size_t points = 0;
    std::vector<HarshEvent> harsh_events;

    double duration_s() const { return end_ts - start_ts; }
};

struct Stop {
    std::string vehicle_id;
    double start_ts = 0.0;
    double end_ts = 0.0;
    LatLon pos{};
    double idle_s = 0.0;  // engine running while stopped
    double duration_s() const { return end_ts - start_ts; }
};

enum class MotionState { Parked, Driving, Pausing };
const char* to_string(MotionState s);

struct SegmentOutput {
    std::vector<Trip> trips;
    std::vector<Stop> stops;
};

// Ignition + speed + stop-duration state machine.
//
//   Parked --(ignition on, speed >= move)--> Driving
//   Driving --(speed < stop or ignition off)--> Pausing
//   Pausing --(ignition on, speed >= move)--> Driving       (traffic light, short stop)
//   Pausing --(stationary >= min_stop_s, or ignition off
//              >= ignition_off_stop_s)--> Parked             (trip closed at pause start)
//
// The asymmetric move/stop thresholds are speed hysteresis: a vehicle creeping
// in traffic at 1.5 m/s neither starts nor ends a trip.
class TripSegmenter {
public:
    explicit TripSegmenter(std::string vehicle_id, TripConfig cfg = {});

    SegmentOutput update(const FilteredPoint& p);
    // Close any open trip at the last seen point (end of stream / end of day).
    SegmentOutput flush();

    MotionState state() const { return state_; }
    const std::optional<Trip>& current_trip() const { return trip_; }

private:
    void close_trip(SegmentOutput& out);

    std::string vehicle_id_;
    TripConfig cfg_;
    MotionState state_ = MotionState::Parked;
    std::optional<Trip> trip_;
    std::optional<FilteredPoint> prev_;

    // Open stop (vehicle parked); only reported once the next trip starts.
    bool stop_open_ = false;
    Stop stop_;

    // Pause bookkeeping inside a trip.
    double pause_start_ts_ = 0.0;
    LatLon pause_pos_{};
    double pause_idle_s_ = 0.0;
    double ignition_off_since_ = -1.0;
    double last_harsh_ts_[2] = {-1e18, -1e18};
};

}  // namespace fleetline
