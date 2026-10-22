#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "fleetline/filter.hpp"
#include "fleetline/geofence.hpp"
#include "fleetline/telemetry.hpp"
#include "fleetline/trips.hpp"

namespace fleetline {

struct StreamConfig {
    double reorder_window_s = 30.0;  // hold points this long (in event time) before processing
    std::size_t max_buffer = 64;     // per vehicle; overflow forces release of the oldest
    double geofence_hysteresis_m = 15.0;
    double geofence_cell_deg = 0.01;
    FilterConfig filter{};
    TripConfig trips{};
};

struct VehicleStatus {
    std::string vehicle_id;
    double last_ts = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double speed_mps = 0.0;
    double heading_deg = 0.0;
    bool ignition = false;
    MotionState state = MotionState::Parked;
    std::vector<std::string> inside_fences;
    std::size_t trips = 0;
    std::size_t buffered = 0;
};

struct StreamStats {
    std::size_t ingested = 0;
    std::size_t processed = 0;
    std::size_t late_dropped = 0;
    std::size_t duplicates = 0;
    std::size_t outliers = 0;
    std::size_t vehicles = 0;
};

// Everything produced since the last drain(), for incremental consumers.
struct StreamOutput {
    std::vector<Trip> trips;
    std::vector<Stop> stops;
    std::vector<GeofenceEvent> geofence_events;
};

// Thread-safe, per-vehicle stream processor.
//
// Locking: a shared_mutex guards the vehicle map (shared for lookups, unique
// only to insert a new vehicle); each vehicle has its own mutex so different
// vehicles are processed in parallel; the geofence index has its own
// shared_mutex (shared while matching, unique when fences change). Lock order
// is always vehicle -> index, never the reverse.
//
// Ordering: points for one vehicle may arrive out of order. They sit in a
// min-heap until the vehicle's watermark (max event time seen minus
// reorder_window_s) passes them. Anything older than the last processed point
// is counted as late and dropped: the filter and state machines are strictly
// forward-in-time.
class StreamProcessor {
public:
    explicit StreamProcessor(StreamConfig cfg = {});
    ~StreamProcessor();
    StreamProcessor(const StreamProcessor&) = delete;
    StreamProcessor& operator=(const StreamProcessor&) = delete;

    void add_geofence(Geofence fence);
    bool remove_geofence(const std::string& id);
    std::vector<Geofence> geofences() const;

    // Returns the number of points processed (released from the reorder buffer) by this call.
    std::size_t ingest(const TelemetryPoint& p);
    std::size_t ingest_batch(const std::vector<TelemetryPoint>& pts);

    // Release all buffered points and close open trips (end of stream).
    void flush();
    void flush_vehicle(const std::string& vehicle_id);

    std::vector<std::string> vehicles() const;
    std::optional<VehicleStatus> status(const std::string& vehicle_id) const;
    std::vector<Trip> trips(const std::string& vehicle_id) const;
    std::vector<Stop> stops(const std::string& vehicle_id) const;
    std::vector<GeofenceEvent> geofence_events(const std::string& vehicle_id) const;
    std::optional<FilteredPoint> last_point(const std::string& vehicle_id) const;
    // Mean speed over the last `window_s` seconds of processed points (for ETA).
    double recent_speed(const std::string& vehicle_id, double window_s = 300.0) const;

    StreamOutput drain();
    StreamStats stats() const;
    const StreamConfig& config() const { return cfg_; }

private:
    struct Vehicle;
    Vehicle& vehicle(const std::string& id);
    const Vehicle* find(const std::string& id) const;
    std::size_t release(Vehicle& v, bool all);
    void process(Vehicle& v, const TelemetryPoint& p);
    void flush_locked(Vehicle& v);

    StreamConfig cfg_;
    mutable std::shared_mutex vehicles_mu_;
    std::unordered_map<std::string, std::unique_ptr<Vehicle>> vehicles_;
    mutable std::shared_mutex index_mu_;
    GeofenceIndex index_;

    std::mutex out_mu_;
    StreamOutput pending_;

    std::atomic<std::size_t> ingested_{0}, processed_{0}, late_{0}, dups_{0};
};

}  // namespace fleetline
