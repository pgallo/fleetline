#include "fleetline/stream.hpp"

#include <algorithm>
#include <deque>
#include <functional>

namespace fleetline {

namespace {
struct LaterFirst {
    bool operator()(const TelemetryPoint& a, const TelemetryPoint& b) const { return a.ts > b.ts; }
};
constexpr double kRecentKeepS = 900.0;
}  // namespace

struct StreamProcessor::Vehicle {
    Vehicle(const std::string& id, const StreamConfig& cfg)
        : id(id), filter(cfg.filter), segmenter(id, cfg.trips), tracker(cfg.geofence_hysteresis_m) {}

    std::string id;
    mutable std::mutex mu;
    std::priority_queue<TelemetryPoint, std::vector<TelemetryPoint>, LaterFirst> buffer;
    double max_ts = -1e18;
    double last_processed_ts = -1e18;
    PositionFilter filter;
    TripSegmenter segmenter;
    GeofenceTracker tracker;
    std::optional<FilteredPoint> last;
    std::deque<std::pair<double, double>> recent;  // (ts, speed)
    std::vector<Trip> trips;
    std::vector<Stop> stops;
    std::vector<GeofenceEvent> events;
};

StreamProcessor::StreamProcessor(StreamConfig cfg)
    : cfg_(cfg), index_(cfg.geofence_cell_deg, cfg.geofence_hysteresis_m * 2.0) {}

StreamProcessor::~StreamProcessor() = default;

void StreamProcessor::add_geofence(Geofence fence) {
    std::unique_lock lk(index_mu_);
    index_.add(std::move(fence));
}

bool StreamProcessor::remove_geofence(const std::string& id) {
    std::unique_lock lk(index_mu_);
    return index_.remove(id);
}

std::vector<Geofence> StreamProcessor::geofences() const {
    std::shared_lock lk(index_mu_);
    std::vector<Geofence> out;
    for (const Geofence* g : index_.all()) out.push_back(*g);
    return out;
}

StreamProcessor::Vehicle& StreamProcessor::vehicle(const std::string& id) {
    {
        std::shared_lock lk(vehicles_mu_);
        auto it = vehicles_.find(id);
        if (it != vehicles_.end()) return *it->second;
    }
    std::unique_lock lk(vehicles_mu_);
    auto& slot = vehicles_[id];  // re-check: another thread may have inserted it
    if (!slot) slot = std::make_unique<Vehicle>(id, cfg_);
    return *slot;  // unique_ptr keeps the address stable across rehashes
}

const StreamProcessor::Vehicle* StreamProcessor::find(const std::string& id) const {
    std::shared_lock lk(vehicles_mu_);
    auto it = vehicles_.find(id);
    return it == vehicles_.end() ? nullptr : it->second.get();
}

std::size_t StreamProcessor::ingest(const TelemetryPoint& p) {
    ingested_.fetch_add(1, std::memory_order_relaxed);
    Vehicle& v = vehicle(p.vehicle_id);
    std::lock_guard lk(v.mu);
    if (p.ts <= v.last_processed_ts) {
        (p.ts == v.last_processed_ts ? dups_ : late_).fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    v.buffer.push(p);
    v.max_ts = std::max(v.max_ts, p.ts);
    return release(v, false);
}

std::size_t StreamProcessor::ingest_batch(const std::vector<TelemetryPoint>& pts) {
    std::size_t n = 0;
    for (const auto& p : pts) n += ingest(p);
    return n;
}

std::size_t StreamProcessor::release(Vehicle& v, bool all) {
    const double watermark = v.max_ts - cfg_.reorder_window_s;
    std::size_t n = 0;
    while (!v.buffer.empty()) {
        const bool overflow = v.buffer.size() > cfg_.max_buffer;
        if (!all && !overflow && v.buffer.top().ts > watermark) break;
        TelemetryPoint p = v.buffer.top();
        v.buffer.pop();
        if (p.ts <= v.last_processed_ts) {
            (p.ts == v.last_processed_ts ? dups_ : late_).fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        process(v, p);
        ++n;
    }
    return n;
}

void StreamProcessor::process(Vehicle& v, const TelemetryPoint& p) {
    v.last_processed_ts = p.ts;
    processed_.fetch_add(1, std::memory_order_relaxed);

    auto fp = v.filter.update(p);
    if (!fp) return;  // outlier

    v.last = *fp;
    v.recent.emplace_back(fp->raw.ts, fp->speed_mps);
    while (!v.recent.empty() && v.recent.front().first < fp->raw.ts - kRecentKeepS) v.recent.pop_front();

    SegmentOutput seg = v.segmenter.update(*fp);
    std::vector<GeofenceEvent> ev;
    {
        std::shared_lock lk(index_mu_);
        ev = v.tracker.update(v.id, fp->raw.ts, fp->pos, index_);
    }

    v.trips.insert(v.trips.end(), seg.trips.begin(), seg.trips.end());
    v.stops.insert(v.stops.end(), seg.stops.begin(), seg.stops.end());
    v.events.insert(v.events.end(), ev.begin(), ev.end());
    if (!seg.trips.empty() || !seg.stops.empty() || !ev.empty()) {
        std::lock_guard lk(out_mu_);
        pending_.trips.insert(pending_.trips.end(), seg.trips.begin(), seg.trips.end());
        pending_.stops.insert(pending_.stops.end(), seg.stops.begin(), seg.stops.end());
        pending_.geofence_events.insert(pending_.geofence_events.end(), ev.begin(), ev.end());
    }
}

void StreamProcessor::flush_locked(Vehicle& v) {
    release(v, true);
    SegmentOutput seg = v.segmenter.flush();
    v.trips.insert(v.trips.end(), seg.trips.begin(), seg.trips.end());
    v.stops.insert(v.stops.end(), seg.stops.begin(), seg.stops.end());
    std::lock_guard lk(out_mu_);
    pending_.trips.insert(pending_.trips.end(), seg.trips.begin(), seg.trips.end());
    pending_.stops.insert(pending_.stops.end(), seg.stops.begin(), seg.stops.end());
}

void StreamProcessor::flush() {
    std::vector<Vehicle*> all;
    {
        std::shared_lock lk(vehicles_mu_);
        for (auto& [_, v] : vehicles_) all.push_back(v.get());
    }
    for (Vehicle* v : all) {
        std::lock_guard lk(v->mu);
        flush_locked(*v);
    }
}

void StreamProcessor::flush_vehicle(const std::string& vehicle_id) {
    const Vehicle* cv = find(vehicle_id);
    if (!cv) return;
    auto* v = const_cast<Vehicle*>(cv);
    std::lock_guard lk(v->mu);
    flush_locked(*v);
}

std::vector<std::string> StreamProcessor::vehicles() const {
    std::shared_lock lk(vehicles_mu_);
    std::vector<std::string> out;
    out.reserve(vehicles_.size());
    for (const auto& [id, _] : vehicles_) out.push_back(id);
    std::sort(out.begin(), out.end());
    return out;
}

std::optional<VehicleStatus> StreamProcessor::status(const std::string& vehicle_id) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return std::nullopt;
    std::lock_guard lk(v->mu);
    VehicleStatus s;
    s.vehicle_id = v->id;
    s.state = v->segmenter.state();
    s.inside_fences = v->tracker.inside();
    s.trips = v->trips.size();
    s.buffered = v->buffer.size();
    if (v->last) {
        s.last_ts = v->last->raw.ts;
        s.lat = v->last->pos.lat;
        s.lon = v->last->pos.lon;
        s.speed_mps = v->last->speed_mps;
        s.heading_deg = v->last->raw.heading_deg;
        s.ignition = v->last->raw.ignition;
    }
    return s;
}

std::vector<Trip> StreamProcessor::trips(const std::string& vehicle_id) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return {};
    std::lock_guard lk(v->mu);
    return v->trips;
}

std::vector<Stop> StreamProcessor::stops(const std::string& vehicle_id) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return {};
    std::lock_guard lk(v->mu);
    return v->stops;
}

std::vector<GeofenceEvent> StreamProcessor::geofence_events(const std::string& vehicle_id) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return {};
    std::lock_guard lk(v->mu);
    return v->events;
}

std::optional<FilteredPoint> StreamProcessor::last_point(const std::string& vehicle_id) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return std::nullopt;
    std::lock_guard lk(v->mu);
    return v->last;
}

double StreamProcessor::recent_speed(const std::string& vehicle_id, double window_s) const {
    const Vehicle* v = find(vehicle_id);
    if (!v) return -1.0;
    std::lock_guard lk(v->mu);
    if (v->recent.empty()) return -1.0;
    const double cutoff = v->recent.back().first - window_s;
    double sum = 0.0;
    std::size_t n = 0;
    for (auto it = v->recent.rbegin(); it != v->recent.rend() && it->first >= cutoff; ++it) {
        sum += it->second;
        ++n;
    }
    return n ? sum / static_cast<double>(n) : -1.0;
}

StreamOutput StreamProcessor::drain() {
    std::lock_guard lk(out_mu_);
    StreamOutput out = std::move(pending_);
    pending_ = StreamOutput{};
    return out;
}

StreamStats StreamProcessor::stats() const {
    StreamStats s;
    s.ingested = ingested_.load();
    s.processed = processed_.load();
    s.late_dropped = late_.load();
    s.duplicates = dups_.load();
    std::shared_lock lk(vehicles_mu_);
    s.vehicles = vehicles_.size();
    for (const auto& [_, v] : vehicles_) {
        std::lock_guard vl(v->mu);
        s.outliers += v->filter.rejected();
    }
    return s;
}

}  // namespace fleetline
