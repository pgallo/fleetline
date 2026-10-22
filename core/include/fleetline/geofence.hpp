#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "fleetline/geo.hpp"

namespace fleetline {

enum class FenceShape { Polygon, Circle };

struct Geofence {
    std::string id;
    std::string name;
    FenceShape shape = FenceShape::Polygon;
    std::vector<LatLon> vertices;  // polygon, implicit closing edge
    LatLon center{};               // circle
    double radius_m = 0.0;         // circle
    double dwell_s = 300.0;        // inside this long -> one Dwell event per visit

    static Geofence polygon(std::string id, std::string name, std::vector<LatLon> vertices, double dwell_s = 300.0);
    static Geofence circle(std::string id, std::string name, LatLon center, double radius_m, double dwell_s = 300.0);
};

// Winding-number point-in-polygon on raw lat/lon (planar approximation, fine at
// city scale and away from the antimeridian). Non-zero winding = inside, which
// also gives the intuitive answer for self-overlapping shapes.
int winding_number(const LatLon& p, const std::vector<LatLon>& poly);
bool point_in_polygon(const LatLon& p, const std::vector<LatLon>& poly);

// Geofence prepared for fast queries: projected vertices and bounding box.
class PreparedFence {
public:
    explicit PreparedFence(Geofence fence);

    const Geofence& fence() const { return fence_; }
    bool contains(const LatLon& p) const;
    // Signed distance to the boundary in metres: negative inside, positive outside.
    double signed_distance_m(const LatLon& p) const;

    double min_lat() const { return min_lat_; }
    double max_lat() const { return max_lat_; }
    double min_lon() const { return min_lon_; }
    double max_lon() const { return max_lon_; }

private:
    Geofence fence_;
    LocalProjection proj_;
    std::vector<LocalProjection::XY> xy_;
    double min_lat_ = 0, max_lat_ = 0, min_lon_ = 0, max_lon_ = 0;
};

// Uniform grid over lat/lon. Each fence is registered in every cell its
// (margin-expanded) bounding box overlaps; a query touches exactly one cell.
// For a city with hundreds to low thousands of fences of similar size, this
// beats an R-tree on simplicity and constant factors; see README.
class GeofenceIndex {
public:
    explicit GeofenceIndex(double cell_deg = 0.01, double margin_m = 50.0);

    void add(Geofence fence);
    bool remove(const std::string& id);
    std::size_t size() const { return fences_.size(); }

    // Fences whose expanded bbox covers p. A superset of the fences within
    // margin_m of p; callers do the exact test.
    std::vector<const PreparedFence*> candidates(const LatLon& p) const;
    const PreparedFence* get(const std::string& id) const;
    std::vector<const Geofence*> all() const;
    double margin_m() const { return margin_m_; }

private:
    using CellKey = std::int64_t;
    CellKey key(std::int64_t ix, std::int64_t iy) const { return (ix << 32) ^ (iy & 0xffffffff); }
    std::int64_t cell_of(double deg) const;
    void reindex();

    double cell_deg_;
    double margin_m_;
    std::unordered_map<std::string, PreparedFence> fences_;
    std::unordered_map<CellKey, std::vector<std::string>> grid_;
};

enum class GeofenceEventType { Enter, Exit, Dwell };

struct GeofenceEvent {
    std::string vehicle_id;
    std::string fence_id;
    GeofenceEventType type = GeofenceEventType::Enter;
    double ts = 0.0;
    LatLon pos{};
    double dwell_s = 0.0;  // for Exit and Dwell: time spent inside so far
};

const char* to_string(GeofenceEventType t);

// Per-vehicle enter/exit/dwell state machine with spatial hysteresis.
//
// A vehicle enters a fence only once it is at least `hysteresis_m` inside the
// boundary, and exits only once it is at least `hysteresis_m` outside. GPS noise
// near an edge therefore cannot produce enter/exit storms.
class GeofenceTracker {
public:
    explicit GeofenceTracker(double hysteresis_m = 15.0) : hysteresis_m_(hysteresis_m) {}

    std::vector<GeofenceEvent> update(const std::string& vehicle_id, double ts, const LatLon& pos,
                                      const GeofenceIndex& index);
    std::vector<std::string> inside() const;

private:
    struct Visit {
        double entered_ts = 0.0;
        bool dwell_emitted = false;
    };
    double hysteresis_m_;
    std::unordered_map<std::string, Visit> inside_;
};

}  // namespace fleetline
