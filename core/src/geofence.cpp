#include "fleetline/geofence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fleetline {

Geofence Geofence::polygon(std::string id, std::string name, std::vector<LatLon> vertices, double dwell_s) {
    if (vertices.size() < 3) throw std::invalid_argument("polygon geofence needs at least 3 vertices");
    Geofence g;
    g.id = std::move(id);
    g.name = std::move(name);
    g.shape = FenceShape::Polygon;
    g.vertices = std::move(vertices);
    g.dwell_s = dwell_s;
    return g;
}

Geofence Geofence::circle(std::string id, std::string name, LatLon center, double radius_m, double dwell_s) {
    if (radius_m <= 0.0) throw std::invalid_argument("circle geofence needs a positive radius");
    Geofence g;
    g.id = std::move(id);
    g.name = std::move(name);
    g.shape = FenceShape::Circle;
    g.center = center;
    g.radius_m = radius_m;
    g.dwell_s = dwell_s;
    return g;
}

namespace {
// > 0 if p is left of the directed line a->b.
inline double is_left(const LatLon& a, const LatLon& b, const LatLon& p) {
    return (b.lon - a.lon) * (p.lat - a.lat) - (p.lon - a.lon) * (b.lat - a.lat);
}
}  // namespace

int winding_number(const LatLon& p, const std::vector<LatLon>& poly) {
    int wn = 0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const LatLon& a = poly[i];
        const LatLon& b = poly[(i + 1) % n];
        if (a.lat <= p.lat) {
            if (b.lat > p.lat && is_left(a, b, p) > 0) ++wn;  // upward crossing, p left of edge
        } else {
            if (b.lat <= p.lat && is_left(a, b, p) < 0) --wn;  // downward crossing, p right of edge
        }
    }
    return wn;
}

bool point_in_polygon(const LatLon& p, const std::vector<LatLon>& poly) { return winding_number(p, poly) != 0; }

PreparedFence::PreparedFence(Geofence fence) : fence_(std::move(fence)) {
    if (fence_.shape == FenceShape::Circle) {
        proj_ = LocalProjection(fence_.center);
        const double dlat = rad2deg(fence_.radius_m / kEarthRadiusM);
        const double dlon = dlat / std::cos(deg2rad(fence_.center.lat));
        min_lat_ = fence_.center.lat - dlat;
        max_lat_ = fence_.center.lat + dlat;
        min_lon_ = fence_.center.lon - dlon;
        max_lon_ = fence_.center.lon + dlon;
        return;
    }
    double slat = 0, slon = 0;
    min_lat_ = min_lon_ = std::numeric_limits<double>::max();
    max_lat_ = max_lon_ = std::numeric_limits<double>::lowest();
    for (const auto& v : fence_.vertices) {
        slat += v.lat;
        slon += v.lon;
        min_lat_ = std::min(min_lat_, v.lat);
        max_lat_ = std::max(max_lat_, v.lat);
        min_lon_ = std::min(min_lon_, v.lon);
        max_lon_ = std::max(max_lon_, v.lon);
    }
    const double n = static_cast<double>(fence_.vertices.size());
    proj_ = LocalProjection({slat / n, slon / n});
    xy_.reserve(fence_.vertices.size());
    for (const auto& v : fence_.vertices) xy_.push_back(proj_.to_xy(v));
}

bool PreparedFence::contains(const LatLon& p) const {
    if (fence_.shape == FenceShape::Circle) return haversine_m(p, fence_.center) <= fence_.radius_m;
    if (p.lat < min_lat_ || p.lat > max_lat_ || p.lon < min_lon_ || p.lon > max_lon_) return false;
    return point_in_polygon(p, fence_.vertices);
}

double PreparedFence::signed_distance_m(const LatLon& p) const {
    if (fence_.shape == FenceShape::Circle) return haversine_m(p, fence_.center) - fence_.radius_m;
    const auto q = proj_.to_xy(p);
    double best = std::numeric_limits<double>::max();
    const std::size_t n = xy_.size();
    for (std::size_t i = 0; i < n; ++i) {
        best = std::min(best, project_on_segment(q, xy_[i], xy_[(i + 1) % n]).distance);
    }
    return point_in_polygon(p, fence_.vertices) ? -best : best;
}

GeofenceIndex::GeofenceIndex(double cell_deg, double margin_m) : cell_deg_(cell_deg), margin_m_(margin_m) {
    if (cell_deg_ <= 0.0) throw std::invalid_argument("cell_deg must be positive");
}

std::int64_t GeofenceIndex::cell_of(double deg) const {
    return static_cast<std::int64_t>(std::floor(deg / cell_deg_));
}

void GeofenceIndex::add(Geofence fence) {
    std::string id = fence.id;
    remove(id);
    auto [it, _] = fences_.emplace(id, PreparedFence(std::move(fence)));
    const PreparedFence& pf = it->second;
    // Expand by the hysteresis margin so a vehicle just outside a fence (but
    // still "inside" by hysteresis) keeps seeing it as a candidate.
    const double mlat = rad2deg(margin_m_ / kEarthRadiusM);
    const double mlon = mlat / std::max(0.01, std::cos(deg2rad(0.5 * (pf.min_lat() + pf.max_lat()))));
    for (auto ix = cell_of(pf.min_lon() - mlon); ix <= cell_of(pf.max_lon() + mlon); ++ix) {
        for (auto iy = cell_of(pf.min_lat() - mlat); iy <= cell_of(pf.max_lat() + mlat); ++iy) {
            grid_[key(ix, iy)].push_back(id);
        }
    }
}

bool GeofenceIndex::remove(const std::string& id) {
    if (fences_.erase(id) == 0) return false;
    for (auto it = grid_.begin(); it != grid_.end();) {
        auto& ids = it->second;
        ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
        it = ids.empty() ? grid_.erase(it) : std::next(it);
    }
    return true;
}

std::vector<const PreparedFence*> GeofenceIndex::candidates(const LatLon& p) const {
    std::vector<const PreparedFence*> out;
    auto it = grid_.find(key(cell_of(p.lon), cell_of(p.lat)));
    if (it == grid_.end()) return out;
    out.reserve(it->second.size());
    for (const auto& id : it->second) out.push_back(&fences_.at(id));
    return out;
}

const PreparedFence* GeofenceIndex::get(const std::string& id) const {
    auto it = fences_.find(id);
    return it == fences_.end() ? nullptr : &it->second;
}

std::vector<const Geofence*> GeofenceIndex::all() const {
    std::vector<const Geofence*> out;
    out.reserve(fences_.size());
    for (const auto& [_, pf] : fences_) out.push_back(&pf.fence());
    std::sort(out.begin(), out.end(), [](auto* a, auto* b) { return a->id < b->id; });
    return out;
}

const char* to_string(GeofenceEventType t) {
    switch (t) {
        case GeofenceEventType::Enter: return "enter";
        case GeofenceEventType::Exit: return "exit";
        case GeofenceEventType::Dwell: return "dwell";
    }
    return "unknown";
}

std::vector<GeofenceEvent> GeofenceTracker::update(const std::string& vehicle_id, double ts, const LatLon& pos,
                                                   const GeofenceIndex& index) {
    std::vector<GeofenceEvent> events;
    std::unordered_set<std::string> seen;

    for (const PreparedFence* pf : index.candidates(pos)) {
        const std::string& fid = pf->fence().id;
        seen.insert(fid);
        const double sd = pf->signed_distance_m(pos);
        auto it = inside_.find(fid);
        if (it == inside_.end()) {
            if (sd <= -hysteresis_m_) {
                inside_.emplace(fid, Visit{ts, false});
                events.push_back({vehicle_id, fid, GeofenceEventType::Enter, ts, pos, 0.0});
            }
            continue;
        }
        Visit& v = it->second;
        if (sd >= hysteresis_m_) {
            events.push_back({vehicle_id, fid, GeofenceEventType::Exit, ts, pos, ts - v.entered_ts});
            inside_.erase(it);
        } else if (!v.dwell_emitted && ts - v.entered_ts >= pf->fence().dwell_s) {
            v.dwell_emitted = true;
            events.push_back({vehicle_id, fid, GeofenceEventType::Dwell, ts, pos, ts - v.entered_ts});
        }
    }

    // Fences we are inside but which the grid did not return: the vehicle has
    // left the fence's expanded bbox entirely (or the fence was removed).
    for (auto it = inside_.begin(); it != inside_.end();) {
        if (seen.count(it->first)) {
            ++it;
            continue;
        }
        events.push_back({vehicle_id, it->first, GeofenceEventType::Exit, ts, pos, ts - it->second.entered_ts});
        it = inside_.erase(it);
    }
    return events;
}

std::vector<std::string> GeofenceTracker::inside() const {
    std::vector<std::string> out;
    out.reserve(inside_.size());
    for (const auto& [id, _] : inside_) out.push_back(id);
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace fleetline
