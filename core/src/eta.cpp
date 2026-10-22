#include "fleetline/eta.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fleetline {

namespace {
constexpr double kDefaultSpeedMps = 11.0;  // ~40 km/h urban default
constexpr double kMinObservedMps = 1.0;
constexpr double kObservedHorizonM = 3000.0;  // observed speed matters mostly in the first few km
}  // namespace

Route::Route(std::vector<LatLon> points, std::vector<double> segment_speed_mps)
    : points_(std::move(points)), speeds_(std::move(segment_speed_mps)) {
    if (points_.size() < 2) throw std::invalid_argument("route needs at least 2 points");
    if (!speeds_.empty() && speeds_.size() != points_.size() - 1) {
        throw std::invalid_argument("segment speed profile must have one entry per segment");
    }
    for (double s : speeds_) {
        if (!(s > 0.0)) throw std::invalid_argument("segment speeds must be positive");
    }
    proj_ = LocalProjection(points_.front());
    xy_.reserve(points_.size());
    for (const auto& p : points_) xy_.push_back(proj_.to_xy(p));

    cumulative_.assign(points_.size(), 0.0);
    for (std::size_t i = 1; i < points_.size(); ++i) {
        cumulative_[i] = cumulative_[i - 1] + haversine_m(points_[i - 1], points_[i]);
    }
    seg_time_suffix_.assign(points_.size(), 0.0);
    for (std::size_t i = segments(); i-- > 0;) {
        const double len = cumulative_[i + 1] - cumulative_[i];
        seg_time_suffix_[i] = seg_time_suffix_[i + 1] + len / segment_speed(i);
    }
}

double Route::segment_speed(std::size_t i) const { return speeds_.empty() ? kDefaultSpeedMps : speeds_.at(i); }

RouteMatch Route::match(const LatLon& p, double min_offset_m, double backtrack_m) const {
    const auto q = proj_.to_xy(p);
    RouteMatch best;
    best.cross_track_m = std::numeric_limits<double>::max();
    const double floor_m = min_offset_m >= 0.0 ? min_offset_m - backtrack_m : -1.0;
    for (std::size_t i = 0; i < segments(); ++i) {
        if (cumulative_[i + 1] < floor_m) continue;
        const auto sp = project_on_segment(q, xy_[i], xy_[i + 1]);
        // Strict < keeps the earliest segment on ties (e.g. at a shared vertex).
        if (sp.distance < best.cross_track_m) {
            best.segment = i;
            best.t = sp.t;
            best.cross_track_m = sp.distance;
        }
    }
    const double seg_len = cumulative_[best.segment + 1] - cumulative_[best.segment];
    best.offset_m = cumulative_[best.segment] + best.t * seg_len;
    const auto& a = xy_[best.segment];
    const auto& b = xy_[best.segment + 1];
    best.snapped = proj_.to_latlon({a.x + best.t * (b.x - a.x), a.y + best.t * (b.y - a.y)});
    return best;
}

double Route::time_to_end_s(const RouteMatch& m) const {
    const std::size_t i = m.segment;
    const double seg_len = cumulative_[i + 1] - cumulative_[i];
    return (1.0 - m.t) * seg_len / segment_speed(i) + seg_time_suffix_[i + 1];
}

EtaResult Route::eta(const LatLon& p, double observed_speed_mps, double min_offset_m,
                     double off_route_threshold_m) const {
    EtaResult r;
    r.match = match(p, min_offset_m);
    r.off_route = r.match.cross_track_m > off_route_threshold_m;
    r.remaining_m = length_m() - r.match.offset_m;
    const double profile_s = time_to_end_s(r.match);
    if (r.off_route) {
        // Add the time to get back onto the route at the default speed.
        r.remaining_m += r.match.cross_track_m;
        r.eta_s = profile_s + r.match.cross_track_m / kDefaultSpeedMps;
        return r;
    }
    if (observed_speed_mps >= kMinObservedMps && r.remaining_m > 0.0) {
        const double w = std::exp(-r.remaining_m / kObservedHorizonM) * 0.6 + 0.2;
        const double profile_speed = r.remaining_m / std::max(profile_s, 1e-9);
        // Harmonic blend of speeds (= arithmetic blend of pace), so a very slow
        // observation lengthens the ETA proportionally rather than being washed out.
        const double pace = w / observed_speed_mps + (1.0 - w) / profile_speed;
        r.eta_s = r.remaining_m * pace;
    } else {
        r.eta_s = profile_s;
    }
    return r;
}

}  // namespace fleetline
