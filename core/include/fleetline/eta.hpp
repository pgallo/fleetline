#pragma once

#include <vector>

#include "fleetline/geo.hpp"

namespace fleetline {

// Snap result of a position onto a route polyline ("map-matching-lite": the
// route is known, so matching reduces to finding the nearest segment, biased
// towards progress already made).
struct RouteMatch {
    std::size_t segment = 0;     // index i of segment [i, i+1]
    double t = 0.0;              // position along that segment in [0, 1]
    double offset_m = 0.0;       // distance from route start along the polyline
    double cross_track_m = 0.0;  // distance from the polyline
    LatLon snapped{};
};

struct EtaResult {
    double remaining_m = 0.0;
    double eta_s = 0.0;
    RouteMatch match{};
    bool off_route = false;
};

// A route as a polyline with an optional per-segment expected speed (the
// "speed profile": e.g. 13 m/s on urban arterials, 25 m/s on the ring road).
class Route {
public:
    Route(std::vector<LatLon> points, std::vector<double> segment_speed_mps = {});

    double length_m() const { return cumulative_.back(); }
    std::size_t segments() const { return points_.size() - 1; }
    const std::vector<LatLon>& points() const { return points_; }
    double segment_speed(std::size_t i) const;

    // Nearest-segment match. If min_offset_m >= 0, segments that end before
    // (min_offset_m - backtrack_m) are skipped: on routes that pass the same
    // street twice, this keeps the match moving forward.
    RouteMatch match(const LatLon& p, double min_offset_m = -1.0, double backtrack_m = 200.0) const;

    // Free-flow time from a matched position to the end of the route.
    double time_to_end_s(const RouteMatch& m) const;

    // ETA from position p. The free-flow profile time is blended with the
    // vehicle's observed recent speed: the profile says how fast the road
    // allows, the observed speed says how fast traffic actually moves now.
    // The weight of the observed component decays from 0.8 to 0.2 with
    // remaining distance (observations say little about traffic 20 km ahead).
    EtaResult eta(const LatLon& p, double observed_speed_mps = -1.0, double min_offset_m = -1.0,
                  double off_route_threshold_m = 75.0) const;

private:
    std::vector<LatLon> points_;
    std::vector<double> speeds_;
    std::vector<double> cumulative_;
    std::vector<double> seg_time_suffix_;  // free-flow time from start of segment i to end
    LocalProjection proj_;
    std::vector<LocalProjection::XY> xy_;
};

}  // namespace fleetline
