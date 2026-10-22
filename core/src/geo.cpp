#include "fleetline/geo.hpp"

#include <algorithm>

namespace fleetline {

double haversine_m(const LatLon& a, const LatLon& b) {
    const double dlat = deg2rad(b.lat - a.lat);
    const double dlon = deg2rad(b.lon - a.lon);
    const double s1 = std::sin(dlat / 2.0);
    const double s2 = std::sin(dlon / 2.0);
    const double h = s1 * s1 + std::cos(deg2rad(a.lat)) * std::cos(deg2rad(b.lat)) * s2 * s2;
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

double bearing_deg(const LatLon& a, const LatLon& b) {
    const double phi1 = deg2rad(a.lat);
    const double phi2 = deg2rad(b.lat);
    const double dlon = deg2rad(b.lon - a.lon);
    const double y = std::sin(dlon) * std::cos(phi2);
    const double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlon);
    const double deg = rad2deg(std::atan2(y, x));
    return std::fmod(deg + 360.0, 360.0);
}

LocalProjection::LocalProjection(LatLon anchor) : anchor_(anchor), cos_lat0_(std::cos(deg2rad(anchor.lat))) {}

LocalProjection::XY LocalProjection::to_xy(const LatLon& p) const {
    return {deg2rad(p.lon - anchor_.lon) * kEarthRadiusM * cos_lat0_, deg2rad(p.lat - anchor_.lat) * kEarthRadiusM};
}

LatLon LocalProjection::to_latlon(const XY& p) const {
    return {anchor_.lat + rad2deg(p.y / kEarthRadiusM), anchor_.lon + rad2deg(p.x / (kEarthRadiusM * cos_lat0_))};
}

SegmentProjection project_on_segment(const LocalProjection::XY& p, const LocalProjection::XY& a,
                                     const LocalProjection::XY& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 0.0) {
        t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
    }
    const double qx = a.x + t * dx - p.x;
    const double qy = a.y + t * dy - p.y;
    return {t, std::sqrt(qx * qx + qy * qy)};
}

}  // namespace fleetline
