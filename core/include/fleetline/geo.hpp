#pragma once

#include <cmath>

namespace fleetline {

inline constexpr double kEarthRadiusM = 6371008.8;
inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double deg2rad(double d) { return d * kPi / 180.0; }
inline constexpr double rad2deg(double r) { return r * 180.0 / kPi; }

struct LatLon {
    double lat = 0.0;
    double lon = 0.0;
};

// Great-circle distance in metres.
double haversine_m(const LatLon& a, const LatLon& b);

// Initial bearing from a to b, degrees clockwise from north in [0, 360).
double bearing_deg(const LatLon& a, const LatLon& b);

// Local tangent-plane projection (equirectangular around an anchor).
// Accurate to well under 0.1% within a few tens of kilometres, which is the
// scale of a city fleet. Everything metric (filtering, segment projection,
// boundary distance) happens in this plane.
class LocalProjection {
public:
    LocalProjection() = default;
    explicit LocalProjection(LatLon anchor);

    struct XY {
        double x = 0.0;  // east, metres
        double y = 0.0;  // north, metres
    };

    XY to_xy(const LatLon& p) const;
    LatLon to_latlon(const XY& p) const;
    const LatLon& anchor() const { return anchor_; }

private:
    LatLon anchor_{};
    double cos_lat0_ = 1.0;
};

// Projection of point p onto segment [a, b] in the plane.
struct SegmentProjection {
    double t = 0.0;         // clamped parameter in [0, 1]
    double distance = 0.0;  // perpendicular (or endpoint) distance
};

SegmentProjection project_on_segment(const LocalProjection::XY& p, const LocalProjection::XY& a,
                                     const LocalProjection::XY& b);

}  // namespace fleetline
