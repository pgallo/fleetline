#pragma once

#include <string>
#include <vector>

#include "fleetline/geo.hpp"
#include "fleetline/telemetry.hpp"

namespace fleetline::testutil {

inline const LatLon kOrigin{45.0, 9.0};

// Point at (east_m, north_m) metres from kOrigin.
inline LatLon offset(double east_m, double north_m) {
    return LocalProjection(kOrigin).to_latlon({east_m, north_m});
}

inline TelemetryPoint fix(const std::string& vid, double ts, LatLon p, double speed, bool ign, double heading = 90.0) {
    TelemetryPoint t;
    t.vehicle_id = vid;
    t.ts = ts;
    t.lat = p.lat;
    t.lon = p.lon;
    t.speed_mps = speed;
    t.heading_deg = heading;
    t.ignition = ign;
    return t;
}

inline FilteredPoint as_filtered(const TelemetryPoint& t) {
    FilteredPoint f;
    f.raw = t;
    f.pos = t.pos();
    f.speed_mps = t.speed_mps;
    f.filter_speed_mps = t.speed_mps;
    return f;
}

// A scripted day: park (ignition off), drive east at `speed` for `drive_s`,
// stop with engine idling for `idle_s`, then switch off and park.
inline std::vector<TelemetryPoint> scripted_trip(const std::string& vid, double t0, double speed, double drive_s,
                                                 double idle_s, double park_s = 600.0, double dt = 1.0) {
    std::vector<TelemetryPoint> out;
    double t = t0;
    double x = 0.0;
    for (double e = 0; e < park_s; e += 10.0, t += 10.0) out.push_back(fix(vid, t, offset(x, 0), 0.0, false));
    for (double e = 0; e < drive_s; e += dt, t += dt) {
        out.push_back(fix(vid, t, offset(x, 0), speed, true));
        x += speed * dt;
    }
    for (double e = 0; e < idle_s; e += dt, t += dt) out.push_back(fix(vid, t, offset(x, 0), 0.0, true));
    for (double e = 0; e < park_s; e += 10.0, t += 10.0) out.push_back(fix(vid, t, offset(x, 0), 0.0, false));
    return out;
}

}  // namespace fleetline::testutil
