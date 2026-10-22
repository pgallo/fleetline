#pragma once

#include <string>

#include "fleetline/geo.hpp"

namespace fleetline {

// One GPS fix as reported by a vehicle tracker.
// ts is seconds since 1970-01-01T00:00:00Z (fractional seconds allowed).
// speed_mps < 0 means "device did not report speed".
struct TelemetryPoint {
    std::string vehicle_id;
    double ts = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double speed_mps = -1.0;
    double heading_deg = 0.0;
    bool ignition = false;

    LatLon pos() const { return {lat, lon}; }
};

// A point after filtering: smoothed position plus the filter's velocity estimate.
struct FilteredPoint {
    TelemetryPoint raw;
    LatLon pos;
    double speed_mps = 0.0;  // best available speed (device if reported, else filter)
    double filter_speed_mps = 0.0;
};

}  // namespace fleetline
