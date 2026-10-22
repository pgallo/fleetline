#pragma once

#include <optional>

#include "fleetline/geo.hpp"
#include "fleetline/telemetry.hpp"

namespace fleetline {

struct FilterConfig {
    double gps_sigma_m = 8.0;          // measurement noise (1 sigma, per axis)
    double accel_sigma_mps2 = 1.5;     // process noise: white-noise acceleration
    double max_implied_speed_mps = 70.0;  // ~250 km/h: anything faster is a GPS jump
    int reset_after_rejections = 3;    // consecutive rejections before trusting the new track
    double max_gap_s = 300.0;          // gap after which the filter restarts from scratch
};

// Constant-velocity Kalman filter on a local tangent plane.
//
// The two axes are filtered independently: with a diagonal measurement
// covariance and the same motion model on both axes, the 4x4 filter is block
// diagonal, so two 2x2 filters give identical results with less arithmetic.
class PositionFilter {
public:
    explicit PositionFilter(FilterConfig cfg = {});

    // Returns the filtered point, or std::nullopt if the fix was rejected
    // as an outlier. Points must be fed in timestamp order.
    std::optional<FilteredPoint> update(const TelemetryPoint& p);

    std::size_t rejected() const { return rejected_total_; }
    void reset();

private:
    struct Axis {
        double p = 0.0, v = 0.0;
        double P00 = 0.0, P01 = 0.0, P11 = 0.0;
        void init(double pos, double pos_var, double vel_var);
        void predict(double dt, double q);
        void correct(double z, double r);
    };

    void initialise(const TelemetryPoint& p);

    FilterConfig cfg_;
    bool initialised_ = false;
    LocalProjection proj_;
    Axis ax_, ay_;
    double last_ts_ = 0.0;
    LatLon last_accepted_raw_{};
    double last_accepted_ts_ = 0.0;
    int consecutive_rejections_ = 0;
    std::size_t rejected_total_ = 0;
};

}  // namespace fleetline
