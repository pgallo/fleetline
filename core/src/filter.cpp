#include "fleetline/filter.hpp"

#include <cmath>

namespace fleetline {

void PositionFilter::Axis::init(double pos, double pos_var, double vel_var) {
    p = pos;
    v = 0.0;
    P00 = pos_var;
    P01 = 0.0;
    P11 = vel_var;
}

void PositionFilter::Axis::predict(double dt, double q) {
    // x' = F x, P' = F P F^T + Q with F = [[1, dt], [0, 1]] and the discrete
    // white-noise-acceleration Q = q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]].
    p += v * dt;
    const double dt2 = dt * dt;
    const double n00 = P00 + 2.0 * dt * P01 + dt2 * P11 + q * dt2 * dt2 / 4.0;
    const double n01 = P01 + dt * P11 + q * dt2 * dt / 2.0;
    const double n11 = P11 + q * dt2;
    P00 = n00;
    P01 = n01;
    P11 = n11;
}

void PositionFilter::Axis::correct(double z, double r) {
    // H = [1, 0]
    const double s = P00 + r;
    const double k0 = P00 / s;
    const double k1 = P01 / s;
    const double y = z - p;
    p += k0 * y;
    v += k1 * y;
    const double n00 = (1.0 - k0) * P00;
    const double n01 = (1.0 - k0) * P01;
    const double n11 = P11 - k1 * P01;
    P00 = n00;
    P01 = n01;
    P11 = n11;
}

PositionFilter::PositionFilter(FilterConfig cfg) : cfg_(cfg) {}

void PositionFilter::reset() {
    initialised_ = false;
    consecutive_rejections_ = 0;
}

void PositionFilter::initialise(const TelemetryPoint& pt) {
    proj_ = LocalProjection(pt.pos());
    const double r = cfg_.gps_sigma_m * cfg_.gps_sigma_m;
    ax_.init(0.0, r, 100.0);
    ay_.init(0.0, r, 100.0);
    // Seed velocity from the device-reported speed and heading when present;
    // it shortens the filter's warm-up considerably.
    if (pt.speed_mps >= 0.0) {
        const double h = deg2rad(pt.heading_deg);
        ax_.v = pt.speed_mps * std::sin(h);
        ay_.v = pt.speed_mps * std::cos(h);
        ax_.P11 = ay_.P11 = 4.0;
    }
    last_ts_ = pt.ts;
    last_accepted_raw_ = pt.pos();
    last_accepted_ts_ = pt.ts;
    consecutive_rejections_ = 0;
    initialised_ = true;
}

std::optional<FilteredPoint> PositionFilter::update(const TelemetryPoint& pt) {
    if (!initialised_ || pt.ts - last_ts_ > cfg_.max_gap_s || pt.ts < last_ts_) {
        initialise(pt);
    } else {
        // Outlier gate on implied speed against the last accepted raw fix.
        // Raw-to-raw (not filtered-to-raw) keeps the gate independent of the
        // filter's own lag during hard manoeuvres.
        const double dt_acc = pt.ts - last_accepted_ts_;
        const double d = haversine_m(last_accepted_raw_, pt.pos());
        // Allow for two fixes' worth of GPS noise before computing speed.
        const double slack = 3.0 * cfg_.gps_sigma_m;
        const double implied = dt_acc > 0.0 ? std::max(0.0, d - slack) / dt_acc : (d > slack ? 1e9 : 0.0);
        if (implied > cfg_.max_implied_speed_mps) {
            ++rejected_total_;
            if (++consecutive_rejections_ < cfg_.reset_after_rejections) {
                return std::nullopt;
            }
            // Several "outliers" in a row: the earlier track was probably the
            // bad one (e.g. a cold-start fix). Restart on the new evidence.
            initialise(pt);
        } else {
            const double dt = pt.ts - last_ts_;
            const double q = cfg_.accel_sigma_mps2 * cfg_.accel_sigma_mps2;
            const double r = cfg_.gps_sigma_m * cfg_.gps_sigma_m;
            const auto z = proj_.to_xy(pt.pos());
            if (dt > 0.0) {
                ax_.predict(dt, q);
                ay_.predict(dt, q);
            }
            ax_.correct(z.x, r);
            ay_.correct(z.y, r);
            last_ts_ = pt.ts;
            last_accepted_raw_ = pt.pos();
            last_accepted_ts_ = pt.ts;
            consecutive_rejections_ = 0;
        }
    }

    FilteredPoint out;
    out.raw = pt;
    out.pos = proj_.to_latlon({ax_.p, ay_.p});
    out.filter_speed_mps = std::hypot(ax_.v, ay_.v);
    out.speed_mps = pt.speed_mps >= 0.0 ? pt.speed_mps : out.filter_speed_mps;
    return out;
}

}  // namespace fleetline
