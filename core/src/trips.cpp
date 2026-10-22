#include "fleetline/trips.hpp"

#include <algorithm>

namespace fleetline {

const char* to_string(HarshType t) { return t == HarshType::Braking ? "braking" : "acceleration"; }

const char* to_string(MotionState s) {
    switch (s) {
        case MotionState::Parked: return "parked";
        case MotionState::Driving: return "driving";
        case MotionState::Pausing: return "pausing";
    }
    return "unknown";
}

TripSegmenter::TripSegmenter(std::string vehicle_id, TripConfig cfg)
    : vehicle_id_(std::move(vehicle_id)), cfg_(cfg) {}

void TripSegmenter::close_trip(SegmentOutput& out) {
    Trip t = std::move(*trip_);
    trip_.reset();
    t.end_ts = pause_start_ts_;
    t.end_pos = pause_pos_;
    if (t.distance_m >= cfg_.min_trip_distance_m) {
        out.trips.push_back(std::move(t));
        stop_open_ = true;
        stop_ = Stop{vehicle_id_, pause_start_ts_, pause_start_ts_, pause_pos_, pause_idle_s_};
    } else if (stop_open_) {
        // Too short to be a trip: fold it back into the surrounding stop.
        stop_.idle_s += t.idle_s + pause_idle_s_;
    } else {
        stop_open_ = true;
        stop_ = Stop{vehicle_id_, t.start_ts, pause_start_ts_, t.start_pos, t.idle_s + pause_idle_s_};
    }
    state_ = MotionState::Parked;
}

SegmentOutput TripSegmenter::update(const FilteredPoint& p) {
    SegmentOutput out;
    const double ts = p.raw.ts;
    const bool ign = p.raw.ignition;
    const double v = p.speed_mps;

    double dt = 0.0;
    bool stationary_idle = false;
    if (prev_) {
        dt = ts - prev_->raw.ts;
        if (dt < 0.0 || dt > cfg_.max_interval_s) dt = 0.0;
        stationary_idle = prev_->raw.ignition && ign && prev_->speed_mps < cfg_.stop_speed_mps &&
                          v < cfg_.stop_speed_mps;
    }

    switch (state_) {
        case MotionState::Parked: {
            if (stop_open_ && stationary_idle) stop_.idle_s += dt;
            if (ign && v >= cfg_.move_speed_mps) {
                if (stop_open_) {
                    stop_.end_ts = prev_ ? prev_->raw.ts : ts;
                    out.stops.push_back(stop_);
                    stop_open_ = false;
                }
                Trip t;
                t.vehicle_id = vehicle_id_;
                // Start at the last stationary fix when it is recent: the
                // vehicle pulled away somewhere in between.
                const bool use_prev = prev_ && dt > 0.0;
                t.start_ts = use_prev ? prev_->raw.ts : ts;
                t.start_pos = use_prev ? prev_->pos : p.pos;
                t.points = use_prev ? 1 : 0;
                trip_ = std::move(t);
                ignition_off_since_ = -1.0;
                state_ = MotionState::Driving;
            } else {
                break;
            }
            [[fallthrough]];
        }
        case MotionState::Driving:
        case MotionState::Pausing: {
            Trip& t = *trip_;
            t.points += 1;
            t.end_ts = ts;
            t.end_pos = p.pos;
            t.max_speed_mps = std::max(t.max_speed_mps, v);
            if (prev_ && dt > 0.0) {
                const bool moving = v >= cfg_.stop_speed_mps || prev_->speed_mps >= cfg_.stop_speed_mps;
                if (moving) t.distance_m += haversine_m(prev_->pos, p.pos);

                if (dt <= cfg_.max_accel_dt_s) {
                    const double a = (v - prev_->speed_mps) / dt;
                    auto maybe_emit = [&](HarshType type) {
                        auto& last = last_harsh_ts_[static_cast<int>(type)];
                        if (ts - last >= cfg_.harsh_debounce_s) {
                            t.harsh_events.push_back({type, ts, p.pos, a});
                            last = ts;
                        }
                    };
                    if (a <= -cfg_.harsh_brake_mps2) maybe_emit(HarshType::Braking);
                    if (a >= cfg_.harsh_accel_mps2) maybe_emit(HarshType::Acceleration);
                }
            }

            ignition_off_since_ = ign ? -1.0 : (ignition_off_since_ < 0.0 ? ts : ignition_off_since_);
            const bool stationary = !ign || v < cfg_.stop_speed_mps;

            if (state_ == MotionState::Driving) {
                if (stationary) {
                    state_ = MotionState::Pausing;
                    pause_start_ts_ = ts;
                    pause_pos_ = p.pos;
                    pause_idle_s_ = 0.0;
                }
            } else {  // Pausing
                if (stationary_idle) pause_idle_s_ += dt;
                if (ign && v >= cfg_.move_speed_mps) {
                    t.idle_s += pause_idle_s_;  // a short stop is part of the trip
                    pause_idle_s_ = 0.0;
                    state_ = MotionState::Driving;
                } else {
                    const bool long_stop = ts - pause_start_ts_ >= cfg_.min_stop_s;
                    const bool switched_off =
                        ignition_off_since_ >= 0.0 && ts - ignition_off_since_ >= cfg_.ignition_off_stop_s;
                    if (long_stop || switched_off) close_trip(out);
                }
            }
            break;
        }
    }

    prev_ = p;
    return out;
}

SegmentOutput TripSegmenter::flush() {
    SegmentOutput out;
    if (trip_) {
        if (state_ == MotionState::Driving && prev_) {
            pause_start_ts_ = prev_->raw.ts;
            pause_pos_ = prev_->pos;
            pause_idle_s_ = 0.0;
        }
        close_trip(out);
    }
    if (stop_open_ && prev_) {
        stop_.end_ts = prev_->raw.ts;
        // A trip cut by end-of-stream leaves a zero-length "stop": not a stop.
        if (stop_.duration_s() > 0.0) out.stops.push_back(stop_);
        stop_open_ = false;
    }
    return out;
}

}  // namespace fleetline
