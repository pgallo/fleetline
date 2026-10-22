// Python bindings for the fleetline C++ core.
//
// Data crosses the boundary in two shapes: small value structs (copied, cheap)
// for queries, and column vectors for bulk ingestion so a whole batch costs one
// call and one GIL release instead of one call per point.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <stdexcept>

#include "fleetline/eta.hpp"
#include "fleetline/filter.hpp"
#include "fleetline/geo.hpp"
#include "fleetline/geofence.hpp"
#include "fleetline/stream.hpp"
#include "fleetline/telemetry.hpp"
#include "fleetline/trips.hpp"

namespace py = pybind11;
using namespace fleetline;

namespace {

std::size_t ingest_columns(StreamProcessor& sp, const std::vector<std::string>& vehicle_ids,
                           const std::vector<double>& ts, const std::vector<double>& lat,
                           const std::vector<double>& lon, const std::vector<double>& speed,
                           const std::vector<double>& heading, const std::vector<bool>& ignition) {
    const std::size_t n = vehicle_ids.size();
    if (ts.size() != n || lat.size() != n || lon.size() != n || speed.size() != n || heading.size() != n ||
        ignition.size() != n) {
        throw std::invalid_argument("all columns must have the same length");
    }
    std::size_t processed = 0;
    py::gil_scoped_release release;
    TelemetryPoint p;
    for (std::size_t i = 0; i < n; ++i) {
        p.vehicle_id = vehicle_ids[i];
        p.ts = ts[i];
        p.lat = lat[i];
        p.lon = lon[i];
        p.speed_mps = speed[i];
        p.heading_deg = heading[i];
        p.ignition = ignition[i];
        processed += sp.ingest(p);
    }
    return processed;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "fleetline C++ core: filtering, geofencing, trip segmentation, ETA, stream processing";

    py::class_<LatLon>(m, "LatLon")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("lat"), py::arg("lon"))
        .def(py::init([](const py::sequence& s) {
            if (py::len(s) != 2) throw py::value_error("LatLon needs (lat, lon)");
            return LatLon{s[0].cast<double>(), s[1].cast<double>()};
        }))
        .def_readwrite("lat", &LatLon::lat)
        .def_readwrite("lon", &LatLon::lon)
        .def("__iter__", [](const LatLon& p) { return py::iter(py::make_tuple(p.lat, p.lon)); })
        .def("__repr__", [](const LatLon& p) {
            std::ostringstream os;
            os.precision(7);
            os << "LatLon(" << p.lat << ", " << p.lon << ")";
            return os.str();
        });
    py::implicitly_convertible<py::tuple, LatLon>();
    py::implicitly_convertible<py::list, LatLon>();

    m.def("haversine_m", &haversine_m, py::arg("a"), py::arg("b"));
    m.def("bearing_deg", &bearing_deg, py::arg("a"), py::arg("b"));
    m.def("point_in_polygon", &point_in_polygon, py::arg("p"), py::arg("polygon"));
    m.def("winding_number", &winding_number, py::arg("p"), py::arg("polygon"));

    py::class_<TelemetryPoint>(m, "TelemetryPoint")
        .def(py::init([](std::string vid, double ts, double lat, double lon, double speed, double heading, bool ign) {
                 return TelemetryPoint{std::move(vid), ts, lat, lon, speed, heading, ign};
             }),
             py::arg("vehicle_id"), py::arg("ts"), py::arg("lat"), py::arg("lon"), py::arg("speed_mps") = -1.0,
             py::arg("heading_deg") = 0.0, py::arg("ignition") = false)
        .def_readwrite("vehicle_id", &TelemetryPoint::vehicle_id)
        .def_readwrite("ts", &TelemetryPoint::ts)
        .def_readwrite("lat", &TelemetryPoint::lat)
        .def_readwrite("lon", &TelemetryPoint::lon)
        .def_readwrite("speed_mps", &TelemetryPoint::speed_mps)
        .def_readwrite("heading_deg", &TelemetryPoint::heading_deg)
        .def_readwrite("ignition", &TelemetryPoint::ignition);

    py::class_<FilteredPoint>(m, "FilteredPoint")
        .def_readonly("raw", &FilteredPoint::raw)
        .def_readonly("pos", &FilteredPoint::pos)
        .def_readonly("speed_mps", &FilteredPoint::speed_mps)
        .def_readonly("filter_speed_mps", &FilteredPoint::filter_speed_mps);

    py::class_<FilterConfig>(m, "FilterConfig")
        .def(py::init<>())
        .def_readwrite("gps_sigma_m", &FilterConfig::gps_sigma_m)
        .def_readwrite("accel_sigma_mps2", &FilterConfig::accel_sigma_mps2)
        .def_readwrite("max_implied_speed_mps", &FilterConfig::max_implied_speed_mps)
        .def_readwrite("reset_after_rejections", &FilterConfig::reset_after_rejections)
        .def_readwrite("max_gap_s", &FilterConfig::max_gap_s);

    py::class_<PositionFilter>(m, "PositionFilter")
        .def(py::init<FilterConfig>(), py::arg("config") = FilterConfig{})
        .def("update", &PositionFilter::update, py::arg("point"))
        .def_property_readonly("rejected", &PositionFilter::rejected)
        .def("reset", &PositionFilter::reset);

    py::enum_<FenceShape>(m, "FenceShape").value("POLYGON", FenceShape::Polygon).value("CIRCLE", FenceShape::Circle);

    py::class_<Geofence>(m, "Geofence")
        .def_static("polygon", &Geofence::polygon, py::arg("id"), py::arg("name"), py::arg("vertices"),
                    py::arg("dwell_s") = 300.0)
        .def_static("circle", &Geofence::circle, py::arg("id"), py::arg("name"), py::arg("center"),
                    py::arg("radius_m"), py::arg("dwell_s") = 300.0)
        .def_readonly("id", &Geofence::id)
        .def_readonly("name", &Geofence::name)
        .def_readonly("shape", &Geofence::shape)
        .def_readonly("vertices", &Geofence::vertices)
        .def_readonly("center", &Geofence::center)
        .def_readonly("radius_m", &Geofence::radius_m)
        .def_readonly("dwell_s", &Geofence::dwell_s)
        .def("contains", [](const Geofence& g, const LatLon& p) { return PreparedFence(g).contains(p); })
        .def("signed_distance_m",
             [](const Geofence& g, const LatLon& p) { return PreparedFence(g).signed_distance_m(p); });

    py::enum_<GeofenceEventType>(m, "GeofenceEventType")
        .value("ENTER", GeofenceEventType::Enter)
        .value("EXIT", GeofenceEventType::Exit)
        .value("DWELL", GeofenceEventType::Dwell)
        .def_property_readonly("label", [](GeofenceEventType t) { return to_string(t); });

    py::class_<GeofenceEvent>(m, "GeofenceEvent")
        .def_readonly("vehicle_id", &GeofenceEvent::vehicle_id)
        .def_readonly("fence_id", &GeofenceEvent::fence_id)
        .def_readonly("type", &GeofenceEvent::type)
        .def_readonly("ts", &GeofenceEvent::ts)
        .def_readonly("pos", &GeofenceEvent::pos)
        .def_readonly("dwell_s", &GeofenceEvent::dwell_s);

    py::class_<TripConfig>(m, "TripConfig")
        .def(py::init<>())
        .def_readwrite("move_speed_mps", &TripConfig::move_speed_mps)
        .def_readwrite("stop_speed_mps", &TripConfig::stop_speed_mps)
        .def_readwrite("min_stop_s", &TripConfig::min_stop_s)
        .def_readwrite("ignition_off_stop_s", &TripConfig::ignition_off_stop_s)
        .def_readwrite("min_trip_distance_m", &TripConfig::min_trip_distance_m)
        .def_readwrite("harsh_brake_mps2", &TripConfig::harsh_brake_mps2)
        .def_readwrite("harsh_accel_mps2", &TripConfig::harsh_accel_mps2);

    py::enum_<HarshType>(m, "HarshType")
        .value("BRAKING", HarshType::Braking)
        .value("ACCELERATION", HarshType::Acceleration)
        .def_property_readonly("label", [](HarshType t) { return to_string(t); });

    py::class_<HarshEvent>(m, "HarshEvent")
        .def_readonly("type", &HarshEvent::type)
        .def_readonly("ts", &HarshEvent::ts)
        .def_readonly("pos", &HarshEvent::pos)
        .def_readonly("accel_mps2", &HarshEvent::accel_mps2);

    py::class_<Trip>(m, "Trip")
        .def_readonly("vehicle_id", &Trip::vehicle_id)
        .def_readonly("start_ts", &Trip::start_ts)
        .def_readonly("end_ts", &Trip::end_ts)
        .def_readonly("start_pos", &Trip::start_pos)
        .def_readonly("end_pos", &Trip::end_pos)
        .def_readonly("distance_m", &Trip::distance_m)
        .def_readonly("idle_s", &Trip::idle_s)
        .def_readonly("max_speed_mps", &Trip::max_speed_mps)
        .def_readonly("points", &Trip::points)
        .def_readonly("harsh_events", &Trip::harsh_events)
        .def_property_readonly("duration_s", &Trip::duration_s);

    py::class_<Stop>(m, "Stop")
        .def_readonly("vehicle_id", &Stop::vehicle_id)
        .def_readonly("start_ts", &Stop::start_ts)
        .def_readonly("end_ts", &Stop::end_ts)
        .def_readonly("pos", &Stop::pos)
        .def_readonly("idle_s", &Stop::idle_s)
        .def_property_readonly("duration_s", &Stop::duration_s);

    py::enum_<MotionState>(m, "MotionState")
        .value("PARKED", MotionState::Parked)
        .value("DRIVING", MotionState::Driving)
        .value("PAUSING", MotionState::Pausing)
        .def_property_readonly("label", [](MotionState s) { return to_string(s); });

    py::class_<SegmentOutput>(m, "SegmentOutput")
        .def_readonly("trips", &SegmentOutput::trips)
        .def_readonly("stops", &SegmentOutput::stops);

    py::class_<TripSegmenter>(m, "TripSegmenter")
        .def(py::init<std::string, TripConfig>(), py::arg("vehicle_id"), py::arg("config") = TripConfig{})
        .def("update", &TripSegmenter::update, py::arg("point"))
        .def("flush", &TripSegmenter::flush)
        .def_property_readonly("state", &TripSegmenter::state);

    py::class_<RouteMatch>(m, "RouteMatch")
        .def_readonly("segment", &RouteMatch::segment)
        .def_readonly("t", &RouteMatch::t)
        .def_readonly("offset_m", &RouteMatch::offset_m)
        .def_readonly("cross_track_m", &RouteMatch::cross_track_m)
        .def_readonly("snapped", &RouteMatch::snapped);

    py::class_<EtaResult>(m, "EtaResult")
        .def_readonly("remaining_m", &EtaResult::remaining_m)
        .def_readonly("eta_s", &EtaResult::eta_s)
        .def_readonly("match", &EtaResult::match)
        .def_readonly("off_route", &EtaResult::off_route);

    py::class_<Route>(m, "Route")
        .def(py::init<std::vector<LatLon>, std::vector<double>>(), py::arg("points"),
             py::arg("segment_speed_mps") = std::vector<double>{})
        .def_property_readonly("length_m", &Route::length_m)
        .def_property_readonly("segments", &Route::segments)
        .def_property_readonly("points", &Route::points)
        .def("match", &Route::match, py::arg("p"), py::arg("min_offset_m") = -1.0, py::arg("backtrack_m") = 200.0)
        .def("time_to_end_s", &Route::time_to_end_s, py::arg("match"))
        .def("eta", &Route::eta, py::arg("p"), py::arg("observed_speed_mps") = -1.0, py::arg("min_offset_m") = -1.0,
             py::arg("off_route_threshold_m") = 75.0);

    py::class_<StreamConfig>(m, "StreamConfig")
        .def(py::init<>())
        .def_readwrite("reorder_window_s", &StreamConfig::reorder_window_s)
        .def_readwrite("max_buffer", &StreamConfig::max_buffer)
        .def_readwrite("geofence_hysteresis_m", &StreamConfig::geofence_hysteresis_m)
        .def_readwrite("geofence_cell_deg", &StreamConfig::geofence_cell_deg)
        .def_readwrite("filter", &StreamConfig::filter)
        .def_readwrite("trips", &StreamConfig::trips);

    py::class_<VehicleStatus>(m, "VehicleStatus")
        .def_readonly("vehicle_id", &VehicleStatus::vehicle_id)
        .def_readonly("last_ts", &VehicleStatus::last_ts)
        .def_readonly("lat", &VehicleStatus::lat)
        .def_readonly("lon", &VehicleStatus::lon)
        .def_readonly("speed_mps", &VehicleStatus::speed_mps)
        .def_readonly("heading_deg", &VehicleStatus::heading_deg)
        .def_readonly("ignition", &VehicleStatus::ignition)
        .def_readonly("state", &VehicleStatus::state)
        .def_readonly("inside_fences", &VehicleStatus::inside_fences)
        .def_readonly("trips", &VehicleStatus::trips)
        .def_readonly("buffered", &VehicleStatus::buffered);

    py::class_<StreamStats>(m, "StreamStats")
        .def_readonly("ingested", &StreamStats::ingested)
        .def_readonly("processed", &StreamStats::processed)
        .def_readonly("late_dropped", &StreamStats::late_dropped)
        .def_readonly("duplicates", &StreamStats::duplicates)
        .def_readonly("outliers", &StreamStats::outliers)
        .def_readonly("vehicles", &StreamStats::vehicles);

    py::class_<StreamOutput>(m, "StreamOutput")
        .def_readonly("trips", &StreamOutput::trips)
        .def_readonly("stops", &StreamOutput::stops)
        .def_readonly("geofence_events", &StreamOutput::geofence_events);

    // All methods that take a lock release the GIL: the processor has its own
    // locking, and Python threads (e.g. web workers) can then ingest in parallel.
    using rel = py::call_guard<py::gil_scoped_release>;
    py::class_<StreamProcessor>(m, "StreamProcessor")
        .def(py::init<StreamConfig>(), py::arg("config") = StreamConfig{})
        .def("add_geofence", &StreamProcessor::add_geofence, py::arg("fence"), rel())
        .def("remove_geofence", &StreamProcessor::remove_geofence, py::arg("id"), rel())
        .def("geofences", &StreamProcessor::geofences, rel())
        .def("ingest", &StreamProcessor::ingest, py::arg("point"), rel())
        .def("ingest_batch", &StreamProcessor::ingest_batch, py::arg("points"), rel())
        .def("ingest_columns", &ingest_columns, py::arg("vehicle_ids"), py::arg("ts"), py::arg("lat"), py::arg("lon"),
             py::arg("speed_mps"), py::arg("heading_deg"), py::arg("ignition"))
        .def("flush", &StreamProcessor::flush, rel())
        .def("flush_vehicle", &StreamProcessor::flush_vehicle, py::arg("vehicle_id"), rel())
        .def("vehicles", &StreamProcessor::vehicles, rel())
        .def("status", &StreamProcessor::status, py::arg("vehicle_id"), rel())
        .def("trips", &StreamProcessor::trips, py::arg("vehicle_id"), rel())
        .def("stops", &StreamProcessor::stops, py::arg("vehicle_id"), rel())
        .def("geofence_events", &StreamProcessor::geofence_events, py::arg("vehicle_id"), rel())
        .def("last_point", &StreamProcessor::last_point, py::arg("vehicle_id"), rel())
        .def("recent_speed", &StreamProcessor::recent_speed, py::arg("vehicle_id"), py::arg("window_s") = 300.0, rel())
        .def("drain", &StreamProcessor::drain, rel())
        .def("stats", &StreamProcessor::stats, rel());
}
