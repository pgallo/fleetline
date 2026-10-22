"""Round-trips across the C++/Python boundary."""

import math
import threading

import pytest

from fleetline import _core
from fleetline.roads import CITY_CENTER, offset_latlon


def east(m: float, north: float = 0.0) -> tuple[float, float]:
    return offset_latlon(CITY_CENTER, m, north)


def test_latlon_accepts_tuples_and_lists():
    a = _core.LatLon(45.0, 9.0)
    assert tuple(a) == (45.0, 9.0)
    assert _core.haversine_m((0.0, 0.0), [1.0, 0.0]) == pytest.approx(111195.0, abs=5)
    assert _core.bearing_deg((0, 0), (0, 1)) == pytest.approx(90.0)
    with pytest.raises(ValueError):
        _core.LatLon((1.0, 2.0, 3.0))


def test_point_in_polygon_concave():
    poly = [(0, 0), (0, 2), (1, 2), (1, 1), (2, 1), (2, 0)]
    assert _core.point_in_polygon((0.5, 1.5), poly)
    assert not _core.point_in_polygon((1.5, 1.5), poly)


def test_telemetry_point_fields_round_trip():
    p = _core.TelemetryPoint("V1", 100.5, 45.1, 9.2, speed_mps=12.5, heading_deg=270.0, ignition=True)
    assert (p.vehicle_id, p.ts, p.lat, p.lon, p.speed_mps, p.heading_deg, p.ignition) == (
        "V1", 100.5, 45.1, 9.2, 12.5, 270.0, True)
    p.speed_mps = 3.0
    assert p.speed_mps == 3.0
    assert _core.TelemetryPoint("V", 0, 0, 0).speed_mps == -1.0  # "not reported"


def test_filter_rejects_outlier_and_reports_count():
    f = _core.PositionFilter()
    for i in range(10):
        assert f.update(_core.TelemetryPoint("v", i, *east(10.0 * i), 10.0, 90.0, True)) is not None
    assert f.update(_core.TelemetryPoint("v", 10, *east(4000, 4000), 10.0, 90.0, True)) is None
    assert f.rejected == 1


def test_geofence_shapes_and_validation():
    c = _core.Geofence.circle("c", "Circle", east(0), 100.0, dwell_s=60)
    assert c.shape == _core.FenceShape.CIRCLE and c.radius_m == 100.0
    assert c.contains(east(50)) and not c.contains(east(150))
    assert c.signed_distance_m(east(150)) == pytest.approx(50.0, abs=0.5)
    sq = _core.Geofence.polygon("p", "Square", [east(-50, -50), east(50, -50), east(50, 50), east(-50, 50)])
    assert len(sq.vertices) == 4 and sq.contains(east(0))
    with pytest.raises(ValueError):
        _core.Geofence.polygon("bad", "bad", [east(0), east(1)])
    with pytest.raises(ValueError):
        _core.Geofence.circle("bad", "bad", east(0), -1.0)


def test_trip_segmenter_through_bindings():
    seg = _core.TripSegmenter("v")
    f = _core.PositionFilter()
    trips = []
    t = 0
    for _ in range(30):
        trips += seg.update(f.update(_core.TelemetryPoint("v", t, *east(0), 0.0, 0.0, False))).trips
        t += 10
    x = 0.0
    for _ in range(300):
        x += 12.0
        trips += seg.update(f.update(_core.TelemetryPoint("v", t, *east(x), 12.0, 90.0, True))).trips
        t += 1
    assert seg.state == _core.MotionState.DRIVING
    for _ in range(30):
        trips += seg.update(f.update(_core.TelemetryPoint("v", t, *east(x), 0.0, 90.0, False))).trips
        t += 10
    assert seg.state == _core.MotionState.PARKED
    assert len(trips) == 1
    assert trips[0].distance_m == pytest.approx(3600, rel=0.02)
    assert trips[0].duration_s == pytest.approx(300, abs=15)


def test_route_eta_round_trip():
    r = _core.Route([east(0), east(1000), east(1000, 1000)], [10.0, 20.0])
    assert r.length_m == pytest.approx(2000, abs=1)
    assert r.segments == 2
    res = r.eta(east(500))
    assert res.remaining_m == pytest.approx(1500, abs=1)
    assert res.eta_s == pytest.approx(100, abs=0.5)
    assert not res.off_route
    assert res.match.segment == 0
    with pytest.raises(ValueError):
        _core.Route([east(0), east(1)], [1.0, 2.0])


def test_stream_processor_columns_and_reordering():
    cfg = _core.StreamConfig()
    cfg.reorder_window_s = 20.0
    sp = _core.StreamProcessor(cfg)
    sp.add_geofence(_core.Geofence.circle("end", "End", east(3000), 150.0, dwell_s=30))
    ts = list(range(400))
    xs = [min(10.0 * t, 3000.0) for t in ts]
    rows = list(zip(ts, xs))
    rows = [r for pair in zip(rows[1::2], rows[0::2]) for r in pair]  # swap neighbours
    n = sp.ingest_columns(["v"] * len(rows), [float(t) for t, _ in rows],
                          [east(x)[0] for _, x in rows], [east(x)[1] for _, x in rows],
                          [10.0 if x < 3000 else 0.0 for _, x in rows], [90.0] * len(rows), [True] * len(rows))
    assert n > 0
    sp.flush()
    st = sp.stats()
    assert st.processed == 400 and st.late_dropped == 0
    assert [e.type.label for e in sp.geofence_events("v")][:2] == ["enter", "dwell"]
    assert len(sp.trips("v")) == 1
    status = sp.status("v")
    assert status.inside_fences == ["end"]
    assert sp.status("missing") is None
    with pytest.raises(ValueError):
        sp.ingest_columns(["v"], [1.0, 2.0], [0.0], [0.0], [0.0], [0.0], [True])


def test_stream_processor_parallel_python_threads():
    sp = _core.StreamProcessor()

    def worker(k: int) -> None:
        for t in range(500):
            sp.ingest(_core.TelemetryPoint(f"v{k}", float(t), *east(8.0 * t, 100.0 * k), 8.0, 90.0, True))

    threads = [threading.Thread(target=worker, args=(k,)) for k in range(6)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    sp.flush()
    assert sp.stats().processed == 3000
    assert sp.vehicles() == [f"v{k}" for k in range(6)]
    assert all(math.isclose(sp.recent_speed(f"v{k}", 60.0), 8.0) for k in range(6))
