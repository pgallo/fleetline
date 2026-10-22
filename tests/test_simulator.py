from fleetline.roads import build_city, haversine_m
from fleetline.simulator import FleetSimulator, SimConfig


def test_same_seed_same_points():
    a = FleetSimulator(SimConfig(vehicles=3, hours=2.0, seed=5)).run()
    b = FleetSimulator(SimConfig(vehicles=3, hours=2.0, seed=5)).run()
    assert len(a) > 1000
    assert a == b


def test_different_seed_different_points():
    a = FleetSimulator(SimConfig(vehicles=3, hours=2.0, seed=5)).run()
    b = FleetSimulator(SimConfig(vehicles=3, hours=2.0, seed=6)).run()
    assert a != b


def test_delivery_is_a_deterministic_permutation(small_sim, small_points):
    d1 = small_sim.deliver(small_points)
    d2 = small_sim.deliver(small_points)
    assert d1 == d2
    assert sorted(d1, key=lambda p: (p.ts, p.vehicle_id)) == small_points
    assert d1 != small_points  # some reordering actually happened


def test_points_are_plausible(small_sim, small_points):
    ids = {p.vehicle_id for p in small_points}
    assert ids == {prof.vehicle_id for prof in small_sim.profiles}
    assert all(small_sim.t0 <= p.ts < small_sim.t_end for p in small_points)
    assert all(0.0 <= p.speed_mps < 30.0 for p in small_points)
    assert all(p.speed_mps == 0.0 for p in small_points if not p.ignition)
    # Every vehicle drives (ignition on with speed) at some point.
    for vid in ids:
        assert any(p.ignition and p.speed_mps > 5 for p in small_points if p.vehicle_id == vid)
    # Sampling: 2 s with ignition on.
    one = [p for p in small_points if p.vehicle_id == min(ids) and p.ignition]
    gaps = [b.ts - a.ts for a, b in zip(one, one[1:]) if b.ts - a.ts < 30]
    assert min(gaps) >= 1.99


def test_road_graph_connected_and_routable():
    g = build_city()
    assert len(g.nodes) > 100
    path = g.shortest_path(g.places["depot"], g.places["port"])
    assert path[0] == g.places["depot"] and path[-1] == g.places["port"]
    pts, speeds = g.path_geometry(path)
    assert len(speeds) == len(pts) - 1
    length = sum(haversine_m(a, b) for a, b in zip(pts, pts[1:]))
    assert 1000 < length < 15000
