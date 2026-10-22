import pytest
from fastapi.testclient import TestClient

from fleetline.engine import FleetEngine
from fleetline.roads import CITY_CENTER, offset_latlon
from fleetline.service import create_app


@pytest.fixture()
def client():
    return TestClient(create_app(FleetEngine(reorder_window_s=10.0)))


@pytest.fixture(scope="module")
def loaded_client(small_sim, small_points):
    engine = FleetEngine(small_sim.graph, reorder_window_s=30.0)
    c = TestClient(create_app(engine))
    arrivals = small_sim.deliver(small_points)
    for i in range(0, len(arrivals), 5000):
        r = c.post("/points", json={"points": [p.as_dict() for p in arrivals[i:i + 5000]]})
        assert r.status_code == 200, r.text
    return c


def test_health_empty(client):
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok" and r.json()["ingested"] == 0


def test_ingest_validation(client):
    assert client.post("/points", json={"points": []}).status_code == 422
    bad = {"vehicle_id": "V", "ts": 0, "lat": 123.0, "lon": 9.0}
    assert client.post("/points", json={"points": [bad]}).status_code == 422


def test_unknown_vehicle_404(client):
    assert client.get("/vehicles/NOPE").status_code == 404
    assert client.get("/vehicles/NOPE/trips").status_code == 404
    assert client.get("/vehicles/NOPE/eta", params={"place": "depot"}).status_code == 404


def test_single_trip_via_api(client):
    pts = []
    t = 1_000_000.0
    for _ in range(20):
        pts.append({"vehicle_id": "V1", "ts": t, "lat": CITY_CENTER[0], "lon": CITY_CENTER[1], "ignition": False})
        t += 10
    x = 0.0
    for _ in range(200):
        x += 10.0
        lat, lon = offset_latlon(CITY_CENTER, x, 0)
        pts.append({"vehicle_id": "V1", "ts": t, "lat": lat, "lon": lon, "speed_mps": 10.0, "heading_deg": 90,
                    "ignition": True})
        t += 1
    r = client.post("/points", json={"points": pts})
    assert r.status_code == 200 and r.json()["accepted"] == 220

    status = client.get("/vehicles/V1").json()
    assert status["state"] == "driving" and status["ignition"] is True
    assert status["speed_kmh"] == pytest.approx(36.0)

    eta = client.get("/vehicles/V1/eta", params={"place": "depot"})
    assert eta.status_code == 200
    body = eta.json()
    assert body["remaining_m"] > 0 and body["eta_s"] > 0
    # The 5-minute window also covers the last parked fixes before departure.
    assert 30.0 < body["observed_speed_kmh"] <= 36.5

    assert client.post("/flush").status_code == 200
    trips = client.get("/vehicles/V1/trips").json()
    assert len(trips) == 1
    assert trips[0]["distance_m"] == pytest.approx(2000, rel=0.03)


def test_eta_destination_errors(client):
    client.post("/points", json={"points": [{"vehicle_id": "V2", "ts": 1.0, "lat": CITY_CENTER[0],
                                             "lon": CITY_CENTER[1]}]})
    assert client.get("/vehicles/V2/eta", params={"place": "moon"}).status_code == 404
    assert client.get("/vehicles/V2/eta").status_code == 422
    # Point is still in the reorder buffer: no processed position yet.
    assert client.get("/vehicles/V2/eta", params={"place": "depot"}).status_code == 409


def test_put_geofence(client):
    lat, lon = CITY_CENTER
    body = {"id": "test-zone", "name": "Test", "shape": "circle", "center": [lat, lon], "radius_m": 100}
    assert client.put("/geofences/test-zone", json=body).status_code == 201
    assert any(f["id"] == "test-zone" for f in client.get("/geofences").json())
    assert client.put("/geofences/other", json=body).status_code == 422
    bad = {"id": "p", "shape": "polygon", "vertices": [[0, 0], [1, 1]]}
    assert client.put("/geofences/p", json=bad).status_code == 422


def test_loaded_fleet_endpoints(loaded_client):
    c = loaded_client
    c.post("/flush")
    vehicles = c.get("/vehicles").json()
    assert len(vehicles) == 4
    assert {v["state"] for v in vehicles} <= {"parked", "driving", "pausing"}
    vid = vehicles[0]["vehicle_id"]
    trips = c.get(f"/vehicles/{vid}/trips").json()
    assert len(trips) >= 3
    assert all(t["distance_m"] >= 250 and t["end_ts"] > t["start_ts"] for t in trips)
    events = c.get("/geofence-events").json()
    assert events and {e["type"] for e in events} <= {"enter", "exit", "dwell"}
    enters = c.get("/geofence-events", params={"type": "enter", "fence_id": "depot"}).json()
    assert all(e["type"] == "enter" and e["fence_id"] == "depot" for e in enters)
    eta = c.get(f"/vehicles/{vid}/eta", params={"place": "port"}).json()
    assert eta["remaining_m"] > 0
    health = c.get("/health").json()
    assert health["late_dropped"] == 0 and health["outliers"] >= 0
