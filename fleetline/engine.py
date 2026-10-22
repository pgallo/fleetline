"""Thin Python facade over the C++ stream processor.

Converts between the core's value types and plain dicts (for the API and for
data frames), and owns the bits that are orchestration rather than hot path:
geofence definitions from config, destination lookup and route planning for
ETA queries.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

from fleetline import _core
from fleetline.roads import RoadGraph, build_city, default_geofences


def fence_from_dict(d: Mapping[str, Any]) -> _core.Geofence:
    if d["shape"] == "circle":
        return _core.Geofence.circle(d["id"], d.get("name", d["id"]), tuple(d["center"]), float(d["radius_m"]),
                                     float(d.get("dwell_s", 300.0)))
    if d["shape"] == "polygon":
        return _core.Geofence.polygon(d["id"], d.get("name", d["id"]), [tuple(v) for v in d["vertices"]],
                                      float(d.get("dwell_s", 300.0)))
    raise ValueError(f"unknown geofence shape {d['shape']!r}")


def fence_to_dict(g: _core.Geofence) -> dict:
    d: dict[str, Any] = {"id": g.id, "name": g.name, "dwell_s": g.dwell_s}
    if g.shape == _core.FenceShape.CIRCLE:
        d.update(shape="circle", center=[g.center.lat, g.center.lon], radius_m=g.radius_m)
    else:
        d.update(shape="polygon", vertices=[[v.lat, v.lon] for v in g.vertices])
    return d


def trip_to_dict(t: _core.Trip) -> dict:
    return {
        "vehicle_id": t.vehicle_id,
        "start_ts": t.start_ts,
        "end_ts": t.end_ts,
        "start": [t.start_pos.lat, t.start_pos.lon],
        "end": [t.end_pos.lat, t.end_pos.lon],
        "distance_m": round(t.distance_m, 1),
        "duration_s": round(t.duration_s, 1),
        "idle_s": round(t.idle_s, 1),
        "max_speed_kmh": round(t.max_speed_mps * 3.6, 1),
        "points": t.points,
        "harsh_braking": sum(e.type == _core.HarshType.BRAKING for e in t.harsh_events),
        "harsh_acceleration": sum(e.type == _core.HarshType.ACCELERATION for e in t.harsh_events),
    }


def stop_to_dict(s: _core.Stop) -> dict:
    return {
        "vehicle_id": s.vehicle_id,
        "start_ts": s.start_ts,
        "end_ts": s.end_ts,
        "pos": [s.pos.lat, s.pos.lon],
        "duration_s": round(s.duration_s, 1),
        "idle_s": round(s.idle_s, 1),
    }


def event_to_dict(e: _core.GeofenceEvent) -> dict:
    return {
        "vehicle_id": e.vehicle_id,
        "fence_id": e.fence_id,
        "type": e.type.label,
        "ts": e.ts,
        "pos": [e.pos.lat, e.pos.lon],
        "dwell_s": round(e.dwell_s, 1),
    }


def status_to_dict(s: _core.VehicleStatus) -> dict:
    return {
        "vehicle_id": s.vehicle_id,
        "last_ts": s.last_ts,
        "lat": s.lat,
        "lon": s.lon,
        "speed_kmh": round(s.speed_mps * 3.6, 1),
        "heading_deg": s.heading_deg,
        "ignition": s.ignition,
        "state": s.state.label,
        "inside_fences": list(s.inside_fences),
        "trips": s.trips,
        "buffered": s.buffered,
    }


class FleetEngine:
    """One stream processor plus the road graph used to plan ETA routes."""

    def __init__(self, graph: RoadGraph | None = None, geofences: Iterable[Mapping[str, Any]] | None = None,
                 reorder_window_s: float = 30.0):
        self.graph = graph or build_city()
        cfg = _core.StreamConfig()
        cfg.reorder_window_s = reorder_window_s
        self.core = _core.StreamProcessor(cfg)
        for d in default_geofences(self.graph) if geofences is None else geofences:
            self.add_geofence(d)

    # -- ingestion

    def add_geofence(self, d: Mapping[str, Any]) -> None:
        self.core.add_geofence(fence_from_dict(d))

    def ingest_dicts(self, points: Iterable[Mapping[str, Any]]) -> int:
        cols: dict[str, list] = {k: [] for k in ("vehicle_id", "ts", "lat", "lon", "speed_mps", "heading_deg",
                                                 "ignition")}
        for p in points:
            cols["vehicle_id"].append(str(p["vehicle_id"]))
            cols["ts"].append(float(p["ts"]))
            cols["lat"].append(float(p["lat"]))
            cols["lon"].append(float(p["lon"]))
            sp = p.get("speed_mps")
            cols["speed_mps"].append(-1.0 if sp is None else float(sp))
            cols["heading_deg"].append(float(p.get("heading_deg") or 0.0))
            cols["ignition"].append(bool(p.get("ignition", False)))
        return self.core.ingest_columns(cols["vehicle_id"], cols["ts"], cols["lat"], cols["lon"], cols["speed_mps"],
                                        cols["heading_deg"], cols["ignition"])

    def ingest_points(self, points: Iterable[Any]) -> int:
        """Accepts simulator Points (anything with the same attributes)."""
        pts = list(points)
        return self.core.ingest_columns([p.vehicle_id for p in pts], [p.ts for p in pts], [p.lat for p in pts],
                                        [p.lon for p in pts], [p.speed_mps for p in pts],
                                        [p.heading_deg for p in pts], [p.ignition for p in pts])

    def flush(self) -> None:
        self.core.flush()

    # -- queries

    def vehicles(self) -> list[dict]:
        return [status_to_dict(self.core.status(v)) for v in self.core.vehicles()]

    def status(self, vehicle_id: str) -> dict | None:
        s = self.core.status(vehicle_id)
        return None if s is None else status_to_dict(s)

    def trips(self, vehicle_id: str) -> list[dict]:
        return [trip_to_dict(t) for t in self.core.trips(vehicle_id)]

    def stops(self, vehicle_id: str) -> list[dict]:
        return [stop_to_dict(s) for s in self.core.stops(vehicle_id)]

    def geofence_events(self, vehicle_id: str | None = None) -> list[dict]:
        ids = [vehicle_id] if vehicle_id else self.core.vehicles()
        events = [event_to_dict(e) for v in ids for e in self.core.geofence_events(v)]
        return sorted(events, key=lambda e: (e["ts"], e["vehicle_id"]))

    def geofences(self) -> list[dict]:
        return [fence_to_dict(g) for g in self.core.geofences()]

    def stats(self) -> dict:
        s = self.core.stats()
        return {k: getattr(s, k) for k in ("ingested", "processed", "late_dropped", "duplicates", "outliers",
                                           "vehicles")}

    def resolve_destination(self, place: str | None = None, lat: float | None = None,
                            lon: float | None = None) -> tuple[float, float]:
        if place is not None:
            if place not in self.graph.places:
                raise KeyError(f"unknown place {place!r}; known: {sorted(self.graph.places)}")
            return self.graph.nodes[self.graph.places[place]]
        if lat is None or lon is None:
            raise ValueError("give either a place name or lat and lon")
        return lat, lon

    def eta(self, vehicle_id: str, destination: tuple[float, float]) -> dict | None:
        """Plan a route on the road graph from the vehicle's filtered position,
        then let the core match the position and estimate time along it."""
        last = self.core.last_point(vehicle_id)
        if last is None:
            return None
        here = (last.pos.lat, last.pos.lon)
        src = self.graph.nearest_node(here)
        dst = self.graph.nearest_node(destination)
        path = self.graph.shortest_path(src, dst)
        points, speeds = self.graph.path_geometry(path)
        # Prepend the current position so the route starts where the vehicle is.
        points = [here, *points]
        speeds = [speeds[0] if speeds else 11.0, *speeds]
        if destination != points[-1]:
            points.append(destination)
            speeds.append(speeds[-1])
        # Drop zero-length legs (vehicle sitting exactly on a node).
        clean_pts, clean_spd = [points[0]], []
        for p, s in zip(points[1:], speeds):
            if p != clean_pts[-1]:
                clean_pts.append(p)
                clean_spd.append(s)
        if len(clean_pts) < 2:
            return {"vehicle_id": vehicle_id, "remaining_m": 0.0, "eta_s": 0.0, "off_route": False,
                    "observed_speed_kmh": 0.0, "route_nodes": path}
        route = _core.Route(clean_pts, clean_spd)
        # Recent speed (including short stops) describes current traffic only
        # while the vehicle is on a trip; a parked vehicle gets the profile ETA.
        state = self.core.status(vehicle_id).state
        moving = state in (_core.MotionState.DRIVING, _core.MotionState.PAUSING)
        observed = self.core.recent_speed(vehicle_id, 300.0) if moving else -1.0
        res = route.eta(here, observed)
        return {
            "vehicle_id": vehicle_id,
            "remaining_m": round(res.remaining_m, 1),
            "eta_s": round(res.eta_s, 1),
            "off_route": res.off_route,
            "observed_speed_kmh": round(max(observed, 0.0) * 3.6, 1),
            "route_nodes": path,
        }
