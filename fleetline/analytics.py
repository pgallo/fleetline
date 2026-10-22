"""Fleet reports on top of the engine's trips, stops and geofence events."""

from __future__ import annotations

import pandas as pd

from fleetline.engine import FleetEngine

TRIP_COLUMNS = ["vehicle_id", "start_ts", "end_ts", "distance_m", "duration_s", "idle_s", "max_speed_kmh",
                "harsh_braking", "harsh_acceleration"]
STOP_COLUMNS = ["vehicle_id", "start_ts", "end_ts", "duration_s", "idle_s"]
EVENT_COLUMNS = ["vehicle_id", "fence_id", "type", "ts", "dwell_s"]


def trips_frame(engine: FleetEngine) -> pd.DataFrame:
    rows = [t for v in engine.core.vehicles() for t in engine.trips(v)]
    return pd.DataFrame(rows, columns=TRIP_COLUMNS) if rows else pd.DataFrame(columns=TRIP_COLUMNS)


def stops_frame(engine: FleetEngine) -> pd.DataFrame:
    rows = [s for v in engine.core.vehicles() for s in engine.stops(v)]
    return pd.DataFrame(rows, columns=STOP_COLUMNS) if rows else pd.DataFrame(columns=STOP_COLUMNS)


def events_frame(engine: FleetEngine) -> pd.DataFrame:
    rows = engine.geofence_events()
    return pd.DataFrame(rows, columns=EVENT_COLUMNS) if rows else pd.DataFrame(columns=EVENT_COLUMNS)


def utilisation(trips: pd.DataFrame, stops: pd.DataFrame, period_s: float,
                vehicles: list[str] | None = None) -> pd.DataFrame:
    """Per vehicle: trips, km, driving hours, idle hours (in trips and at stops)
    and utilisation = driving time / reporting period."""
    ids = sorted(set(vehicles or []) | set(trips["vehicle_id"]) | set(stops["vehicle_id"]))
    t = trips.groupby("vehicle_id").agg(trips=("distance_m", "size"), distance_km=("distance_m", "sum"),
                                         driving_s=("duration_s", "sum"), trip_idle_s=("idle_s", "sum"),
                                         harsh_events=("harsh_braking", "sum"),
                                         harsh_accel=("harsh_acceleration", "sum"))
    s = stops.groupby("vehicle_id").agg(stop_idle_s=("idle_s", "sum"))
    df = t.join(s, how="outer").reindex(ids).fillna(0.0)
    df["trips"] = df["trips"].astype(int)
    df["distance_km"] = (df["distance_km"] / 1000.0).round(1)
    df["harsh_events"] = (df["harsh_events"] + df.pop("harsh_accel")).astype(int)
    df["driving_h"] = (df["driving_s"] / 3600.0).round(2)
    df["idle_h"] = ((df["trip_idle_s"] + df["stop_idle_s"]) / 3600.0).round(2)
    df["utilisation_pct"] = (100.0 * df["driving_s"] / period_s).round(1)
    df["avg_speed_kmh"] = (df["distance_km"] / (df["driving_s"] / 3600.0)).where(df["driving_s"] > 0, 0.0).round(1)
    df.index.name = "vehicle_id"
    return df[["trips", "distance_km", "driving_h", "idle_h", "utilisation_pct", "avg_speed_kmh",
               "harsh_events"]].sort_values("utilisation_pct", ascending=False)


def idle_ranking(trips: pd.DataFrame, stops: pd.DataFrame) -> pd.DataFrame:
    """Engine-on stationary time per vehicle, worst first. idle_share is idle
    time over total engine-on time (driving plus idle)."""
    trip_idle = trips.groupby("vehicle_id")["idle_s"].sum()
    driving = trips.groupby("vehicle_id")["duration_s"].sum()
    stop_idle = stops.groupby("vehicle_id")["idle_s"].sum()
    long_idles = stops[stops["idle_s"] >= 300].groupby("vehicle_id").size()
    df = pd.DataFrame({"trip_idle_s": trip_idle, "stop_idle_s": stop_idle, "driving_s": driving,
                       "idles_over_5min": long_idles}).fillna(0.0)
    df["idle_min"] = ((df["trip_idle_s"] + df["stop_idle_s"]) / 60.0).round(1)
    engine_on = df["driving_s"] + df["stop_idle_s"]
    df["idle_share_pct"] = (100.0 * (df["trip_idle_s"] + df["stop_idle_s"]) / engine_on).where(engine_on > 0,
                                                                                               0.0).round(1)
    df["idles_over_5min"] = df["idles_over_5min"].astype(int)
    df.index.name = "vehicle_id"
    out = df[["idle_min", "idle_share_pct", "idles_over_5min"]].sort_values("idle_min", ascending=False)
    out.insert(0, "rank", range(1, len(out) + 1))
    return out


def dwell_report(events: pd.DataFrame, fence_names: dict[str, str] | None = None) -> pd.DataFrame:
    """Per geofence: visits (enters), distinct vehicles, completed visits
    (exits), mean and max dwell of completed visits, and visits that crossed
    the fence's dwell threshold."""
    if events.empty:
        return pd.DataFrame(columns=["fence", "visits", "vehicles", "completed", "mean_dwell_min",
                                     "max_dwell_min", "dwell_alerts"])
    g = events.groupby("fence_id")
    exits = events[events["type"] == "exit"].groupby("fence_id")["dwell_s"]
    df = pd.DataFrame({
        "visits": g.apply(lambda x: int((x["type"] == "enter").sum()), include_groups=False),
        "vehicles": g["vehicle_id"].nunique(),
        "completed": exits.size(),
        "mean_dwell_min": (exits.mean() / 60.0).round(1),
        "max_dwell_min": (exits.max() / 60.0).round(1),
        "dwell_alerts": g.apply(lambda x: int((x["type"] == "dwell").sum()), include_groups=False),
    }).fillna({"completed": 0, "mean_dwell_min": 0.0, "max_dwell_min": 0.0})
    df["completed"] = df["completed"].astype(int)
    df.insert(0, "fence", [fence_names.get(i, i) if fence_names else i for i in df.index])
    df.index.name = "fence_id"
    return df.sort_values("visits", ascending=False)
