"""End-to-end demo: simulate a fleet, push the points through the C++ core in
network-arrival order, and print fleet reports.

    python -m fleetline.demo --vehicles 12 --hours 10 --seed 7
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime, timezone

import pandas as pd

from fleetline import analytics
from fleetline.engine import FleetEngine
from fleetline.simulator import FleetSimulator, SimConfig


def run(vehicles: int = 12, hours: float = 10.0, seed: int = 7, batch: int = 500) -> dict:
    sim = FleetSimulator(SimConfig(vehicles=vehicles, hours=hours, seed=seed))
    t0 = time.perf_counter()
    points = sim.run()
    arrivals = sim.deliver(points)
    t_sim = time.perf_counter() - t0

    engine = FleetEngine(sim.graph)
    t1 = time.perf_counter()
    etas: list[dict] = []
    snapshot_at = len(arrivals) // 2
    snapshot_ts = None
    for i in range(0, len(arrivals), batch):
        engine.ingest_points(arrivals[i:i + batch])
        if not etas and i + batch >= snapshot_at:
            # Mid-shift snapshot: "how long until each vehicle could be back at the depot?"
            snapshot_ts = arrivals[min(i + batch, len(arrivals)) - 1].ts
            depot = engine.resolve_destination("depot")
            for v in engine.core.vehicles():
                e = engine.eta(v, depot)
                if e is not None:
                    etas.append({"vehicle_id": v, "state": engine.status(v)["state"],
                                 "remaining_km": round(e["remaining_m"] / 1000.0, 2),
                                 "eta_min": round(e["eta_s"] / 60.0, 1),
                                 "observed_kmh": e["observed_speed_kmh"]})
    engine.flush()
    t_core = time.perf_counter() - t1

    trips = analytics.trips_frame(engine)
    stops = analytics.stops_frame(engine)
    events = analytics.events_frame(engine)
    names = {f["id"]: f["name"] for f in engine.geofences()}
    return {
        "sim": sim,
        "engine": engine,
        "points": len(points),
        "t_sim": t_sim,
        "t_core": t_core,
        "trips": trips,
        "utilisation": analytics.utilisation(trips, stops, hours * 3600.0, engine.core.vehicles()),
        "idle": analytics.idle_ranking(trips, stops),
        "dwell": analytics.dwell_report(events, names),
        "snapshot_ts": snapshot_ts,
        "etas": pd.DataFrame(etas).set_index("vehicle_id") if etas else pd.DataFrame(),
    }


def _section(title: str) -> None:
    print(f"\n{title}\n{'-' * len(title)}")


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--vehicles", type=int, default=12)
    ap.add_argument("--hours", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args(argv)

    r = run(args.vehicles, args.hours, args.seed)
    sim, engine = r["sim"], r["engine"]
    st = engine.stats()
    start = datetime.fromtimestamp(sim.t0, tz=timezone.utc).strftime("%Y-%m-%d %H:%M UTC")

    pd.set_option("display.width", 120)
    pd.set_option("display.max_columns", 20)
    print(f"fleetline demo: {args.vehicles} vehicles, {args.hours:g} simulated hours from {start}, seed {args.seed}")
    print(f"simulated {r['points']:,} points in {r['t_sim']:.2f}s; "
          f"core processed {st['processed']:,} in {r['t_core']:.2f}s "
          f"({st['processed'] / max(r['t_core'], 1e-9):,.0f} points/s)")
    print(f"late dropped: {st['late_dropped']}, duplicates: {st['duplicates']}, "
          f"GPS outliers rejected: {st['outliers']}, trips: {len(r['trips'])}")

    _section("Utilisation per vehicle")
    print(r["utilisation"].to_string())
    _section("Idle-time ranking (engine on, stationary)")
    print(r["idle"].to_string())
    _section("Geofence dwell report")
    print(r["dwell"].to_string())
    if not r["etas"].empty:
        at = datetime.fromtimestamp(r["snapshot_ts"], tz=timezone.utc).strftime("%H:%M UTC")
        _section(f"ETA to depot, mid-stream snapshot at {at}")
        print(r["etas"].to_string())


if __name__ == "__main__":
    main()
