"""Deterministic synthetic fleet simulator.

Each vehicle starts parked at the depot, runs one shift of jobs between places
in the fictional city, and returns to the depot. Motion is integrated at 1 s
with simple kinematics (bounded acceleration, braking towards the next stop
point), then sampled the way a tracker would report it: every 2 s with the
ignition on, every 60 s with it off. GPS noise, rare position jumps and network
reordering are added on top so the core has something to filter.

Everything is driven by one ``random.Random(seed)``: same seed, same points.
"""

from __future__ import annotations

import bisect
import math
import random
from dataclasses import dataclass, field

from fleetline.roads import RoadGraph, build_city, haversine_m, offset_latlon

# Monday 2 March 2026, 00:00 UTC: the simulated day starts here.
SIM_EPOCH = 1772409600.0

ACCEL_MPS2 = 1.4
BRAKE_MPS2 = 1.8
HARSH_BRAKE_MPS2 = 5.0


@dataclass(frozen=True)
class Point:
    vehicle_id: str
    ts: float
    lat: float
    lon: float
    speed_mps: float
    heading_deg: float
    ignition: bool

    def as_dict(self) -> dict:
        return {
            "vehicle_id": self.vehicle_id,
            "ts": self.ts,
            "lat": self.lat,
            "lon": self.lon,
            "speed_mps": self.speed_mps,
            "heading_deg": self.heading_deg,
            "ignition": self.ignition,
        }


@dataclass
class VehicleProfile:
    vehicle_id: str
    kind: str  # "van" or "truck"
    speed_factor: float  # fraction of the road's free-flow speed this driver targets
    idle_propensity: float  # probability of leaving the engine running at a job stop
    harsh_propensity: float  # probability of braking late at a red light
    shift_start_s: float  # seconds after SIM_EPOCH
    shift_len_s: float


@dataclass
class SimConfig:
    vehicles: int = 12
    hours: float = 10.0
    seed: int = 7
    sample_on_s: float = 2.0
    sample_off_s: float = 60.0
    gps_sigma_m: float = 5.0
    outlier_prob: float = 0.001
    red_light_prob: float = 0.3
    start_hour: float = 6.0  # simulated wall-clock hour when the run begins


@dataclass
class _Track:
    """A polyline being driven, with cumulative offsets for interpolation."""

    points: list[tuple[float, float]]
    speeds: list[float]
    cum: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.cum = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + haversine_m(a, b))

    @property
    def length(self) -> float:
        return self.cum[-1]

    def locate(self, s: float) -> tuple[tuple[float, float], float, float]:
        """Position, heading and free-flow speed at offset s."""
        i = min(max(bisect.bisect_right(self.cum, s) - 1, 0), len(self.points) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg <= 0 else min(1.0, max(0.0, (s - self.cum[i]) / seg))
        a, b = self.points[i], self.points[i + 1]
        pos = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        dy = b[0] - a[0]
        dx = (b[1] - a[1]) * math.cos(math.radians(a[0]))
        heading = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
        return pos, heading, self.speeds[i]


class FleetSimulator:
    def __init__(self, config: SimConfig | None = None, graph: RoadGraph | None = None):
        self.cfg = config or SimConfig()
        self.graph = graph or build_city()
        self.rng = random.Random(self.cfg.seed)
        self.t0 = SIM_EPOCH + self.cfg.start_hour * 3600.0
        self.t_end = self.t0 + self.cfg.hours * 3600.0
        self.profiles = [self._profile(i) for i in range(self.cfg.vehicles)]

    # ------------------------------------------------------------------ setup

    def _profile(self, i: int) -> VehicleProfile:
        r = self.rng
        kind = "truck" if i % 4 == 3 else "van"
        return VehicleProfile(
            vehicle_id=f"{'TRK' if kind == 'truck' else 'VAN'}-{i + 1:03d}",
            kind=kind,
            speed_factor=r.uniform(0.75, 0.95) * (0.88 if kind == "truck" else 1.0),
            idle_propensity=r.choice([0.1, 0.25, 0.4, 0.7]),
            harsh_propensity=r.choice([0.0, 0.03, 0.08, 0.2]),
            shift_start_s=r.uniform(0.0, 2.0) * 3600.0,
            shift_len_s=r.uniform(7.0, 9.0) * 3600.0,
        )

    # ------------------------------------------------------------- generation

    def run(self) -> list[Point]:
        """All points of all vehicles, sorted by event time."""
        pts: list[Point] = []
        for prof in self.profiles:
            pts.extend(_VehicleRun(self, prof).run())
        pts.sort(key=lambda p: (p.ts, p.vehicle_id))
        return pts

    def deliver(self, points: list[Point], mean_delay_s: float = 2.0, burst_prob: float = 0.02,
                max_delay_s: float = 25.0) -> list[Point]:
        """Reorder points by simulated network arrival time: small exponential
        delays plus occasional store-and-forward bursts. Uses its own RNG so
        delivery does not perturb the motion of the fleet."""
        r = random.Random(self.cfg.seed * 7919 + 1)
        arrivals = []
        for p in points:
            delay = min(r.expovariate(1.0 / mean_delay_s), max_delay_s)
            if r.random() < burst_prob:
                delay = r.uniform(10.0, max_delay_s)
            arrivals.append((p.ts + delay, p.vehicle_id, p))
        arrivals.sort(key=lambda a: (a[0], a[1]))
        return [a[2] for a in arrivals]


class _VehicleRun:
    def __init__(self, sim: FleetSimulator, prof: VehicleProfile):
        self.sim = sim
        self.cfg = sim.cfg
        self.g = sim.graph
        self.p = prof
        self.r = random.Random(f"{sim.cfg.seed}:{prof.vehicle_id}")
        self.node = self.g.places["depot"]
        self.pos = self.g.nodes[self.node]
        self.heading = 0.0
        self.t = sim.t0
        self.out: list[Point] = []
        self.last_emit = -math.inf

    # -- emission

    def _emit(self, speed: float, ignition: bool, force: bool = False) -> None:
        interval = self.cfg.sample_on_s if ignition else self.cfg.sample_off_s
        if not force and self.t - self.last_emit < interval - 1e-9:
            return
        if self.t - self.last_emit < 1.0:
            # Trackers report at most once per second; a forced report (ignition
            # change) right after a periodic one is pushed one second later.
            self.t = self.last_emit + 1.0
        if self.t >= self.sim.t_end:
            return
        self.last_emit = self.t
        r = self.r
        if r.random() < self.cfg.outlier_prob:
            # Multipath / cold-start jump: hundreds of metres off.
            ang = r.uniform(0, 2 * math.pi)
            dist = r.uniform(400.0, 2000.0)
            lat, lon = offset_latlon(self.pos, dist * math.cos(ang), dist * math.sin(ang))
        else:
            lat, lon = offset_latlon(self.pos, r.gauss(0, self.cfg.gps_sigma_m), r.gauss(0, self.cfg.gps_sigma_m))
        reported = max(0.0, speed + r.gauss(0, 0.15)) if ignition else 0.0
        self.out.append(Point(self.p.vehicle_id, round(self.t, 3), lat, lon, round(reported, 3),
                              round(self.heading, 1), ignition))

    def _wait(self, duration_s: float, ignition: bool) -> None:
        end = min(self.t + duration_s, self.sim.t_end)
        step = 1.0 if ignition else 10.0
        while self.t < end:
            self._emit(0.0, ignition)
            self.t += step

    # -- driving

    def _drive_to(self, dst: str) -> None:
        path = self.g.shortest_path(self.node, dst)
        if len(path) < 2:
            return
        points, speeds = self.g.path_geometry(path)
        track = _Track(points, speeds)
        # Stop points: red lights at some intermediate intersections, plus the destination.
        stops: list[tuple[float, float, bool]] = []  # (offset, wait_s, harsh)
        for k in range(1, len(path) - 1):
            if self.r.random() < self.cfg.red_light_prob:
                stops.append((track.cum[k], self.r.uniform(15.0, 60.0), self.r.random() < self.p.harsh_propensity))
        stops.append((track.length, 0.0, False))
        traffic = self.r.uniform(0.7, 1.0)

        s, v = 0.0, 0.0
        while stops and self.t < self.sim.t_end:
            stop_at, wait_s, harsh = stops[0]
            d = stop_at - s
            pos, heading, free_flow = track.locate(s)
            target = free_flow * self.p.speed_factor * traffic
            brake = HARSH_BRAKE_MPS2 if harsh else BRAKE_MPS2
            v_cap = math.sqrt(2.0 * brake * max(d - 1.0, 0.0))
            if harsh and v_cap > v:
                v_cap = math.inf  # a late braker keeps speed until the last moment
            desired = min(target, v_cap)
            if v < desired:
                v = min(desired, v + ACCEL_MPS2)
            elif v_cap <= target:
                v = desired  # following the braking curve towards a stop point
            else:
                v = max(desired, v - BRAKE_MPS2)  # slower road ahead: ease off gently
            self.pos, self.heading = pos, heading
            self._emit(v, True)
            s = min(stop_at, s + v)
            self.t += 1.0
            if stop_at - s < 1.5 and v < 2.0:
                s = stop_at
                self.pos = track.locate(s)[0]
                v = 0.0
                stops.pop(0)
                if wait_s > 0:
                    self._wait(wait_s, True)
        self.node = dst
        self.pos = self.g.nodes[dst]

    # -- the day

    def run(self) -> list[Point]:
        shift_start = self.sim.t0 + self.p.shift_start_s
        shift_end = shift_start + self.p.shift_len_s
        self._wait(shift_start - self.t, False)
        if self.t >= self.sim.t_end:
            return self.out
        self._emit(0.0, True, force=True)
        self._wait(self.r.uniform(60.0, 240.0), True)  # warm-up idle at the depot

        places = [p for p in self.g.places if p != "depot"]
        nodes = list(self.g.nodes)
        jobs_since_fuel = 0
        while self.t < min(shift_end, self.sim.t_end):
            if jobs_since_fuel >= 5 and self.r.random() < 0.5:
                dst = self.g.places["fuel"]
                jobs_since_fuel = 0
            elif self.r.random() < 0.5:
                dst = self.g.places[self.r.choice(places)]
            else:
                dst = self.r.choice(nodes)
            if dst == self.node:
                continue
            self._drive_to(dst)
            jobs_since_fuel += 1
            if self.r.random() < self.p.idle_propensity:
                self._wait(self.r.uniform(180.0, 720.0), True)  # engine left running
            else:
                self._wait(self.r.uniform(20.0, 60.0), True)
                self._emit(0.0, False, force=True)
                self._wait(self.r.uniform(300.0, 1500.0), False)
                self._emit(0.0, True, force=True)
                self._wait(self.r.uniform(15.0, 45.0), True)

        if self.t < self.sim.t_end:
            self._drive_to(self.g.places["depot"])
            self._wait(self.r.uniform(30.0, 120.0), True)
            self._emit(0.0, False, force=True)
            self._wait(self.sim.t_end - self.t, False)
        return self.out


def simulate(vehicles: int = 12, hours: float = 10.0, seed: int = 7) -> list[Point]:
    return FleetSimulator(SimConfig(vehicles=vehicles, hours=hours, seed=seed)).run()
