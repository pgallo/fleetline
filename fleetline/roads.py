"""Synthetic road network for a fictional city.

The city is a grid of streets with a faster ring road around it and a couple of
diagonal arterials. It exists only so the simulator has somewhere to drive and
the ETA endpoint has something to route on; it is not meant to look like any
real place.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

EARTH_RADIUS_M = 6371008.8

# Centre of the fictional city (an arbitrary inland point in northern Italy).
CITY_CENTER = (45.20, 9.60)


def offset_latlon(origin: tuple[float, float], east_m: float, north_m: float) -> tuple[float, float]:
    lat0, lon0 = origin
    lat = lat0 + math.degrees(north_m / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Pure-Python haversine, used only for graph construction and snapping."""
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1])
    h = math.sin(dlat / 2) ** 2 + math.cos(math.radians(a[0])) * math.cos(math.radians(b[0])) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


@dataclass(frozen=True)
class Edge:
    to: str
    length_m: float
    speed_mps: float
    kind: str

    @property
    def time_s(self) -> float:
        return self.length_m / self.speed_mps


@dataclass
class RoadGraph:
    nodes: dict[str, tuple[float, float]] = field(default_factory=dict)
    adj: dict[str, list[Edge]] = field(default_factory=dict)
    places: dict[str, str] = field(default_factory=dict)  # place name -> node id

    def add_node(self, node_id: str, pos: tuple[float, float]) -> None:
        self.nodes[node_id] = pos
        self.adj.setdefault(node_id, [])

    def add_road(self, a: str, b: str, speed_mps: float, kind: str) -> None:
        length = haversine_m(self.nodes[a], self.nodes[b])
        self.adj[a].append(Edge(b, length, speed_mps, kind))
        self.adj[b].append(Edge(a, length, speed_mps, kind))

    def edge(self, a: str, b: str) -> Edge:
        for e in self.adj[a]:
            if e.to == b:
                return e
        raise KeyError(f"no road {a} -> {b}")

    def nearest_node(self, pos: tuple[float, float]) -> str:
        return min(self.nodes, key=lambda n: haversine_m(self.nodes[n], pos))

    def shortest_path(self, src: str, dst: str) -> list[str]:
        """Dijkstra on free-flow travel time. The graph is ~100 nodes, so a
        plain binary heap is plenty."""
        if src == dst:
            return [src]
        dist = {src: 0.0}
        prev: dict[str, str] = {}
        heap = [(0.0, src)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == dst:
                break
            if d > dist.get(u, math.inf):
                continue
            for e in self.adj[u]:
                nd = d + e.time_s
                if nd < dist.get(e.to, math.inf):
                    dist[e.to] = nd
                    prev[e.to] = u
                    heapq.heappush(heap, (nd, e.to))
        if dst not in prev:
            raise ValueError(f"no path from {src} to {dst}")
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        return path[::-1]

    def path_geometry(self, path: list[str]) -> tuple[list[tuple[float, float]], list[float]]:
        """Polyline points and per-segment free-flow speeds for a node path."""
        points = [self.nodes[n] for n in path]
        speeds = [self.edge(a, b).speed_mps for a, b in zip(path, path[1:])]
        return points, speeds


def build_city(grid: int = 9, spacing_m: float = 650.0) -> RoadGraph:
    """Grid streets (13.9 m/s, 50 km/h), two diagonal arterials (16.7 m/s) and
    a ring road one block outside the grid (25 m/s, 90 km/h)."""
    g = RoadGraph()
    half = (grid - 1) / 2.0
    for i in range(grid):
        for j in range(grid):
            g.add_node(f"n{i}_{j}", offset_latlon(CITY_CENTER, (i - half) * spacing_m, (j - half) * spacing_m))
    for i in range(grid):
        for j in range(grid):
            if i + 1 < grid:
                g.add_road(f"n{i}_{j}", f"n{i + 1}_{j}", 13.9, "street")
            if j + 1 < grid:
                g.add_road(f"n{i}_{j}", f"n{i}_{j + 1}", 13.9, "street")
    # Diagonal arterials through the centre (replace nothing, just add shortcuts).
    for k in range(grid - 1):
        g.add_road(f"n{k}_{k}", f"n{k + 1}_{k + 1}", 16.7, "arterial")
        g.add_road(f"n{k}_{grid - 1 - k}", f"n{k + 1}_{grid - 2 - k}", 16.7, "arterial")

    # Ring road: a square one block outside the grid, connected at mid-sides and corners.
    ring_half = (half + 1) * spacing_m
    ring_ids: list[str] = []
    steps = grid + 1
    for s in range(4 * steps):
        side, k = divmod(s, steps)
        t = -ring_half + 2 * ring_half * k / steps
        east, north = [(t, -ring_half), (ring_half, t), (-t, ring_half), (-ring_half, -t)][side]
        rid = f"r{s}"
        g.add_node(rid, offset_latlon(CITY_CENTER, east, north))
        ring_ids.append(rid)
    for a, b in zip(ring_ids, ring_ids[1:] + ring_ids[:1]):
        g.add_road(a, b, 25.0, "ring")
    # Slip roads from the outermost grid nodes to the nearest ring node.
    last = grid - 1
    for node in [f"n0_{half:.0f}", f"n{last}_{half:.0f}", f"n{half:.0f}_0", f"n{half:.0f}_{last}",
                 "n0_0", f"n{last}_0", f"n0_{last}", f"n{last}_{last}"]:
        nearest = min(ring_ids, key=lambda r: haversine_m(g.nodes[r], g.nodes[node]))
        g.add_road(node, nearest, 16.7, "slip")

    m = grid // 2
    g.places = {
        "depot": f"n1_1",
        "market": f"n{m}_{m}",
        "hospital": f"n{grid - 2}_{m + 1}",
        "port": f"n{grid - 1}_1",
        "university": f"n2_{grid - 2}",
        "fuel": f"n{m + 1}_{m - 2}",
    }
    return g


def default_geofences(g: RoadGraph) -> list[dict]:
    """Fence definitions as plain dicts, so the same list feeds the core, the
    API and the reports."""
    def square(center: tuple[float, float], half_m: float) -> list[tuple[float, float]]:
        return [offset_latlon(center, dx, dy) for dx, dy in
                [(-half_m, -half_m), (half_m, -half_m), (half_m, half_m), (-half_m, half_m)]]

    depot = g.nodes[g.places["depot"]]
    market = g.nodes[g.places["market"]]
    # The old town is an irregular (concave) polygon around the market.
    old_town = [offset_latlon(market, dx, dy) for dx, dy in
                [(-900, -700), (300, -900), (950, -300), (500, 200), (900, 800), (-200, 950), (-800, 400)]]
    return [
        {"id": "depot", "name": "Main depot", "shape": "polygon", "vertices": square(depot, 120), "dwell_s": 600},
        {"id": "old-town", "name": "Old town (restricted)", "shape": "polygon", "vertices": old_town, "dwell_s": 1200},
        {"id": "hospital", "name": "Hospital loading bay", "shape": "circle",
         "center": g.nodes[g.places["hospital"]], "radius_m": 150, "dwell_s": 300},
        {"id": "port", "name": "Port gate", "shape": "circle", "center": g.nodes[g.places["port"]], "radius_m": 200,
         "dwell_s": 300},
        {"id": "fuel", "name": "Fuel station", "shape": "circle", "center": g.nodes[g.places["fuel"]], "radius_m": 80,
         "dwell_s": 240},
    ]
