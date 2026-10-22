"""HTTP service over the fleet engine (in-memory, single process).

The README Quickstart shows how to serve it.
"""

from __future__ import annotations

from typing import Literal

from fastapi import Depends, FastAPI, HTTPException, Query
from pydantic import BaseModel, Field, model_validator

from fleetline.engine import FleetEngine


class PointIn(BaseModel):
    vehicle_id: str = Field(min_length=1, max_length=64)
    ts: float
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    speed_mps: float | None = Field(default=None, ge=0, le=100)
    heading_deg: float = Field(default=0.0, ge=0, lt=360)
    ignition: bool = False


class IngestRequest(BaseModel):
    points: list[PointIn] = Field(min_length=1, max_length=50_000)


class IngestResponse(BaseModel):
    accepted: int
    processed: int


class GeofenceIn(BaseModel):
    id: str = Field(min_length=1, max_length=64)
    name: str = ""
    shape: Literal["polygon", "circle"]
    vertices: list[tuple[float, float]] | None = None
    center: tuple[float, float] | None = None
    radius_m: float | None = Field(default=None, gt=0)
    dwell_s: float = Field(default=300.0, gt=0)

    @model_validator(mode="after")
    def _check_shape(self) -> GeofenceIn:
        if self.shape == "polygon" and (not self.vertices or len(self.vertices) < 3):
            raise ValueError("polygon needs at least 3 vertices")
        if self.shape == "circle" and (self.center is None or self.radius_m is None):
            raise ValueError("circle needs center and radius_m")
        return self


def create_app(engine: FleetEngine | None = None) -> FastAPI:
    app = FastAPI(title="fleetline", version="0.1.0",
                  description="Fleet telematics lab: ingestion, live status, trips, geofence events and ETA.")
    app.state.engine = engine or FleetEngine()

    def get_engine() -> FleetEngine:
        return app.state.engine

    def require_vehicle(vehicle_id: str, eng: FleetEngine = Depends(get_engine)) -> str:
        if eng.status(vehicle_id) is None:
            raise HTTPException(status_code=404, detail=f"unknown vehicle {vehicle_id!r}")
        return vehicle_id

    @app.get("/health")
    def health(eng: FleetEngine = Depends(get_engine)) -> dict:
        return {"status": "ok", **eng.stats()}

    @app.post("/points", response_model=IngestResponse)
    def ingest(req: IngestRequest, eng: FleetEngine = Depends(get_engine)) -> IngestResponse:
        processed = eng.ingest_dicts(p.model_dump() for p in req.points)
        return IngestResponse(accepted=len(req.points), processed=processed)

    @app.post("/flush")
    def flush(eng: FleetEngine = Depends(get_engine)) -> dict:
        eng.flush()
        return eng.stats()

    @app.get("/vehicles")
    def vehicles(eng: FleetEngine = Depends(get_engine)) -> list[dict]:
        return eng.vehicles()

    @app.get("/vehicles/{vehicle_id}")
    def vehicle(vehicle_id: str = Depends(require_vehicle), eng: FleetEngine = Depends(get_engine)) -> dict:
        return eng.status(vehicle_id)

    @app.get("/vehicles/{vehicle_id}/trips")
    def trips(vehicle_id: str = Depends(require_vehicle), eng: FleetEngine = Depends(get_engine)) -> list[dict]:
        return eng.trips(vehicle_id)

    @app.get("/vehicles/{vehicle_id}/stops")
    def stops(vehicle_id: str = Depends(require_vehicle), eng: FleetEngine = Depends(get_engine)) -> list[dict]:
        return eng.stops(vehicle_id)

    @app.get("/geofences")
    def geofences(eng: FleetEngine = Depends(get_engine)) -> list[dict]:
        return eng.geofences()

    @app.put("/geofences/{fence_id}", status_code=201)
    def put_geofence(fence_id: str, fence: GeofenceIn, eng: FleetEngine = Depends(get_engine)) -> dict:
        if fence.id != fence_id:
            raise HTTPException(status_code=422, detail="path id and body id differ")
        eng.add_geofence(fence.model_dump())
        return {"id": fence_id}

    @app.get("/geofence-events")
    def geofence_events(vehicle_id: str | None = None, fence_id: str | None = None,
                        type: Literal["enter", "exit", "dwell"] | None = None,
                        eng: FleetEngine = Depends(get_engine)) -> list[dict]:
        events = eng.geofence_events(vehicle_id)
        if fence_id:
            events = [e for e in events if e["fence_id"] == fence_id]
        if type:
            events = [e for e in events if e["type"] == type]
        return events

    @app.get("/vehicles/{vehicle_id}/eta")
    def eta(vehicle_id: str = Depends(require_vehicle), place: str | None = None,
            lat: float | None = Query(default=None, ge=-90, le=90),
            lon: float | None = Query(default=None, ge=-180, le=180),
            eng: FleetEngine = Depends(get_engine)) -> dict:
        try:
            dest = eng.resolve_destination(place, lat, lon)
        except KeyError as e:
            raise HTTPException(status_code=404, detail=str(e.args[0])) from None
        except ValueError as e:
            raise HTTPException(status_code=422, detail=str(e)) from None
        result = eng.eta(vehicle_id, dest)
        if result is None:
            raise HTTPException(status_code=409, detail="vehicle has no processed position yet")
        return result

    return app


app = create_app()
