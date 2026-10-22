"""fleetline: fleet telematics lab.

The hot path (filtering, geofencing, trip segmentation, ETA, the per-vehicle
stream processor) lives in the C++ extension ``fleetline._core``. This package
adds orchestration on top: a synthetic fleet simulator, an HTTP service and
tabular analytics.
"""

from fleetline._core import (  # noqa: F401
    FilterConfig,
    Geofence,
    GeofenceEventType,
    HarshType,
    LatLon,
    MotionState,
    PositionFilter,
    Route,
    StreamConfig,
    StreamProcessor,
    TelemetryPoint,
    TripConfig,
    TripSegmenter,
    bearing_deg,
    haversine_m,
    point_in_polygon,
)

__version__ = "0.1.0"
