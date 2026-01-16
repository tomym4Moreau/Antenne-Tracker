from dataclasses import dataclass
from typing import Optional, Dict, Any


@dataclass
class TrackerReference:
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_m: Optional[float] = None
    reference_set: bool = False
    gps_reference_required: bool = False


@dataclass
class TrackerOrientation:
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    yaw_source: Optional[str] = None
    mag_raw: Optional[Dict[str, Any]] = None


@dataclass
class PointingState:
    valid: bool = False
    src: Optional[str] = None

    az_deg: Optional[float] = None
    el_deg: Optional[float] = None

    az_rel_deg: Optional[float] = None
    el_rel_deg: Optional[float] = None

    slant_range_m: Optional[float] = None
    ground_range_m: Optional[float] = None