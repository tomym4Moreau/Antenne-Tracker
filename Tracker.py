import logging
import math
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

def _deg2rad(x: float) -> float:
    return x * math.pi / 180.0


def _rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


def normalize_heading_deg(deg: float) -> float:
    """Force un angle en degrés dans [0, 360)."""
    deg = deg % 360.0
    return deg + 360.0 if deg < 0 else deg


def geodetic_lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    """LLA (deg, deg, m) -> ECEF (m, m, m)."""
    lat = _deg2rad(lat_deg)
    lon = _deg2rad(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)

    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - _WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_local_enu(
    target_x: float,
    target_y: float,
    target_z: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_x: float,
    ref_y: float,
    ref_z: float,
) -> Tuple[float, float, float]:

    lat = _deg2rad(ref_lat_deg)
    lon = _deg2rad(ref_lon_deg)

    dx = target_x - ref_x
    dy = target_y - ref_y
    dz = target_z - ref_z

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def compute_az_el_from_positions(
    tracker_lat: float,
    tracker_lon: float,
    tracker_alt_m: float,
    target_lat: float,
    target_lon: float,
    target_alt_m: float,
) -> Dict[str, float]:

    ref_x, ref_y, ref_z = geodetic_lla_to_ecef(tracker_lat, tracker_lon, tracker_alt_m)
    x, y, z = geodetic_lla_to_ecef(target_lat, target_lon, target_alt_m)

    e, n, u = ecef_to_local_enu(x, y, z, tracker_lat, tracker_lon, ref_x, ref_y, ref_z)

    ground = math.hypot(e, n)
    slant = math.sqrt(e * e + n * n + u * u)

    az = normalize_heading_deg(_rad2deg(math.atan2(e, n)))
    el = _rad2deg(math.atan2(u, ground)) if ground > 1e-6 else (90.0 if u > 0 else -90.0)

    return {
        "az_deg": az,
        "el_deg": el,
        "ground_range_m": ground,
        "slant_range_m": slant,
    }


def telemetry_extract_target_lla(
    tel: Optional[Dict[str, Any]],
) -> Optional[Tuple[float, float, float, str]]:
    
    if not tel:
        return None

    mtype = tel.get("_type") or tel.get("mavpackettype")
    if not mtype:
        return None

    try:
        if mtype == "GLOBAL_POSITION_INT":
            lat = float(tel["lat"]) * 1e-7
            lon = float(tel["lon"]) * 1e-7
            alt_m = float(tel.get("alt", 0.0)) * 1e-3
            return lat, lon, alt_m, "GLOBAL_POSITION_INT"

        if mtype == "GPS_RAW_INT":
            lat = float(tel["lat"]) * 1e-7
            lon = float(tel["lon"]) * 1e-7
            alt_m = float(tel.get("alt", 0.0)) * 1e-3
            return lat, lon, alt_m, "GPS_RAW_INT"

        if mtype == "AHRS2":
            # AHRS2: lat/lng (souvent en 1e7 deg), altitude (m), yaw (rad)
            lat_i = tel.get("lat")
            lon_i = tel.get("lng")
            if lat_i is None or lon_i is None:
                return None

            lat = float(lat_i) * 1e-7
            lon = float(lon_i) * 1e-7
            alt_m = float(tel.get("altitude", 0.0) or 0.0)

            # filtre basique: ignore lat/lon ~ 0 si pas de GPS
            if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                return None

            return lat, lon, alt_m, "AHRS2"

        return None
    except Exception as e:
        logger.debug(f"telemetry_extract_target_lla failed: type={mtype} err={e}")
        return None


def az_el_world_to_mount_commands(
    az_world_deg: float,
    el_world_deg: float,
    tracker_heading_deg: Optional[float],
) -> Dict[str, float]:

    if tracker_heading_deg is None:
        return {"az_rel_deg": az_world_deg, "el_rel_deg": el_world_deg}

    return {
        "az_rel_deg": normalize_heading_deg(az_world_deg - tracker_heading_deg),
        "el_rel_deg": el_world_deg,
    }


wrap_360 = normalize_heading_deg
lla_to_ecef = geodetic_lla_to_ecef
ecef_to_enu = ecef_to_local_enu
compute_pointing_solution = compute_az_el_from_positions
extract_target_lla_from_telemetry = telemetry_extract_target_lla
compute_relative_commands = az_el_world_to_mount_commands