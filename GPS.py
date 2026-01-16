import logging
import os
import time
import serial
import pynmea2
from typing import Optional, Dict, Any, List

logger = logging.getLogger(__name__)
_ser = None
_port = None
_baud = None
_last_fix: Optional[Dict[str, Any]] = None


def _candidate_ports() -> List[str]:
    candidates = [
        "/dev/serial0",
        "/dev/serial1",
        "/dev/ttyAMA0",
        "/dev/ttyS0",
    ]
    return [p for p in candidates if os.path.exists(p)]


def init(port: Optional[str] = None, baud: int = 115200, timeout: float = 0.05):
    global _ser, _port, _baud, _last_fix

    close()
    _last_fix = None
    ports_to_try = [port] if port else _candidate_ports()
    if not ports_to_try:
        raise RuntimeError(
            "Aucun port GPS détecté. Sur Windows, passe port='COMx'. "
            "Sur Linux, vérifie /dev/serial0 etc."
        )

    last_err = None
    for p in ports_to_try:
        try:
            logger.debug(f"Tentative ouverture GPS: {p} @ {baud} ...")
            _ser = serial.Serial(port=p, baudrate=baud, timeout=timeout)
            _port = p
            _baud = baud
            logger.debug(f"GPS ouvert: {p} @ {baud}")
            return
        except Exception as e:
            last_err = e
            _ser = None

    raise RuntimeError(f"Impossible d'ouvrir le GPS. Dernière erreur: {last_err}")


def read(max_lines: int = 10) -> Optional[Dict[str, Any]]:
    global _last_fix
    if _ser is None:
        raise RuntimeError("GPS non initialisé (appeler GPS.init() avant read/close)")

    for _ in range(max_lines):
        raw = _ser.readline()
        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        if not line.startswith("$"):
            continue

        if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
            continue

        try:
            msg = pynmea2.parse(line)

            fix_quality = int(getattr(msg, "gps_qual", 0) or 0)
            lat = getattr(msg, "latitude", None)
            lon = getattr(msg, "longitude", None)

            if lat is None or lon is None:
                continue

            alt = getattr(msg, "altitude", None)
            try:
                alt_m = float(alt) if alt is not None else 0.0
            except Exception:
                alt_m = 0.0

            satellites = getattr(msg, "num_sats", None)
            try:
                satellites_i = int(satellites) if satellites is not None else None
            except Exception:
                satellites_i = None

            hdop = getattr(msg, "horizontal_dil", None)
            try:
                hdop_f = float(hdop) if hdop is not None else None
            except Exception:
                hdop_f = None

            fix = {
                "lat": float(lat),
                "lon": float(lon),
                "alt_m": alt_m,
                "fix_quality": fix_quality,
                "satellites": satellites_i,
                "hdop": hdop_f,
                "utc": getattr(msg, "timestamp", None) or getattr(msg, "time", None),
                "raw": line,
                "port": _port,
                "baud": _baud,
                "ts": time.time(),
            }

            _last_fix = fix

            if fix_quality < 1: 
                _last_fix = fix
                return None

            return fix
        except Exception:
            continue

    return None


def close():
    global _ser, _port, _baud
    if _ser is not None:
        try:
            _ser.close()
        except Exception:
            pass
    _ser = None
    _port = None
    _baud = None


def last_fix() -> Optional[Dict[str, Any]]:
    return _last_fix