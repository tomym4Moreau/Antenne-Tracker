import glob
import logging
import time
from typing import Optional, Dict, Any, List, Tuple

from pymavlink import mavutil

logger = logging.getLogger(__name__)

_conn = None
_port: Optional[str] = None
_baud: Optional[int] = None

# Snapshot agrégé (dernières valeurs connues)
_snapshot: Optional[Dict[str, Any]] = None


def _candidate_ports() -> List[str]:
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    ports += ["/dev/serial0", "/dev/ttyS0", "/dev/ttyAMA0"]
    uniq: List[str] = []
    for p in ports:
        if p not in uniq:
            if p == "/dev/serial0" or glob.glob(p) or p.startswith("/dev/tty"):
                uniq.append(p)
    return uniq


def _set_target_from_heartbeat(c) -> Tuple[int, int]:
    hb = c.wait_heartbeat(timeout=2.0)
    if hb is None:
        raise TimeoutError("Pas de heartbeat reçu (timeout)")
    sysid = int(hb.get_srcSystem())
    compid = int(hb.get_srcComponent())
    c.target_system = sysid
    c.target_component = compid
    return sysid, compid


def _request_streams(c, hz: float = 10.0) -> None:
    ts = int(getattr(c, "target_system", 0) or 1)
    tc = int(getattr(c, "target_component", 0) or 1)

    def set_interval(msg_id: int, rate_hz: float):
        if rate_hz <= 0:
            return
        interval_us = int(1_000_000 / rate_hz)
        c.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    # Messages utiles
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 5.0)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5.0)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5.0)

    # Fallback legacy
    try:
        c.mav.request_data_stream_send(
            ts,
            tc,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(hz),
            1
        )
    except Exception:
        pass


def init(
    baud: int = 57600,
    port: Optional[str] = None,
    ports: Optional[List[str]] = None,
    heartbeat_timeout_s: float = 2.0,
    request_streams: bool = True,
):
    global _conn, _port, _baud, _snapshot

    close()
    _snapshot = None

    if port is not None:
        ports_to_try = [port]
    else:
        ports_to_try = ports if ports is not None else _candidate_ports()

    if not ports_to_try:
        raise RuntimeError("Aucun port télémétrie détecté")

    last_err = None
    for p in ports_to_try:
        try:
            logger.info(f"Télémétrie: ouverture {p} @ {baud}")
            c = mavutil.mavlink_connection(
                p,
                baud=baud,
                autoreconnect=False,
                source_system=255,
            )

            if heartbeat_timeout_s and heartbeat_timeout_s > 0:
                try:
                    sysid, compid = _set_target_from_heartbeat(c)
                    logger.info(f"Télémétrie: heartbeat OK (sysid={sysid} compid={compid})")
                except Exception:
                    logger.warning(f"Télémétrie: pas de heartbeat en {heartbeat_timeout_s}s sur {p} (on continue)")

            if request_streams:
                try:
                    _request_streams(c, hz=10.0)
                    logger.info("Télémétrie: demande de streams envoyée (ATTITUDE/AHRS2/GLOBAL_POSITION_INT/VFR_HUD/GPS_RAW_INT)")
                except Exception as e:
                    logger.warning(f"Télémétrie: impossible de demander les streams: {e}")

            _conn = c
            _port = p
            _baud = baud
            _snapshot = {
                "meta": {
                    "port": _port,
                    "baud": _baud,
                    "target_system": getattr(_conn, "target_system", None),
                    "target_component": getattr(_conn, "target_component", None),
                },
                "last_rx_ts": None,
                "attitude": None,
                "ahrs2": None,
                "global_position_int": None,
                "gps_raw_int": None,
                "vfr_hud": None,
            }
            return

        except Exception as e:
            last_err = e
            try:
                c.close()
            except Exception:
                pass

    raise RuntimeError(f"Impossible d'ouvrir la télémétrie. Dernière erreur: {last_err}")


def _ensure_snapshot() -> Dict[str, Any]:
    global _snapshot
    if _snapshot is None:
        _snapshot = {
            "meta": {"port": _port, "baud": _baud},
            "last_rx_ts": None,
            "attitude": None,
            "ahrs2": None,
            "global_position_int": None,
            "gps_raw_int": None,
            "vfr_hud": None,
        }
    return _snapshot


def read_snapshot(max_msgs: int = 50) -> Optional[Dict[str, Any]]:

    global _snapshot

    if _conn is None:
        raise RuntimeError("Télémétrie non initialisée (appeler telemetry.init() avant read_snapshot/close)")

    snap = _ensure_snapshot()
    got_any = False

    for _ in range(max_msgs):
        msg = _conn.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue

        got_any = True
        now = time.time()
        mtype = msg.get_type()
        d = msg.to_dict()
        d["_type"] = mtype
        d["ts"] = now
        d["port"] = _port
        d["baud"] = _baud

        snap["last_rx_ts"] = now

        if mtype == "ATTITUDE":
            snap["attitude"] = d
        elif mtype == "AHRS2":
            snap["ahrs2"] = d
        elif mtype == "GLOBAL_POSITION_INT":
            snap["global_position_int"] = d
        elif mtype == "GPS_RAW_INT":
            snap["gps_raw_int"] = d
        elif mtype == "VFR_HUD":
            snap["vfr_hud"] = d
        else:
            pass

    _snapshot = snap
    if got_any:
        return snap
    return snap if snap.get("last_rx_ts") is not None else None


def snapshot() -> Optional[Dict[str, Any]]:
    return _snapshot


def close():
    global _conn, _port, _baud, _snapshot
    if _conn is not None:
        try:
            _conn.close()
        except Exception:
            pass
    _conn = None
    _port = None
    _baud = None
    _snapshot = None

