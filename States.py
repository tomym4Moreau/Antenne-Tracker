import logging
import time
import Tracker

logger = logging.getLogger(__name__)


def _ask_float(prompt: str, allow_empty: bool = False, default: float = 0.0) -> float:
    while True:
        s = input(prompt).strip()
        if allow_empty and s == "":
            return default
        try:
            return float(s)
        except ValueError:
            print("❌ Valeur invalide. Réessaie.")


class State_Init:

    def __init__(self, manager):
        self.next_state = None

        try:
            manager.sensor_manager.init()
        except Exception as e:
            logger.exception(f"Exception dans SensorManager.init(): {e}")
            manager.System_Error = True

        if manager.System_Error:
            self.next_state = "ERROR"
        else:
            self.next_state = "TRACKING"


class State_Tracker:
    def __init__(self, manager):
        self.next_state = None

        try:
            data = manager.sensor_manager.read()

            gps = data.get("gps")
            snap = data.get("telemetry") 

            now = time.monotonic()
            if now - manager._last_telemetry_log_ts >= 1.0:
                manager._last_telemetry_log_ts = now

                if snap is None:
                    logger.info("Telemetry: aucune donnée reçue")
                else:
                    att = snap.get("attitude") or {}
                    gpi = snap.get("global_position_int") or {}
                    gpsraw = snap.get("gps_raw_int") or {}

                    logger.info(
                        "Telemetry SNAP: last_rx_ts=%s yaw(rad)=%s rel_alt(mm)=%s hdg=%s "
                        "GPS fix_type=%s sats=%s lat=%s lon=%s",
                        snap.get("last_rx_ts"),
                        att.get("yaw"),
                        gpi.get("relative_alt"),
                        gpi.get("hdg"),
                        gpsraw.get("fix_type"),
                        gpsraw.get("satellites_visible"),
                        gpi.get("lat"),
                        gpi.get("lon"),
                    )

            manager.pointing.valid = False
            manager.pointing.src = None

            if gps and snap:
                tgt = Tracker.telemetry_extract_target_lla(snap)  
                if tgt is not None:
                    target_lat, target_lon, target_alt_m, src = tgt

                    sol = Tracker.compute_az_el_from_positions(
                        float(gps["lat"]),
                        float(gps["lon"]),
                        float(gps.get("alt_m", 0.0) or 0.0),
                        float(target_lat),
                        float(target_lon),
                        float(target_alt_m),
                    )

                    manager.pointing.valid = True
                    manager.pointing.src = src
                    manager.pointing.az_deg = sol["az_deg"]
                    manager.pointing.el_deg = sol["el_deg"]
                    manager.pointing.slant_range_m = sol["slant_range_m"]
                    manager.pointing.ground_range_m = sol["ground_range_m"]

                    if now - manager._last_pointing_log_ts >= 1.0:
                        manager._last_pointing_log_ts = now
                        logger.info(
                            f"Pointing: dist={manager.pointing.slant_range_m:.0f}m "
                            f"az={manager.pointing.az_deg:.1f}° el={manager.pointing.el_deg:.1f}°"
                        )

        except Exception as e:
            logger.exception(f"Exception dans SensorManager.read(): {e}")
            manager.System_Error = True

        self.next_state = "ERROR" if manager.System_Error else "TRACKING"


class State_Error:
    def __init__(self, manager):
        self.next_state = None

        try:
            manager.sensor_manager.error()
        except Exception as e:
            logger.exception(f"Exception dans SensorManager.error(): {e}")

        if manager.sensor_manager.requested_shutdown:
            self.next_state = "SHUTDOWN"
            return

        if manager.tracker_ref.gps_reference_required and (not manager.tracker_ref.reference_set):
            logger.warning("GPS: pas de fix -> demande référence manuelle utilisateur")
            print("\n" + "=" * 60)
            print("⚠️  GPS indisponible (pas de fix)")
            print("Saisis les coordonnées de RÉFÉRENCE du tracker (position fixe) :")
            print("=" * 60)

            manager.tracker_ref.lat = _ask_float("Latitude (ex: 47.7626368) : ")
            manager.tracker_ref.lon = _ask_float("Longitude (ex: -3.4013184) : ")
            manager.tracker_ref.alt_m = _ask_float(
                "Altitude en mètres (ex: 12.5) [Entrée = 0] : ",
                allow_empty=True,
                default=0.0,
            )

            manager.tracker_ref.reference_set = True
            logger.info(
                f"✓ Référence manuelle saisie: lat={manager.tracker_ref.lat}, "
                f"lon={manager.tracker_ref.lon}, alt={manager.tracker_ref.alt_m}"
            )
            print("=" * 60)
            print("✅ Référence enregistrée. Reprise du tracking...\n")

        if not manager.System_Error:
            self.next_state = "INIT"
        else:
            self.next_state = "ERROR"


class State_Shutdown:

    def __init__(self, manager):

        try:
            manager.sensor_manager.close()
            logger.info("Capteurs fermés avec succès.")
        except Exception as e:
            logger.exception(f"Erreur pendant la fermeture des capteurs: {e}")

        self.next_state = "EXIT"