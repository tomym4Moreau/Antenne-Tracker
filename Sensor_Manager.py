import logging
import time
import Sensor.MPU as MPU
import Sensor.GPS as GPS
import Sensor.Telemetry as Telemetry

logger = logging.getLogger(__name__)


class SensorManager:

    def __init__(self, manager):
        self.manager = manager

        # Configuration de la récupération des erreurs
        self._max_attempts_recover = 5
        self._recover_attempts = 0

        # Timeout GPS 
        self._gps_fix_timeout_s = 10.0
        self._gps_no_fix_since = None

        self.requested_shutdown = False

    def init(self):

        # Flags "Available"
        self.manager.MPU_Available = False
        self.manager.GPS_Available = False
        self.manager.ELRS_Available = False
        self.manager.Telemetry_Available = False

        # Flags "Error"
        self.manager.GPS_Error = False
        self.manager.MPU_Error = False
        self.manager.Com_Error = False
        self.manager.ELRS_Error = False

        # Reset timer GPS
        self._gps_no_fix_since = None

        # MPU
        try:
            MPU.init()
            self.manager.MPU_Available = True
            self.manager.MPU_Error = False
            logger.info("✓ MPU init OK")
        except Exception as e:
            self.manager.MPU_Available = False
            self.manager.MPU_Error = True
            logger.exception(f"MPU init FAILED: {e}")

        # GPS
        try:
            GPS.init()
            self.manager.GPS_Available = True
            self.manager.GPS_Error = False
            logger.info("✓ GPS init OK")
        except Exception as e:
            self.manager.GPS_Available = False
            self.manager.GPS_Error = True
            logger.exception(f"GPS init FAILED: {e}")

        # Telemetry
        try:
            Telemetry.init(baud=57600, port="/dev/ttyUSB0", heartbeat_timeout_s=2.0)
            self.manager.Telemetry_Available = True
            self.manager.Com_Error = False
            logger.info("✓ Telemetry init OK")
        except Exception as e:
            self.manager.Telemetry_Available = False
            self.manager.Com_Error = True
            logger.warning(f"Telemetry init FAILED: {e}")

        self.manager.System_Error = bool(self.manager.MPU_Error)

    def read(self):
        data = {}

        # MPU
        if self.manager.MPU_Available:
            try:
                m = MPU.read()
                data["mpu"] = m

                ori = (m or {}).get("orientation_deg") or {}
                self.manager.tracker_ori.roll_deg = ori.get("roll")
                self.manager.tracker_ori.pitch_deg = ori.get("pitch")
                self.manager.tracker_ori.yaw_deg = ori.get("yaw")
                self.manager.tracker_ori.yaw_source = ori.get("yaw_source")
                self.manager.tracker_ori.mag_raw = (m or {}).get("mag_raw")

            except Exception as e:
                self.manager.MPU_Available = False
                self.manager.MPU_Error = True
                self.manager.System_Error = True
                logger.exception(f"MPU read FAILED: {e}")

        # GPS
        if self.manager.GPS_Available and (not self.manager.tracker_ref.gps_reference_required):
            try:
                fix = GPS.read(max_lines=2)
                data["gps"] = fix

                if fix is None:
                    now = time.monotonic()
                    if self._gps_no_fix_since is None:
                        self._gps_no_fix_since = now
                        logger.debug("GPS: démarrage chrono 'pas de fix'")
                    else:
                        elapsed = now - self._gps_no_fix_since
                        if elapsed >= self._gps_fix_timeout_s:
                            self.manager.tracker_ref.gps_reference_required = True
                            logger.warning(
                                f"GPS: pas de fix depuis {elapsed:.0f}s -> référence manuelle requise"
                            )
                else:
                    self._gps_no_fix_since = None

            except Exception as e:
                self.manager.GPS_Available = False
                self.manager.GPS_Error = True
                logger.exception(f"GPS read FAILED: {e}")
                
        if self.manager.tracker_ref.reference_set:
            data["gps"] = {
                "lat": self.manager.tracker_ref.lat,
                "lon": self.manager.tracker_ref.lon,
                "alt_m": self.manager.tracker_ref.alt_m or 0.0,
                "fix_quality": 99,
                "source": "manual",
            }

        if self.manager.Telemetry_Available:
            try:
                snap = Telemetry.read_snapshot(max_msgs=50)
                if snap is not None:
                    data["telemetry"] = snap
            except Exception as e:
                self.manager.Telemetry_Available = False
                self.manager.Com_Error = True
                logger.exception(f"Telemetry read FAILED: {e}")

        self.manager.System_Error = bool(self.manager.MPU_Error)
        return data

    def error(self):

        if self.requested_shutdown:
            return

        if self._recover_attempts >= self._max_attempts_recover:
            logger.error("Nombre d'erreurs global atteint -> SHUTDOWN demandé")
            self.requested_shutdown = True
            return

        if self.manager.MPU_Error and (not self.manager.MPU_Available):
            self._recover_attempts += 1
            try:
                MPU.init()
                self.manager.MPU_Available = True
                self.manager.MPU_Error = False
                logger.info("✓ MPU recovery OK")
            except Exception as e:
                logger.warning(f"MPU recovery FAILED: {e}")

        if self.manager.GPS_Error and (not self.manager.GPS_Available):
            self._recover_attempts += 1
            try:
                GPS.init()
                self.manager.GPS_Available = True
                self.manager.GPS_Error = False
                self._gps_no_fix_since = None
                logger.info("✓ GPS recovery OK")
            except Exception as e:
                logger.warning(f"GPS recovery FAILED: {e}")
        self.manager.System_Error = bool(self.manager.MPU_Error)

    def close(self):

        try:
            MPU.close()
        except Exception as e:
            logger.debug(f"MPU close ignored: {e}")

        try:
            GPS.close()
        except Exception as e:
            logger.debug(f"GPS close ignored: {e}")

        try:
            Telemetry.close()
        except Exception as e:
            logger.debug(f"Telemetry close ignored: {e}")