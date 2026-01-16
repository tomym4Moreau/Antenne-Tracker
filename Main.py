import logging
import os
import time
from datetime import datetime
from States import State_Init, State_Tracker, State_Error, State_Shutdown
from Sensor_Manager import SensorManager
from DataClass import TrackerReference, TrackerOrientation, PointingState  # <- import PointingState

current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"log-{current_time}.txt"
log_dir = os.path.join(os.getcwd(), "Log")
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, log_filename)

root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)

if not root_logger.handlers:
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)  # <- console propre (DEBUG seulement dans le fichier)

    fh = logging.FileHandler(log_path, mode="w", encoding="utf-8")
    fh.setLevel(logging.DEBUG)

    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    root_logger.addHandler(ch)
    root_logger.addHandler(fh)
logger = logging.getLogger(f"logger_{current_time}")

class Manager:
    def __init__(self):
        self.current_state = "INIT"
        self.sensor_manager = SensorManager(self)

        # Indicateurs de disponibilité du matériel
        self.MPU_Available = False
        self.GPS_Available = False
        self.ELRS_Available = False
        self.Telemetry_Available = False

        # Indicateurs d'erreur système
        self.System_Error = False
        self.GPS_Error = False
        self.MPU_Error = False
        self.Com_Error = False
        self.ELRS_Error = False

        # Données tracker (dataclasses)
        self.tracker_ref = TrackerReference()
        self.tracker_ori = TrackerOrientation()
        self.pointing = PointingState()
        self._last_pointing_log_ts = 0.0
        self._last_telemetry_log_ts = 0.0

        # Machine à états
        self.states = {
            "INIT": State_Init,
            "TRACKING": State_Tracker,
            "SHUTDOWN": State_Shutdown,
            "ERROR": State_Error,
        }

    def run(self):
        while self.current_state != "EXIT":

            if (
                self.tracker_ref.gps_reference_required
                and (not self.tracker_ref.reference_set)
                and self.current_state not in ["ERROR", "EXIT"]
            ):
                logger.warning("GPS: référence manuelle requise -> transition vers ERROR")
                self.current_state = "ERROR"

            if self.System_Error and self.current_state not in ["ERROR", "EXIT"]:
                logger.warning("Erreur détectée -> transition vers ERROR")
                self.current_state = "ERROR"

            state_cls = self.states.get(self.current_state)
            if state_cls is None:
                logger.error(f"État inconnu: {self.current_state}")
                self.System_Error = True
                self.current_state = "ERROR"
                continue

            state_instance = state_cls(self)
            next_state = getattr(state_instance, "next_state", None)

            if next_state and next_state != self.current_state:
                logger.info(f"Transition: {self.current_state} -> {next_state}")
                self.current_state = next_state

            time.sleep(0.05)


if __name__ == "__main__":
    manager = Manager()
    try:
        manager.run()
    except KeyboardInterrupt:
        manager.current_state = "SHUTDOWN"
        if "SHUTDOWN" in manager.states:
            manager.states["SHUTDOWN"](manager)
    finally:
        logger.info("Programme terminé.")