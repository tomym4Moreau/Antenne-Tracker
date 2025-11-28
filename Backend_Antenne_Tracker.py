import logging
import os
from datetime import datetime
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
import serial
import time
import pynmea2
import smbus2
import math


# Configuration du logger
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"log-{current_time}.txt"
log_dir = os.path.join(os.getcwd(), "Log")
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, log_filename)
logger = logging.getLogger(f"logger_{current_time}")
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
fh = logging.FileHandler(log_path, mode='w', encoding='utf-8')
fh.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
fh.setFormatter(formatter)
logger.addHandler(ch)
logger.addHandler(fh)
logger.debug('Debug message')
logger.info('Info message')
logger.warning('Warning message')
logger.error('Error message')
logger.critical('Critical message')

class Manager:
    def __init__(self):
        self.current_state = "INIT"
        self.has_error = False
        self.gps_serial = None
        self.mag_available = False
        self.mag_bus = None
        self.mag_address = None
        
        # Données GPS/Position
        self.tracker_lat = None
        self.tracker_lon = None
        self.drone_lat = None
        self.drone_lon = None
        self.drone_alt = None
        
        # Orientation
        self.magnetic_heading = 0.0
        self.target_azimuth = 0.0
        self.target_elevation = 0.0
        
        self.states = {
            "INIT": State_Init,
            "TRACKING": State_Tracker,
            "ERROR": State_Error,
            "SHUTDOWN": State_Shutdown
        }
    
    def run(self):
        while self.current_state != "EXIT":
            logger.info(f"État actuel: {self.current_state}")

            if self.has_error and self.current_state != "ERROR":
                logger.warning("Erreur détectée! Transition vers ERROR")
                self.current_state = "ERROR"

            if self.current_state in self.states:
                state_instance = self.states[self.current_state](self)
                next_state = state_instance.next_state
                if next_state:
                    logger.info(f"Transition: {self.current_state} -> {next_state}")
                    self.current_state = next_state
            else:
                logger.error(f"État inconnu: {self.current_state}")
                self.has_error = True


class State_Init:
    def __init__(self, manager):
        logger.info("Initializing system...")
        self.next_state = self.log_check(manager)

    def log_check(self, manager):
        if os.path.exists(log_path):
            logger.info("Logs found for the current time.")
            return self.gps_check(manager)
        else:
            logger.warning("No logs found for the current time.")
            manager.has_error = True
            return "ERROR"
    
    def gps_check(self, manager):
        gps_available = False
        serial_ports = ['/dev/serial0', '/dev/ttyS0']
        
        for port in serial_ports:
            try:
                manager.gps_serial = serial.Serial(
                    port=port, 
                    baudrate=9600,
                    timeout=2
                )
                logger.info(f"Port série {port} ouvert avec succès")
                logger.info("Tentative de lecture GPS...")
                start_time = time.time()

                while time.time() - start_time < 5:
                    if manager.gps_serial.in_waiting > 0:
                        line = manager.gps_serial.readline().decode('ascii', errors='ignore')
                        if line.startswith('$GP') or line.startswith('$GN'):
                            logger.info(f"GPS détecté sur {port}! Données reçues: {line.strip()}")
                            gps_available = True
                            break
                
                if gps_available:
                    break
                else:
                    logger.warning(f"Port {port} ouvert mais aucune donnée NMEA reçue")
                    if manager.gps_serial.is_open:
                        manager.gps_serial.close()
                    
            except serial.SerialException as e:
                logger.warning(f"Erreur de connexion au GPS sur {port}: {e}")
            except Exception as e:
                logger.warning(f"Erreur lors de la vérification GPS sur {port}: {e}")
        
        mag_available = self.check_magnetometer(manager)

        if gps_available:
            logger.info("GPS disponible, attente du fix...")
            return self.wait_for_gps_fix(manager)
        elif mag_available:
            logger.info("Magnétomètre disponible.")
            manager.mag_available = True
            manager.has_error = False
            return "TRACKING"
        else:
            logger.error("Ni GPS ni magnétomètre disponibles")
            manager.has_error = True
            return "ERROR"
    
    def wait_for_gps_fix(self, manager):
        """Attendre que le GPS ait un fix avant de passer en TRACKING"""
        logger.info("Attente du fix GPS...")
        last_log_time = time.time()
        satellites_count = 0
        fix_quality = 0
        
        try:
            while True:
                if manager.gps_serial and manager.gps_serial.in_waiting > 0:
                    line = manager.gps_serial.readline().decode('ascii', errors='ignore')

                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            fix_quality = msg.gps_qual
                            satellites_count = msg.num_sats if msg.num_sats else 0

                            if fix_quality >= 1 and msg.latitude and msg.longitude:
                                logger.info(f"Fix GPS obtenu! Qualité: {fix_quality}, Satellites: {satellites_count}")
                                logger.info(f"Position: {msg.latitude:.6f}, {msg.longitude:.6f}")
                                manager.tracker_lat = msg.latitude
                                manager.tracker_lon = msg.longitude
                                manager.has_error = False
                                return "TRACKING"
                        except pynmea2.ParseError:
                            pass

                    elif line.startswith('$GPGSA') or line.startswith('$GNGSA'):
                        try:
                            msg = pynmea2.parse(line)
                            if msg.mode_fix_type:
                                logger.debug(f"Mode fix: {msg.mode_fix_type} (1=no fix, 2=2D, 3=3D)")
                        except pynmea2.ParseError:
                            pass
                        
                    elif line.startswith('$GPGSV') or line.startswith('$GNGSV'):
                        try:
                            msg = pynmea2.parse(line)
                            if msg.num_sv_in_view:
                                satellites_count = msg.num_sv_in_view
                        except pynmea2.ParseError:
                            pass

                current_time = time.time()
                if current_time - last_log_time >= 60:
                    logger.info(f"Pas de fix GPS - Satellites visibles: {satellites_count}, Qualité: {fix_quality}")
                    last_log_time = current_time
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            logger.warning("Attente du fix GPS interrompue par l'utilisateur")
            manager.has_error = True
            return "SHUTDOWN"
        except Exception as e:
            logger.error(f"Erreur lors de l'attente du fix GPS: {e}")
            manager.has_error = True
            return "ERROR"

    def check_magnetometer(self, manager): 
        try:
            common_addresses = [0x0D, 0x1E, 0x0C]
            bus = smbus2.SMBus(1)  
            
            for addr in common_addresses:
                try:
                    bus.read_byte(addr)
                    logger.info(f"Magnétomètre détecté à l'adresse I2C: 0x{addr:02X}")
                    manager.mag_bus = bus
                    manager.mag_address = addr
                    manager.mag_available = True
                    
                    if addr == 0x0D:
                        self.init_magnetometer(manager)
                    
                    return True
                except OSError:
                    continue
            
            bus.close()
            logger.warning("Aucun magnétomètre détecté sur le bus I2C")
            return False 

        except Exception as e:
            logger.error(f"Erreur magnétomètre: {e}")  
            manager.has_error = True
            return False
    
    def init_magnetometer(self, manager):
        try:
            manager.mag_bus.write_byte_data(manager.mag_address, 0x09, 0x1D)
            manager.mag_bus.write_byte_data(manager.mag_address, 0x0A, 0x01)
            logger.info("Magnétomètre QMC5883L initialisé")
        except Exception as e:
            logger.error(f"Erreur initialisation Magnétomètre QMC5883L: {e}")


class State_Tracker:
    def __init__(self, manager):
        logger.info("Tracking antennas...")
        
        self.manager = manager
        self.tracking_active = True
        self.next_state = None
        
        try:
            self.tracking_loop()
        except KeyboardInterrupt:
            logger.info("Arrêt demandé par l'utilisateur")
            self.next_state = "SHUTDOWN"
        except Exception as e:
            logger.error(f"Erreur dans la boucle de tracking: {e}")
            manager.has_error = True
            self.next_state = "ERROR"
    
    def tracking_loop(self):
        loop_count = 0
        max_loops = 100 
        
        while self.tracking_active and loop_count < max_loops:
            loop_count += 1
            self.read_gps()
            self.read_magnetometer()

            time.sleep(0.5)  
        logger.info("Tracking terminé")
        self.next_state = "SHUTDOWN"
    
    def read_gps(self):
        try:
            if self.manager.gps_serial and self.manager.gps_serial.in_waiting > 0:
                line = self.manager.gps_serial.readline().decode('ascii', errors='ignore')
                
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    msg = pynmea2.parse(line)
                    if msg.latitude and msg.longitude:

                        if self.manager.tracker_lat is None:
                            self.manager.tracker_lat = msg.latitude
                            self.manager.tracker_lon = msg.longitude
                            logger.info(f"Position tracker enregistrée: {msg.latitude:.6f}, {msg.longitude:.6f}")
                        self.manager.drone_lat = msg.latitude + 0.001
                        self.manager.drone_lon = msg.longitude + 0.001
                        self.manager.drone_alt = 100 
                        return True
                
                elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    msg = pynmea2.parse(line)
                    if msg.latitude and msg.longitude:
                        if self.manager.tracker_lat is None:
                            self.manager.tracker_lat = msg.latitude
                            self.manager.tracker_lon = msg.longitude
                        return True
                        
        except pynmea2.ParseError as e:
            logger.warning(f"Erreur parsing NMEA: {e}")
        except Exception as e:
            logger.error(f"Erreur lecture GPS: {e}")
        
        return False
    
    def read_magnetometer(self):
        try:
            data = self.manager.mag_bus.read_i2c_block_data(self.manager.mag_address, 0x00, 6)
            x = self.convert_to_signed(data[1] << 8 | data[0])
            y = self.convert_to_signed(data[3] << 8 | data[2])
            z = self.convert_to_signed(data[5] << 8 | data[4])
            heading = math.atan2(y, x) * 180 / math.pi
            if heading < 0:
                heading += 360
            
            return heading
            
        except Exception as e:
            logger.error(f"Erreur lecture magnétomètre: {e}")
            return None
    
    def convert_to_signed(self, value):
        if value > 32767:
            return value - 65536
        return value
    


class State_Error:
    def __init__(self, manager):
        logger.error("Gestion de l'erreur...")
        
        if manager.gps_serial and manager.gps_serial.is_open:
            manager.gps_serial.close()
            logger.info("Port série GPS fermé suite à erreur")
        manager.has_error = False
        self.next_state = "SHUTDOWN"


class State_Shutdown:
    def __init__(self, manager):
        logger.info("Shutting down system...")

        if manager.gps_serial and manager.gps_serial.is_open:
            manager.gps_serial.close()
            logger.info("Port série GPS fermé proprement")

        if manager.mag_bus:
            try:
                manager.mag_bus.close()
                logger.info("Bus I2C fermé proprement")
            except:
                pass
        
        self.next_state = "EXIT"

if __name__ == "__main__":
    manager = Manager()
    manager.run()
