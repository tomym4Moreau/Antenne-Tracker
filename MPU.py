import logging
import math
import time
import smbus2

logger = logging.getLogger(__name__)

MPU_ADDR_CANDIDATES = (0x68, 0x69)
REG_WHO_AM_I = 0x75
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B

EXPECTED_WHOAMI = {
    0x70: "MPU-6500",
    0x71: "MPU-9250",
    0x73: "MPU-9255",
}

# Constantes de conversion
ACCEL_LSB_PER_G = 16384.0   
GYRO_LSB_PER_DPS = 131.0     
COMPLEMENTARY_ALPHA = 0.98

_bus = None
_addr = None
_last_whoami = None

# Etat attitude
_last_ts = None
_est_roll = 0.0   
_est_pitch = 0.0  
_est_yaw = 0.0    


def _require_bus():
    if _bus is None or _addr is None:
        raise RuntimeError("MPU non initialisé (appeler MPU.init() avant read/close)")


def _to_i16(msb: int, lsb: int) -> int:
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _accel_roll_pitch(ax_g: float, ay_g: float, az_g: float):
    roll = math.atan2(ay_g, az_g)
    pitch = math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g))
    return roll, pitch


def init(i2c_bus_id: int = 1):
    global _bus, _addr, _last_whoami, _last_ts, _est_roll, _est_pitch, _est_yaw

    _bus = smbus2.SMBus(i2c_bus_id)

    last_err = None
    for addr in MPU_ADDR_CANDIDATES:
        try:
            whoami = _bus.read_byte_data(addr, REG_WHO_AM_I)
            _addr = addr
            _last_whoami = whoami
            _bus.write_byte_data(_addr, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.01)

            chip = EXPECTED_WHOAMI.get(whoami, f"UNKNOWN(0x{whoami:02X})")
            logger.info(f"MPU détecté: addr=0x{_addr:02X}, WHO_AM_I=0x{whoami:02X} ({chip})")

            # Reset estimateur
            _last_ts = None
            _est_roll = 0.0
            _est_pitch = 0.0
            _est_yaw = 0.0
            return
        except Exception as e:
            last_err = e
            continue

    close()
    raise RuntimeError(
        f"MPU non détecté sur {tuple(hex(a) for a in MPU_ADDR_CANDIDATES)}. Dernière erreur: {last_err}"
    )


def read():
    global _last_ts, _est_roll, _est_pitch, _est_yaw

    _require_bus()

    block = _bus.read_i2c_block_data(_addr, REG_ACCEL_XOUT_H, 14)
    ax = _to_i16(block[0], block[1])
    ay = _to_i16(block[2], block[3])
    az = _to_i16(block[4], block[5])
    temp_raw = _to_i16(block[6], block[7])
    temp_c = (temp_raw / 333.87) + 21.0
    gx = _to_i16(block[8], block[9])
    gy = _to_i16(block[10], block[11])
    gz = _to_i16(block[12], block[13])

    now = time.monotonic()
    if _last_ts is None:
        dt = 0.0
        _last_ts = now
    else:
        dt = now - _last_ts
        _last_ts = now

    # Convert raw
    ax_g = ax / ACCEL_LSB_PER_G
    ay_g = ay / ACCEL_LSB_PER_G
    az_g = az / ACCEL_LSB_PER_G

    gx_dps = gx / GYRO_LSB_PER_DPS
    gy_dps = gy / GYRO_LSB_PER_DPS
    gz_dps = gz / GYRO_LSB_PER_DPS

    # Roll/pitch accel
    roll_acc, pitch_acc = _accel_roll_pitch(ax_g, ay_g, az_g)

    if dt > 0.0:
        # Intégration gyro (rad/s)
        roll_gyro = _est_roll + math.radians(gx_dps) * dt
        pitch_gyro = _est_pitch + math.radians(gy_dps) * dt
        yaw_gyro = _est_yaw + math.radians(gz_dps) * dt

        # Fusion complémentaire (roll/pitch)
        _est_roll = COMPLEMENTARY_ALPHA * roll_gyro + (1.0 - COMPLEMENTARY_ALPHA) * roll_acc
        _est_pitch = COMPLEMENTARY_ALPHA * pitch_gyro + (1.0 - COMPLEMENTARY_ALPHA) * pitch_acc
        _est_yaw = _wrap_pi(yaw_gyro)
    else:
        _est_roll = roll_acc
        _est_pitch = pitch_acc

    return {
        "addr": _addr,
        "whoami": _last_whoami,
        "accel_raw": {"x": ax, "y": ay, "z": az},
        "gyro_raw": {"x": gx, "y": gy, "z": gz},
        "temp_raw": temp_raw,
        "temp_c": temp_c,
        "dt_s": dt,
        "orientation_deg": {
            "roll": math.degrees(_est_roll),
            "pitch": math.degrees(_est_pitch),
            "yaw": math.degrees(_est_yaw),
            "yaw_source": "gyro",  
        },
        "mag_raw": None,
    }


def close():
    global _bus, _addr, _last_whoami, _last_ts

    if _bus is not None:
        try:
            _bus.close()
        except Exception:
            pass

    _bus = None
    _addr = None
    _last_whoami = None
    _last_ts = None