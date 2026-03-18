import time
import machine
import _thread
import asyncio
import network
import socket
from mpu6050 import MPU6050
from machine import Pin, PWM

# Global variable for gyro pitch angle
sensor_data = None
data_lock = _thread.allocate_lock()

# WiFi / UDP Configuration
try:
    from config import WIFI_SSID, WIFI_PASS, CMD_PORT
except ImportError:
    WIFI_SSID = "RobotAP"
    WIFI_PASS = "robot12345"
    CMD_PORT  = 1234

ANGLE_OFFSET = 6.5

# --- Servo Configuration (GP22) ---
SERVO_PIN = 22
servo_pwm = PWM(Pin(SERVO_PIN))
servo_pwm.freq(50)

def set_servo_angle(angle):
    """
    Maps 0-90 degrees to duty_u16
    RESTRICTED TO 0-90 RANGE AS PER USER REQUEST
    """
    if angle < 0: angle = 0
    if angle > 90: angle = 90
    
    # Pulse width calculation still based on standard 180-degree PWM range
    duty = int((angle / 180) * 6554 + 1638)
    servo_pwm.duty_u16(duty)

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    try:
        wlan.ifconfig(('192.168.4.10', '255.255.255.0', '192.168.4.1', '192.168.4.1'))
    except Exception as e:
        print(f"Static IP error: {e}")

    wlan.connect(WIFI_SSID, WIFI_PASS)
    print(f"Connecting to WiFi: {WIFI_SSID}...")
    max_wait = 15
    while max_wait > 0:
        if wlan.isconnected():
            break
        max_wait -= 1
        time.sleep(1)
    if wlan.isconnected():
        wlan.config(pm=0xa11140) 
        print(f"Connected! IP: {wlan.ifconfig()[0]} (Static .10)")
    else:
        print("WiFi connection failed.")

def mpu_read_loop():
    """Main loop for high-frequency gyro data acquisition on Core 1"""
    global sensor_data
    try:
        mpu = MPU6050(bus=0, sda=4, scl=5, address=0x68)
        mpu.dmp_initialize()
        mpu.set_DMP_enabled(True)
        packet_size = mpu.DMP_get_FIFO_packet_size()
    except Exception as e:
        print(f"MPU Init Error: {e}")
        return

    while True:
        try:
            FIFO_count = mpu.get_FIFO_count()
            mpu_int_status = mpu.get_int_status()
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                mpu.reset_FIFO()
            elif (mpu_int_status & 0x02):
                while FIFO_count < packet_size:
                    FIFO_count = mpu.get_FIFO_count()
                FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
                quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = mpu.DMP_get_gravity(quat)
                _, pitch, _ = mpu.DMP_get_roll_pitch_yaw(quat, grav)
                with data_lock:
                    sensor_data = pitch + ANGLE_OFFSET
        except: pass
        time.sleep_us(10)

_thread.start_new_thread(mpu_read_loop, ())

async def servo_sync_task():
    """Syncs servo angle with gyro pitch angle within 0-90 range"""
    print("Servo task active (Range: 0° - 90°)")
    while True:
        pitch = None
        with data_lock:
            pitch = sensor_data
        
        if pitch is not None:
            # CENTER = 45° (Range: 45 - 45 = 0° to 45 + 45 = 90°)
            servo_target = 45 + pitch
            set_servo_angle(servo_target)
        await asyncio.sleep(0.01)

async def udp_recv_task():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', CMD_PORT))
    sock.setblocking(False)
    
    while True:
        try:
            while True:
                data, addr = sock.recvfrom(128)
        except OSError: pass
        await asyncio.sleep(0.05)

async def main():
    wifi_connect()
    asyncio.create_task(servo_sync_task())
    asyncio.create_task(udp_recv_task())
    
    print("System active (Servo Sync Only - 0-90 Range)")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping...")
