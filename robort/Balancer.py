import time
import machine
import _thread
import asyncio
import network
import socket
from mpu6050 import MPU6050
from encoder import Encoder
from L298N import PWM_logic
from machine import Pin

# Global variables for sharing MPU data between cores.
sensor_data = None
data_ready = False
data_lock = _thread.allocate_lock()

# WiFi / UDP Configuration
try:
    from config import WIFI_SSID, WIFI_PASS, CMD_PORT, TELE_PORT
except ImportError:
    # Fallback if config.py is not uploaded to Pico
    WIFI_SSID = "RobotAP"
    WIFI_PASS = "robot12345"
    CMD_PORT  = 1234
    TELE_PORT = 1235

# Remote control & Telemetry globals
target_linear_vel = 0.0
target_angular_vel = 0.0
last_recv_ms = 0
remote_addr = None
ANGLE_OFFSET = 6.5 # Defined globally

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    print(f"Connecting to WiFi: {WIFI_SSID}...")
    
    # Wait for connection
    max_wait = 10
    while max_wait > 0:
        if wlan.isconnected():
            break
        max_wait -= 1
        print('waiting for connection...')
        time.sleep(1)
    
    if wlan.isconnected():
        # Disable WiFi power management for better stability
        # hex 0xa11140 is the constant for 'performance' mode
        wlan.config(pm=0xa11140) 
        print(f"Connected! Pico IP: {wlan.ifconfig()[0]} (PM disabled)")
    else:
        print("WiFi connection failed.")

def get_sensor_data():
    global sensor_data, data_ready
    with data_lock:
        if data_ready and sensor_data is not None:
            data = sensor_data
            data_ready = False
            return data
    return (None, None)

def mpu_read_loop():
    global sensor_data, data_ready
    mpu = MPU6050(bus=0, sda=4, scl=5, address=0x68)
    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    packet_size = mpu.DMP_get_FIFO_packet_size()
    FIFO_buffer = [0] * 64
    
    # ANGLE_OFFSET used from global scope

    while True:
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
            # roll, _, _ = mpu.DMP_get_roll_pitch_yaw(quat, grav)
            # gx, _, _ = mpu.get_rotation()
            # with data_lock:
            #     sensor_data = (roll + ANGLE_OFFSET, -gx)
            #     data_ready = True
           # 1. get pitch
            _, pitch, _ = mpu.DMP_get_roll_pitch_yaw(quat, grav)
            # 2. get gy
            _, gy, _ = mpu.get_rotation()
            with data_lock:
                # 3. update sensor data
                sensor_data = (pitch + ANGLE_OFFSET, -gy)
                data_ready = True 
        time.sleep_us(10)

_thread.start_new_thread(mpu_read_loop, ())

class Balancer:
    def __init__(self, left_motor, right_motor, left_encoder, right_encoder):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        #self.gains = [4.5, 0.017, .57, .1]
        #self.gains = [4.5, 0.03, .97, .2]
        self.gains = [5, 0.02, .7, .15]
        self.Kg = 0.01
        self.KgRot = 0.005
        self.oldLeftP = 0
        self.oldRightP = 0
        self.PosDes  = 0
        self.YawDes = 0
        self.current_v = 0.0  # Current linear velocity for telemetry
        self.t0 = time.ticks_us()

    def ticks_to_deg(self, ticks):
        return ticks * (360.0 / 1980.0)
    
    def control(self, roll, rolldot, linearVel=0, angularVel=0):
        t1 = time.ticks_us()
        delt = time.ticks_diff(t1, self.t0) * 1e-6
        leftP = self.ticks_to_deg(self.left_encoder.position())
        rightP = self.ticks_to_deg(self.right_encoder.position())
        #print(leftP,rightP)
        leftV = (leftP - self.oldLeftP) / delt
        rightV = (rightP - self.oldRightP) / delt
        self.PosDes += linearVel * delt
        self.YawDes += angularVel * delt
        x0 = roll
        x1 = (leftP + rightP) / 2.0 - self.PosDes
        x2 = rolldot
        x3 = (leftV + rightV) / 2.0 - linearVel
        self.oldLeftP = leftP
        self.oldRightP = rightP
        self.t0 = t1
        self.current_v = (leftV + rightV) / 2.0
        
        YawEncoder = (leftP - rightP) * 1.4 / 6.0 - self.YawDes
        u1 = self.gains[0]*x0 + self.gains[1]*x1 + self.gains[2]*x2 + self.gains[3]*x3
        u1 *= self.Kg
        u2 = -self.KgRot * YawEncoder
        return u1, u2

class StateMachine:
    def __init__(self, balancer):
        self.state = "stop"
        self.balancer = balancer

    async def run(self):
        # Start remote tasks
        asyncio.create_task(self.udp_recv_task())
        # asyncio.create_task(self.telemetry_task()) # Disabled for stability
        while True:
            if self.state == "balance":
                await self.balance()
            elif self.state == "stop":
                await self.stop()
            elif self.state == "move":
                await self.move()
            elif self.state == "turnRight":
                await self.turnRight()
            await asyncio.sleep(0)

    async def udp_recv_task(self):
        global target_linear_vel, target_angular_vel, last_recv_ms, remote_addr
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', CMD_PORT))
        sock.setblocking(False)
        print(f"UDP command listener started on port {CMD_PORT}")
        
        while True:
            try:
                data, addr = sock.recvfrom(128) # Increased buffer slightly
                decoded = data.decode().strip()
                msg = decoded.split(',')
                # Our V2 remote sends: speed, turn, q0, q1, q2, q3, gx, gy, gz
                if len(msg) >= 2:
                    # Scaling: joystick ranges roughly -0.4 to 0.4
                    # Multiply by 100 to get degrees/sec or similar units for Balancer
                    target_linear_vel = float(msg[0]) * 100.0 
                    target_angular_vel = float(msg[1]) * 150.0 
                    remote_addr = addr  # Save remote IP
                    last_recv_ms = time.ticks_ms()
            except OSError:
                pass
            except Exception as e:
                # print(f"UDP Error: {e}")
                pass
            
            # Safety timeout: if no packet for 0.5s, clear targets
            if time.ticks_diff(time.ticks_ms(), last_recv_ms) > 500:
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                
            await asyncio.sleep(0.025) # 40Hz command check

    # async def telemetry_task(self):
    #     """Sends posture and speeds to both Remote Controller and PC Simulator via Broadcast."""
    #     tele_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     # 1 = SOL_SOCKET, 32 = SO_BROADCAST in MicroPython
    #     tele_sock.setsockopt(1, 32, 1) 
        
    #     BROADCAST_ADDR = "192.168.4.255"
        
    #     while True:
    #         try:
    #             # Get roll from MPU and calculate differential speeds from balancer
    #             roll, _ = get_sensor_data()
    #             if roll is not None:
    #                 # Calculate vL and vR (average for now)
    #                 v_avg = self.balancer.current_v
    #                 # Format: "roll,vL,vR"
    #                 data = "%.2f,%.1f,%.1f" % (roll, v_avg, v_avg)
    #                 tele_sock.sendto(data.encode(), (BROADCAST_ADDR, TELE_PORT))
    #         except Exception as e:
    #             pass 
    #         await asyncio.sleep(0.1)  # 10Hz Telemetry - Rock solid stability

    async def balance(self):
        print('Entered balance state')
        while self.state == "balance":
            roll, rolldot = get_sensor_data()
            if roll is not None and rolldot is not None:
                # Since roll already includes ANGLE_OFFSET, we check abs(roll)
                if abs(roll) > 45: 
                    self.state = "stop"
                else:
                    # Use remote control inputs
                    u1, u2 = self.balancer.control(roll, rolldot, target_linear_vel, target_angular_vel)
                    self.balancer.left_motor.volts(u1 + u2)
                    self.balancer.right_motor.volts(u1 - u2)
            await asyncio.sleep(0)

    async def stop(self):
        self.balancer.left_motor.stop()
        self.balancer.right_motor.stop()
        self.balancer.left_encoder.position(0)
        self.balancer.right_encoder.position(0)
        print('Entered stop state')
        while self.state == "stop":
            roll, rolldot = get_sensor_data()
            if roll is not None and rolldot is not None:
                # Trigger balance if near 0 (which is offset-corrected)
                if abs(roll) < 10: 
                    self.state = "balance"
            await asyncio.sleep(0.1) # Slow check for stop state

    async def turnRight(self):
        print('Entered turnRight state')
        t0 = time.ticks_ms() # Keep track of the time
        while self.state == "turnRight":
            # Transition back to balance after 5 seconds
            if time.ticks_diff(time.ticks_ms(), t0) > 5000:
                self.state = "balance"
            roll, rolldot = get_sensor_data()
            if roll is not None and rolldot is not None:
                if abs(roll) > 45:
                    self.state = "stop"
                else:
                    u1, u2 = self.balancer.control(roll, rolldot, 0, 90)
                    self.balancer.left_motor.volts(u1 + u2)
                    self.balancer.right_motor.volts(u1 - u2)
            await asyncio.sleep(0)

right_pos = Encoder(14, 15)
left_pos  = Encoder(16, 17)
left_motor  = PWM_logic(0, 1)
right_motor = PWM_logic(2, 3)

wifi_connect()

balancer = Balancer(left_motor, right_motor, left_pos, right_pos)
state_machine = StateMachine(balancer)
asyncio.run(state_machine.run())
