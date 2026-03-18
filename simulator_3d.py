from ursina import *
from ursina.prefabs.first_person_controller import FirstPersonController
import math
import socket
import threading
import time

# --- Configuration & Constants ---
G = 9.81            # Gravity
M_P = 0.5           # Body mass (kg)
M_W = 0.1           # Wheel mass
L = 0.15            # CM Height (m)
R = 0.045           # Wheel radius (m) - Larger for aesthetics
I_P = (M_P * (L**2)) / 3.0 
I_W = 0.5 * M_W * (R**2)

try:
    from config import CMD_PORT, TELE_PORT
except:
    CMD_PORT, TELE_PORT = 1234, 1235

UDP_IP = "0.0.0.0"
ROBOT_TIMEOUT = 0.5

# Simulation State: [world_x, world_z, roll_rad, omega_rad, yaw_rad, forward_v]
sim_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
target_speed = 0.0
target_turn = 0.0
remote_roll = 0.0   # Updated: Roll mirroring from remote controller
last_robot_msg_time = 0

# --- UDP Logic ---
def udp_receiver():
    global target_speed, target_turn, last_robot_msg_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    try:
        sock.bind((UDP_IP, CMD_PORT))
        print(f"[*] 3D Sim: Listening for Command on port {CMD_PORT}")
    except Exception as e:
        print(f"[!] Bind Error: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8')
            parts = msg.split(',')
            if len(parts) >= 6:
                try:
                    # Corrected: speed flipped, turn restored to match remote
                    target_speed = -float(parts[0])
                    target_turn = float(parts[1])
                    
                    # Parse Quaternions for Lightsaber Roll
                    q0, q1, q2, q3 = map(float, parts[2:6])
                    # Calc roll from quat: atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
                    global remote_roll
                    remote_roll = math.atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2))
                    
                    last_robot_msg_time = time.time()
                except: pass
        except: pass
        time.sleep(0.01)

# --- Physics Controller ---
class VirtualBalancer:
    def __init__(self):
        self.gains = [8.0, 0.02, 1.2, 0.25] # Increased gains for more snap
        self.Kg = 0.015
        self.PosDes = 0.0
        
    def control(self, state, dt):
        roll_rad = state[2]
        omega_rad = state[3]
        v_fwd = state[5]
        
        # Target: Lean forward to move forward
        # remote target_speed is ~0.4 max. We map this to a target velocity.
        target_v = target_speed * 10.0 # Increased scaling: 0.4 -> 4.0 m/s
        target_lean_rad = -target_speed * 1.5 # Lean proportional to command
        
        # PID control
        torque = (
            20.0 * (roll_rad - target_lean_rad) +  # Stay at target lean
            2.5 * omega_rad +                      # Damping
            8.0 * (v_fwd - target_v)               # Velocity tracking
        )
        
        return max(min(torque, 20.0), -20.0)

balancer = VirtualBalancer()

def step_physics(dt_frame):
    sub_steps = 10
    dt = dt_frame / sub_steps
    for _ in range(sub_steps):
        torque = balancer.control(sim_state, dt)
        
        # Dynamics
        m, M, l, Ip, r = M_P, M_W, L, I_P, R
        theta, omega = sim_state[2], sim_state[3]
        
        sin_t, cos_t = math.sin(theta), math.cos(theta)
        M_total = M + m + I_W/(r**2)
        det = M_total * (Ip + m*l**2) - (m*l*cos_t)**2
        
        rhs1 = (torque / r) + m * l * (omega**2) * sin_t
        rhs2 = m * 9.81 * l * sin_t - torque
        
        accel = (rhs1 * (Ip + m*l**2) - rhs2 * m * l * cos_t) / det
        alpha = (M_total * rhs2 - m * l * cos_t * rhs1) / det

        sim_state[3] += alpha * dt
        sim_state[5] += accel * dt
        sim_state[2] += sim_state[3] * dt
        
        # 3D Positioning Logic - Standard Z-forward mapping
        sim_state[4] += target_turn * 5.0 * dt # Yaw Integration
        # At yaw=0, robot faces Z+. So vx=sin, vz=cos.
        vx = sim_state[5] * math.sin(sim_state[4])
        vz = sim_state[5] * math.cos(sim_state[4])
        sim_state[0] += vx * dt # World X
        sim_state[1] += vz * dt # World Z

    sim_state[3] *= 0.98 # Damping
    sim_state[5] *= 0.96

    if abs(sim_state[2]) > 1.2: # Reset if fallen
        sim_state[2] *= 0.5
        sim_state[5] *= 0.5

last_roll = 0.0

# --- Ursina Scene Setup ---
app = Ursina()

# --- Lightsaber Audio Assets ---
# Must be created AFTER app = Ursina()
try:
    hum_audio = Audio('lightsaber_hum.wav', loop=True, autoplay=True, volume=0.7)
    swing_audio = Audio('lightsaber_swing.wav', loop=False, autoplay=False, volume=1.0)
    print("[*] Lightsaber Audio Loaded Successfully")
except Exception as e:
    print(f"[!] Audio Load Error: {e}")
    hum_audio = None
    swing_audio = None
Sky(color=color.hsv(200, 0.1, 0.1)) # Very dark blue sky for depth

# Lighting is crucial for visibility
AmbientLight(color=color.rgba(120, 120, 120, 100))
DirectionalLight(y=3, z=-5, shadows=True, rotation=(45, -45, 0))

# Use a visible grid for the floor
floor = Entity(model='plane', scale=2000, texture='white_cube', texture_scale=(200,200), color=color.gray)

# Add landmark pylons so movement is VERY obvious
import random
for i in range(40):
    Entity(model='cube', color=color.hsv(random.randint(0,360), 0.7, 0.9), 
           position=(random.uniform(-100, 100), 1, random.uniform(-100, 100)),
           scale=(1, 2, 1))

# High Quality Robot Model (Enlarged)
robot_pivot = Entity()
chassis = Entity(parent=robot_pivot, model='cube', scale=(0.6, 0.9, 0.2), color=color.hsv(200, 0.8, 0.8), y=0.45)
bezel = Entity(parent=chassis, model='cube', scale=(1.05, 0.8, 1.1), color=color.black, z=0.01)
eye = Entity(parent=chassis, model='sphere', scale=(0.3, 0.1, 0.1), color=color.red, z=0.6, y=0.35)
eye_glow = Entity(parent=eye, model='sphere', scale=1.3, color=color.rgba(255, 0, 0, 80))

# Wheels - Using scaled spheres as discs. 
# Initial orientation: spheres are naturally aligned. We scale X thin to make them discs.
# Position them on the sides (X axis).
wheel_l = Entity(parent=robot_pivot, model='sphere', scale=(0.1, 0.55, 0.55), x=-0.4, y=0, color=color.black66)
wheel_r = Entity(parent=robot_pivot, model='sphere', scale=(0.1, 0.55, 0.55), x=0.4, y=0, color=color.black66)
rim_l = Entity(parent=wheel_l, model='sphere', scale=(1.1, 0.4, 0.4), color=color.cyan)
rim_r = Entity(parent=wheel_r, model='sphere', scale=(1.1, 0.4, 0.4), color=color.cyan)

# --- Lightsaber (Right Hand) ---
saber_pivot = Entity(parent=chassis, x=0.45, y=0.1) 
saber_handle = Entity(parent=saber_pivot, model='cube', scale=(0.1, 0.2, 0.1), color=color.dark_gray, y=0.1)
# unlit=True makes it glow regardless of scene light
saber_blade = Entity(parent=saber_handle, model='sphere', scale=(0.5, 6.5, 0.5), y=3.5, color=color.cyan, unlit=True)
saber_glow = Entity(parent=saber_blade, model='sphere', scale=1.3, color=color.rgba(0, 255, 255, 80), unlit=True)
# PointLight for actual surrounding illumination
saber_light = PointLight(parent=saber_blade, color=color.cyan, range=10)

# Ground Glow
glow = Entity(parent=robot_pivot, model='circle', rotation_x=90, y=0.01, scale=1.5, color=color.rgba(0, 150, 255, 30))

ui_panel = WindowPanel(
    title='Core Stats',
    content=(
        Text('Remote: Waiting...', color=color.yellow),
        Text('Angle: 0.0°'),
        Text('Spd: 0.00 m/s'),
    ),
    position=(-0.8, 0.45)
)
signal_dot = Entity(parent=camera.ui, model='circle', scale=0.02, position=(0.85, 0.45), color=color.gray)

def update():
    dt = time.dt
    step_physics(dt)
    
    # Signal indicator & Speed reset
    time_since_last_msg = time.time() - last_robot_msg_time
    if time_since_last_msg < ROBOT_TIMEOUT:
        signal_dot.color = color.green
        ui_panel.content[0].text = 'Remote: Connected'
        ui_panel.content[0].color = color.green
    else:
        signal_dot.color = color.gray
        ui_panel.content[0].text = 'Remote: Disconnected'
        ui_panel.content[0].color = color.red
        # Reset targets to zero if signal is lost
        global target_speed, target_turn
        target_speed = 0.0
        target_turn = 0.0
    
    # Update visuals
    robot_pivot.position = (sim_state[0], 0, sim_state[1])
    # Ursina rotation_y is degrees. Math yaw is radians CCW.
    robot_pivot.rotation_y = math.degrees(sim_state[4])
    chassis.rotation_x = -math.degrees(sim_state[2])
    
    # Update Lightsaber Pitch based on remote controller ROLL
    # Using rotation_x as it's the pivot for 'swinging' the sword
    saber_pivot.rotation_x = -math.degrees(remote_roll) 
    
    # Audio Logic: Play swing if roll velocity is high
    global last_roll
    roll_velocity = abs(remote_roll - last_roll) / dt
    if swing_audio and roll_velocity > 5.0: # Threshold for 'swinging'
        if not swing_audio.playing:
            swing_audio.play()
    last_roll = remote_roll
    
    # Wheel spinning visual - Rotating around local X axis (the Axel)
    spin_delta = math.degrees(sim_state[5]/R) * dt
    wheel_l.rotation_x += spin_delta
    wheel_r.rotation_x += spin_delta
    
    # Eye blinking / breathing
    eye_glow.scale = 1.2 + math.sin(time.time()*5)*0.2

    # Camera follow
    camera.position = lerp(camera.position, robot_pivot.position + robot_pivot.back * 5 + Vec3(0, 2.5, 0), dt * 3)
    camera.look_at(robot_pivot.position + Vec3(0, 0.5, 0))
    
    # Update UI
    ui_panel.content[1].text = f"Angle: {math.degrees(sim_state[2]):.1f}°"
    ui_panel.content[2].text = f"Spd: {sim_state[5]:.2f} m/s"

threading.Thread(target=udp_receiver, daemon=True).start()
app.run()
