import pygame
import math
import socket
import threading
import time

# --- Configuration & Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 400
FPS = 60

# Physics constants for a ~0.5kg robot
G = 9.81            # Gravity
M_P = 0.5           # Body mass (kg)
M_W = 0.1           # Wheel mass (total for two wheels, kg)
L = 0.12            # Distance from wheel axis to body center of mass (m)
R = 0.033           # Wheel radius (m)
I_P = (M_P * (L**2)) / 3.0 # Body moment of inertia
I_W = 0.5 * M_W * (R**2)   # Wheel moment of inertia

# Simulation state
# x: position (m), v: velocity (m/s), theta: angle (rad), omega: ang_vel (rad/s)
state = [0.0, 0.0, 0.0, 0.0]

target_speed = 0.0
target_turn = 0.0

from config import CMD_PORT, TELE_PORT
UDP_IP = "0.0.0.0" 
UDP_PORT = CMD_PORT # Listen for RC Commands instead of just telemetry
remote_addr = None 
last_robot_msg_time = 0
ROBOT_TIMEOUT = 0.5 # 500ms timeout to switch between Real and Sim
is_real_robot = False
signal_blink = 0 # Counter for blinking indicator

# --- UDP Receiver Thread ---
def udp_receiver():
    global target_speed, target_turn, remote_addr
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((UDP_IP, UDP_PORT))
        print(f"UDP Receiver listening on {UDP_IP}:{UDP_PORT}")
    except Exception as e:
        print(f"Failed to bind UDP port: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8')
            parts = msg.split(',')
            
            # V2 Remote Format: speed,turn,q0,q1,q2,q3,gx,gy,gz
            if len(parts) >= 6:
                global target_speed, target_turn
                target_speed = float(parts[0])
                target_turn = float(parts[1])
                # In simulation mode, we don't necessarily need to update the state from remote quats
                # but we could. For now, let's just use it to show connectivity.
                last_robot_msg_time = time.time()
                is_real_robot = True
                signal_blink = 5
            
            # Robot Telemetry (Fallback/Monitoring): roll,vL,vR
            elif len(parts) == 3:
                last_robot_msg_time = time.time()
                is_real_robot = True
                signal_blink = 5
                state[2] = math.radians(float(parts[0]))
        except Exception:
            pass

def telemetry_sender():
    """Sends simulation state back to the remote controller."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        if remote_addr:
            try:
                # Format: "roll_deg,v_left_deg_s,v_right_deg_s"
                roll_deg = math.degrees(state[2])
                v_forward_deg_s = math.degrees(state[1] / R)
                
                # Estimate differential speeds for display based on turn command
                # target_turn range is roughly +/- 0.2 from the remote
                turn_bias = target_turn * 300.0 
                v_left = v_forward_deg_s - turn_bias
                v_right = v_forward_deg_s + turn_bias

                msg = f"{roll_deg:.2f},{v_left:.1f},{v_right:.1f}"
                sock.sendto(msg.encode('utf-8'), remote_addr)
            except Exception:
                pass # Suppress network errors (e.g. [WinError 10051])
        time.sleep(0.05) # 20Hz update

# --- Controller State (Matching User's Balancer class) ---
class VirtualBalancer:
    def __init__(self):
        # Matching user's gains and Kg
        self.gains = [5.0, 0.02, 0.7, 0.15]
        self.Kg = 0.01
        self.PosDes = 0.0
        
    def control(self, state_rad, dt):
        x_world, v_world, theta_rad, omega_rad = state_rad
        
        # 1. Convert units to Degrees (matching physical robot)
        roll_deg = math.degrees(theta_rad)
        rolldot_deg = math.degrees(omega_rad)
        
        # Wheel rotation in degrees
        # world_pos = wheel_angle_rad * R -> wheel_angle_deg = (pos / R) * (180/pi)
        wheel_pos_deg = math.degrees(x_world / R)
        wheel_vel_deg = math.degrees(v_world / R)
        
        # 2. Target inputs from UDP (scaled exactly like user's code)
        linearVel_cmd = target_speed * 100.0 # User's scaling: float(msg[0]) * 100
        
        # 3. Position Desired Integration
        self.PosDes += linearVel_cmd * dt
        
        # 4. State Vector Calculation (Matching user's x0-x3)
        x0 = roll_deg
        x1 = wheel_pos_deg - self.PosDes
        x2 = rolldot_deg
        x3 = wheel_vel_deg - linearVel_cmd
        
        # 5. Control Law
        u1 = self.gains[0]*x0 + self.gains[1]*x1 + self.gains[2]*x2 + self.gains[3]*x3
        u1 *= self.Kg
        
        return u1

balancer = VirtualBalancer()

# --- Physics Engine ---
def step_physics(dt_frame):
    global state
    sub_steps = 10
    dt = dt_frame / sub_steps
    
    for _ in range(sub_steps):
        # Get control output from our virtual balancer
        torque = balancer.control(state, dt)
        
        # Clamp torque to realistic limits (L298 / Battery range)
        torque = max(min(torque, 15.0), -15.0)

        # --- Dynamics Calculation (Simplified for stability) ---
        m, M, l, g, Ip, r = M_P, M_W, L, G, I_P, R
        x, v, theta, omega = state
        
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        
        # Determinant and Equations
        M_total = M + m + I_W/(r**2)
        Inertia_body = Ip + m*(l**2)
        det = M_total * Inertia_body - (m * l * cos_t)**2
        
        rhs1 = (torque / r) + m * l * (omega**2) * sin_t
        rhs2 = m * g * l * sin_t - torque
        
        accel = (rhs1 * Inertia_body - rhs2 * m * l * cos_t) / det
        alpha = (M_total * rhs2 - m * l * cos_t * rhs1) / det

        # Integration
        state[3] += alpha * dt
        state[1] += accel * dt
        state[2] += state[3] * dt
        state[0] += state[1] * dt

    # Damping
    state[3] *= 0.98 # Increased damping for 0.5kg bot
    state[1] *= 0.95

    # Safety: Reset if robot falls (keeps simulation clean)
    if abs(state[2]) > math.radians(70):
        state[1] = 0
        state[3] = 0
        state[2] *= 0.8
        if abs(state[2]) < 0.05: # Snap to vertical
             state[2] = 0
             state[0] = 0 # Center when reset

# --- Rendering Helpers ---
def draw_gradient_rect(screen, color1, color2, rect):
    fill = pygame.Surface((rect.width, rect.height))
    for y in range(rect.height):
        r = color1[0] + (color2[0] - color1[0]) * y / rect.height
        g = color1[1] + (color2[1] - color1[1]) * y / rect.height
        b = color1[2] + (color2[2] - color1[2]) * y / rect.height
        pygame.draw.line(fill, (int(r), int(g), int(b)), (0, y), (rect.width, y))
    screen.blit(fill, rect)

def draw(screen, font):
    # Background
    draw_gradient_rect(screen, (10, 10, 20), (30, 30, 50), screen.get_rect())
    
    # Ground & Grid
    grid_size = 50
    offset_x = (state[0] * 400) % grid_size
    for x in range(0, SCREEN_WIDTH + grid_size, grid_size):
        pygame.draw.line(screen, (40, 40, 70), (x - offset_x, 0), (x - offset_x, SCREEN_HEIGHT), 1)

    pygame.draw.line(screen, (0, 180, 255), (0, 300), (SCREEN_WIDTH, 300), 4)

    x, v, theta, omega = state
    scale = 400
    screen_cx = SCREEN_WIDTH // 2
    wheel_y = 300 - R * scale
    
    # Wheel
    wheel_color = (200, 200, 205)
    pygame.draw.circle(screen, wheel_color, (screen_cx, int(wheel_y)), int(R * scale), 4)
    wheel_rot = -x / R
    spoke_end = (screen_cx + math.sin(wheel_rot) * R * scale, wheel_y + math.cos(wheel_rot) * R * scale)
    pygame.draw.line(screen, wheel_color, (screen_cx, wheel_y), spoke_end, 3)

    # Body
    body_len = L * 2.5 * scale
    body_end = (screen_cx + math.sin(theta) * body_len, wheel_y - math.cos(theta) * body_len)
    pygame.draw.line(screen, (0, 120, 255), (screen_cx, wheel_y), body_end, 12)
    pygame.draw.line(screen, (150, 220, 255), (screen_cx, wheel_y), body_end, 4)
    pygame.draw.circle(screen, (255, 60, 60), (int(body_end[0]), int(body_end[1])), 18)

    # Dashboard
    dash_rect = pygame.Rect(20, 20, 280, 150)
    dash_surf = pygame.Surface((dash_rect.width, dash_rect.height), pygame.SRCALPHA)
    pygame.draw.rect(dash_surf, (255, 255, 255, 40), dash_surf.get_rect(), border_radius=15)
    screen.blit(dash_surf, dash_rect)

    # UI Text
    titles = [
        f"ROLL: {math.degrees(theta):+.2f}°",
        f"X0(Err): {math.degrees(theta):+.2f}",
        f"X1(Pos): {math.degrees(x/R) - balancer.PosDes:+.1f}",
        f"TGT_SPD: {target_speed*100:+.1f} deg/s",
        f"TRN: {target_turn*150:+.1f}"
    ]
    for i, t in enumerate(titles):
        col = (0, 255, 180) if "TGT" in t else (255, 255, 255)
        txt = font.render(t, True, col)
        screen.blit(txt, (40, 40 + i * 26))

# --- Main ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("D-Twin: Robot State-Space Sim")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 20)

    # Start UDP receiver thread
    udp_thread = threading.Thread(target=udp_receiver, daemon=True)
    udp_thread.start()

    # Start Telemetry sender thread
    tele_thread = threading.Thread(target=telemetry_sender, daemon=True)
    tele_thread.start()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        # Mode Selection Logic
        if time.time() - last_robot_msg_time > ROBOT_TIMEOUT:
            is_real_robot = False
            step_physics(1/FPS)
        else:
            is_real_robot = True
            # In real robot mode, we skip physics and let udp_receiver update 'state'

        draw(screen, font)
        
        # Overlay mode status
        mode_text = "MODE: [ REAL ROBOT ]" if is_real_robot else "MODE: [ SIMULATION ]"
        col = (255, 100, 100) if is_real_robot else (100, 255, 100)
        
        # Signal Indicator
        if is_real_robot:
            global signal_blink
            if signal_blink > 0:
                pygame.draw.circle(screen, (0, 255, 0), (SCREEN_WIDTH - 270, 32), 6)
                signal_blink -= 1
        
        txt = font.render(mode_text, True, col)
        screen.blit(txt, (SCREEN_WIDTH - 250, 20))

        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()

if __name__ == "__main__":
    main()
