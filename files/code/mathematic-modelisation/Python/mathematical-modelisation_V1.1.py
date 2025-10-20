"""
MATHEMATICAL MODELISATION OF A 4-WHEEL MECANUM (45°) HOLONOMIC ROBOT
Simple 2D simulation using pygame.

Controls (keyboard):
    arrows / ZQSD  : translate requested vx, vy
    A / E          : rotate requested angular velocity (w)
    space          : stop (zero requested velocities)
    r              : reset pose
    +/- (numpad)   : increase/decrease wheel radius scale (optional)
    ESC or window close : quit

All code comments are in English.
"""
import math
import sys
import time

import numpy as np
import pygame

# ---------------------------
# Configuration / parameters
# ---------------------------

# Robot geometry & wheel parameters (use variables, easy to tune)
WHEEL_RADIUS = 1.0       # wheel radius (units)
LX = 0.20                # half-distance in x between left and right wheel axes (meters)
LY = 0.30                # half-distance in y between front and back wheel axes (meters)
# Note: original script used lx and ly as full distances; here we assume LX and LY are half-distances.
# If you prefer full distances, set lx = full/2 etc.

# Simulation settings
SCREEN_SIZE = (1000, 800)
WORLD_TO_PIX = 100.0     # scale: 1 meter -> WORLD_TO_PIX pixels
ROBOT_WIDTH = 0.4        # robot width in meters (x direction)
ROBOT_LENGTH = 0.6       # robot length in meters (y direction)
FPS = 60
MAX_REQUEST_V = 2.0      # max linear requested speed (m/s)
MAX_REQUEST_OMEGA = 3.0  # max angular requested speed (rad/s)

# Colors
COLOR_BG = (30, 30, 30)
COLOR_ROBOT = (200, 200, 200)
COLOR_WHEEL = (120, 120, 255)
COLOR_TEXT = (220, 220, 220)
COLOR_VECTOR = (255, 80, 80)
COLOR_AXIS_X = (255, 0, 0)    # Red for X axis
COLOR_AXIS_Y = (0, 255, 0)    # Green for Y axis
COLOR_WHEEL_FORWARD = (0, 255, 0)    # Green for forward rotation
COLOR_WHEEL_BACKWARD = (255, 0, 0)    # Red for backward rotation
# Control modes
MODE_MANUAL = 0
MODE_AUTO = 1
# PID control gains for autonomous mode
KP_POS = 1.0    # Position proportional gain
KP_ANGLE = 2.0  # Angle proportional gain
# Target point visualization
TARGET_RADIUS = 8
COLOR_TARGET = (255, 160, 0)
AXIS_LENGTH = 50             # Length in pixels (not meters)
AXIS_THICKNESS = 2
AXIS_MARGIN = 50             # Margin from screen borders
WHEEL_ARROW_SIZE = 25                # Base size for wheel rotation indicators

# ---------------------------
# Kinematics functions
# ---------------------------

def forward_kinematics(vx, vy, w, wheel_radius=WHEEL_RADIUS, lx=LX, ly=LY):
    """
    Compute wheel angular velocities (w1..w4) given desired body velocities.
    Using convention:
        w1 = front-left
        w2 = front-right
        w3 = back-right
        w4 = back-left
    Equations adapted from the provided model.
    """
    # Using (lx + ly) as distance factor where lx and ly are half-distances; if using full distances adjust accordingly
    r = wheel_radius
    L = lx + ly
    w1 = (1.0 / r) * (vx - vy - L * w)
    w2 = (1.0 / r) * (vx + vy + L * w)
    w3 = (1.0 / r) * (vx + vy - L * w)
    w4 = (1.0 / r) * (vx - vy + L * w)
    return np.array([w1, w2, w3, w4])


def inverse_kinematics(wheels, wheel_radius=WHEEL_RADIUS, lx=LX, ly=LY):
    """
    Compute body velocities (vxt, vyt, wt) from wheel angular velocities.
    Returns (vxt, vyt, wt)
    """
    r = wheel_radius
    L = lx + ly
    w1, w2, w3, w4 = wheels
    vxt = (w1 + w2 + w3 + w4) * (r / 4.0)
    vyt = (-w1 + w2 + w3 - w4) * (r / 4.0)
    wt = (-w1 + w2 - w3 + w4) * (r / (4.0 * L))
    return vxt, vyt, wt


def body_to_world(vx_body, vy_body, omega, theta):
    """
    Transform velocities expressed in robot (body) frame to world (global) frame.
    """
    vx_world = vx_body * math.cos(theta) - vy_body * math.sin(theta)
    vy_world = vx_body * math.sin(theta) + vy_body * math.cos(theta)
    return vx_world, vy_world, omega


# ---------------------------
# Pygame rendering helpers
# ---------------------------

def meters_to_pixels(x_m, y_m, screen_center):
    """
    Convert world meters to pixel coordinates, with (0,0) at screen_center.
    y is inverted for screen coordinates.
    """
    px = screen_center[0] + x_m * WORLD_TO_PIX
    py = screen_center[1] - y_m * WORLD_TO_PIX
    return int(px), int(py)


def draw_wheel_rotation_indicator(surface, center, angular_velocity, theta, max_speed=MAX_REQUEST_V):
    """
    Draw an arrow indicating wheel rotation direction and speed.
    The arrow length is proportional to the wheel speed.
    """
    if abs(angular_velocity) < 0.01:  # Skip if wheel is practically not moving
        return
        
    # Normalize velocity for arrow length (0 to 1)
    normalized_speed = min(abs(angular_velocity), max_speed) / max_speed
    arrow_length = WHEEL_ARROW_SIZE * normalized_speed
    
    # Calculate arrow end point based on direction
    if angular_velocity > 0:  # One direction
        angle = theta + math.pi/2  # Perpendicular to wheel orientation
    else:  # Other direction
        angle = theta - math.pi/2
        
    # Calculate arrow end points
    end_x = center[0] + arrow_length * math.cos(angle)
    end_y = center[1] - arrow_length * math.sin(angle)  # Minus because pygame y is inverted
    
    # Draw arrow line
    color = COLOR_WHEEL_FORWARD if angular_velocity > 0 else COLOR_WHEEL_BACKWARD
    pygame.draw.line(surface, color, center, (end_x, end_y), 2)
    
    # Draw arrow head
    head_length = min(8, arrow_length/3)  # Arrow head size proportional to length but with max
    if arrow_length > head_length * 2:  # Only draw head if arrow is long enough
        angle_head = math.pi/6  # 30 degrees
        if angular_velocity > 0:
            angle1 = angle + angle_head
            angle2 = angle - angle_head
        else:
            angle1 = angle + math.pi + angle_head
            angle2 = angle + math.pi - angle_head
            
        head1_x = end_x - head_length * math.cos(angle1)
        head1_y = end_y + head_length * math.sin(angle1)
        head2_x = end_x - head_length * math.cos(angle2)
        head2_y = end_y + head_length * math.sin(angle2)
        
        pygame.draw.line(surface, color, (end_x, end_y), (head1_x, head1_y), 2)
        pygame.draw.line(surface, color, (end_x, end_y), (head2_x, head2_y), 2)

def draw_robot(surface, pose, wheels_angular, requested, params):
    """
    Draw the robot rectangle, heading, and small wheels. Also draw a velocity vector.
    pose: (x, y, theta) in meters and radians
    wheels_angular: array-like [w1..w4] (rad/s)
    requested: (vx_req, vy_req, w_req)
    params: dict with robot dims
    """
    # Initialize font at the start of the function
    font = pygame.font.get_default_font()
    f = pygame.font.Font(font, 16)

    x, y, theta = pose
    screen_center = (surface.get_width() // 2, surface.get_height() // 2)
    # robot rectangle corners in local frame
    L = params['length']
    W = params['width']
    corners = [
        ( L/2,  W/2),
        ( L/2, -W/2),
        (-L/2, -W/2),
        (-L/2,  W/2),
    ]
    # rotate + translate corners to world
    pts = []
    for cx, cy in corners:
        wx = x + (cx * math.cos(theta) - cy * math.sin(theta))
        wy = y + (cx * math.sin(theta) + cy * math.cos(theta))
        pts.append(meters_to_pixels(wx, wy, screen_center))
    pygame.draw.polygon(surface, COLOR_ROBOT, pts)
    # heading line (front middle)
    front_mid_x = x + (L/2) * math.cos(theta)
    front_mid_y = y + (L/2) * math.sin(theta)
    start = meters_to_pixels(x, y, screen_center)
    end = meters_to_pixels(front_mid_x, front_mid_y, screen_center)
    pygame.draw.line(surface, COLOR_VECTOR, start, end, 3)
    # draw wheels as small rectangles on each side (approx)
    # wheel positions in local frame (front-left, front-right, back-right, back-left)
    wheel_positions = [
        ( L/2,  W/2),  # Front-left
        ( L/2, -W/2),  # Front-right
        (-L/2, -W/2),  # Back-right
        (-L/2,  W/2),  # Back-left
    ]
    wheel_size_px = (8, 18)
    wheel_names = ['FL', 'FR', 'BR', 'BL']
    
    for (wx_loc, wy_loc), w_ang, name in zip(wheel_positions, wheels_angular, wheel_names):
        # wheel center in world
        wx_world = x + (wx_loc * math.cos(theta) - wy_loc * math.sin(theta))
        wy_world = y + (wx_loc * math.sin(theta) + wy_loc * math.cos(theta))
        rect_center = meters_to_pixels(wx_world, wy_world, screen_center)
        
        # Draw wheel rectangle
        rect = pygame.Rect(0, 0, wheel_size_px[0], wheel_size_px[1])
        rect.center = rect_center
        pygame.draw.rect(surface, COLOR_WHEEL, rect)
        
        # Draw rotation indicator arrow
        draw_wheel_rotation_indicator(surface, rect_center, w_ang, theta)
        
    # draw requested velocity vector (in robot frame) transformed to world for visualization
    vx_req, vy_req, w_req = requested
    vx_world, vy_world, _ = body_to_world(vx_req, vy_req, 0.0, theta)
    vec_end = meters_to_pixels(x + vx_world * 0.5, y + vy_world * 0.5, screen_center)
    pygame.draw.line(surface, (80,255,120), start, vec_end, 3)
    # draw small text info
    font = pygame.font.get_default_font()
    f = pygame.font.Font(font, 16)
    info_lines = [
        f"pose x:{x:.2f} m  y:{y:.2f} m  θ:{math.degrees(theta)%360:.1f}°",
        f"requested vx:{vx_req:.2f} m/s  vy:{vy_req:.2f} m/s  ω:{w_req:.2f} rad/s",
        f"FL:{wheels_angular[0]:6.2f} rad/s  FR:{wheels_angular[1]:6.2f} rad/s",
        f"BL:{wheels_angular[3]:6.2f} rad/s  BR:{wheels_angular[2]:6.2f} rad/s"
    ]
    for i, line in enumerate(info_lines):
        surf = f.render(line, True, COLOR_TEXT)
        surface.blit(surf, (10, 10 + 18 * i))

def draw_coordinate_system(surface):
    """
    Draw X-Y coordinate system in the bottom-left corner of the screen.
    X axis in red, Y axis in green.
    """
    # Position in bottom-left corner with margin
    origin_x = AXIS_MARGIN
    origin_y = surface.get_height() - AXIS_MARGIN
    origin = (origin_x, origin_y)
    
    # Draw X axis (red)
    end_x = (origin_x + AXIS_LENGTH, origin_y)
    pygame.draw.line(surface, COLOR_AXIS_X, origin, end_x, AXIS_THICKNESS)
    
    # Draw Y axis (green)
    end_y = (origin_x, origin_y - AXIS_LENGTH)
    pygame.draw.line(surface, COLOR_AXIS_Y, origin, end_y, AXIS_THICKNESS)
    
    # Draw axis labels
    font = pygame.font.get_default_font()
    f = pygame.font.Font(font, 16)
    
    # X label
    x_label = f.render("X", True, COLOR_AXIS_X)
    surface.blit(x_label, (end_x[0] + 5, end_x[1] - 20))
    
    # Y label
    y_label = f.render("Y", True, COLOR_AXIS_Y)
    surface.blit(y_label, (end_y[0] - 20, end_y[1] - 5))


# ---------------------------
# Control functions
# ---------------------------

def compute_control_to_target(current_pose, target_point):
    """
    Compute control commands (vx, vy, w) to reach target point.
    Uses a simple proportional control law.
    """
    x, y, theta = current_pose
    target_x, target_y = target_point
    
    # Compute error in global frame
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx**2 + dy**2)
    
    # Convert to robot frame
    dx_robot = dx * math.cos(theta) + dy * math.sin(theta)
    dy_robot = -dx * math.sin(theta) + dy * math.cos(theta)
    
    # Simple proportional control
    vx = KP_POS * dx_robot
    vy = KP_POS * dy_robot
    
    # Angle to target (simple proportional on angle error)
    target_angle = math.atan2(dy, dx)
    angle_error = (target_angle - theta + math.pi) % (2*math.pi) - math.pi
    w = KP_ANGLE * angle_error
    
    # Clamp values
    vx = clamp(vx, -MAX_REQUEST_V, MAX_REQUEST_V)
    vy = clamp(vy, -MAX_REQUEST_V, MAX_REQUEST_V)
    w = clamp(w, -MAX_REQUEST_OMEGA, MAX_REQUEST_OMEGA)
    
    return vx, vy, w, distance


# ---------------------------
# Main simulation
# ---------------------------

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

# Graceful exit helper: quits pygame and exits the Python process.
def clean_exit():
    """Quit pygame and exit process cleanly."""
    pygame.quit()
    sys.exit()

def main():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    pygame.display.set_caption("Holonomic 4-wheel Mecanum Simulation")
    clock = pygame.time.Clock()

    # Add font initialization at the start of main()
    font = pygame.font.get_default_font()
    f = pygame.font.Font(font, 14)  # Define 'f' as the font object we'll use

    # initial robot pose (world frame)
    pose = [0.0, 0.0, 0.0]  # x(m), y(m), theta(rad)
    requested = [0.0, 0.0, 0.0]  # vx, vy, omega (in robot frame)
    
    # Add control mode variables
    control_mode = MODE_MANUAL
    target_point = None

    params = {'width': ROBOT_WIDTH, 'length': ROBOT_LENGTH}

    running = True
    paused = False
    last_time = time.time()

    # Add new variables for control mode and target
    control_mode = MODE_MANUAL
    target_point = None
    
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                clean_exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    clean_exit()
                elif event.key == pygame.K_SPACE:
                    requested = [0.0, 0.0, 0.0]
                elif event.key == pygame.K_r:
                    pose = [0.0, 0.0, 0.0]
                    requested = [0.0, 0.0, 0.0]
                elif event.key == pygame.K_m:  # Toggle mode
                    control_mode = MODE_MANUAL if control_mode == MODE_AUTO else MODE_AUTO
                    requested = [0.0, 0.0, 0.0]  # Reset commands when switching modes
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    # Convert mouse position to world coordinates
                    mx, my = event.pos
                    screen_center = (screen.get_width() // 2, screen.get_height() // 2)
                    target_point = ((mx - screen_center[0]) / WORLD_TO_PIX,
                                  (screen_center[1] - my) / WORLD_TO_PIX)
        
        if control_mode == MODE_MANUAL:
            # Existing manual control code
            keys = pygame.key.get_pressed()
            inc_lin = 1.5 * dt
            inc_ang = 2.0 * dt
            if keys[pygame.K_UP] or keys[pygame.K_z]:
                requested[0] = clamp(requested[0] + inc_lin, -MAX_REQUEST_V, MAX_REQUEST_V)
            if keys[pygame.K_DOWN] or keys[pygame.K_s]:
                requested[0] = clamp(requested[0] - inc_lin, -MAX_REQUEST_V, MAX_REQUEST_V)
            if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                requested[1] = clamp(requested[1] - inc_lin, -MAX_REQUEST_V, MAX_REQUEST_V)  # right is negative vy in this convention
            if keys[pygame.K_LEFT] or keys[pygame.K_q]:
                requested[1] = clamp(requested[1] + inc_lin, -MAX_REQUEST_V, MAX_REQUEST_V)
            if keys[pygame.K_a]:
                requested[2] = clamp(requested[2] + inc_ang, -MAX_REQUEST_OMEGA, MAX_REQUEST_OMEGA)
            if keys[pygame.K_e]:
                requested[2] = clamp(requested[2] - inc_ang, -MAX_REQUEST_OMEGA, MAX_REQUEST_OMEGA)

        else:  # MODE_AUTO
            if target_point is not None:
                # Compute control commands to reach target
                vx, vy, w, distance = compute_control_to_target(pose, target_point)
                requested = [vx, vy, w]
                
                # Optional: clear target when reached
                if distance < 0.1:  # 10cm threshold
                    target_point = None
                    requested = [0.0, 0.0, 0.0]
            else:
                requested = [0.0, 0.0, 0.0]
        
        # Compute wheel angular velocities from requested body velocities
        wheels = forward_kinematics(requested[0], requested[1], requested[2],
                                    wheel_radius=WHEEL_RADIUS, lx=LX, ly=LY)
        # Recompute actual body velocities via inverse kinematics (models actuator -> body)
        vxb, vyb, wb = inverse_kinematics(wheels, wheel_radius=WHEEL_RADIUS, lx=LX, ly=LY)

        # Integrate pose: convert body velocities to world frame
        vx_world, vy_world, omega_world = body_to_world(vxb, vyb, wb, pose[2])
        # Explicit Euler integration
        pose[0] += vx_world * dt
        pose[1] += vy_world * dt
        pose[2] += omega_world * dt

        # Rendering
        screen.fill(COLOR_BG)
        draw_robot(screen, pose, wheels, requested, params)

        # Additional drawing for auto mode
        if target_point is not None:
            screen_center = (screen.get_width() // 2, screen.get_height() // 2)
            target_px = meters_to_pixels(target_point[0], target_point[1], screen_center)
            pygame.draw.circle(screen, COLOR_TARGET, target_px, TARGET_RADIUS)
        
        # Update mode info in text
        mode_text = f"Mode: {'AUTO' if control_mode == MODE_AUTO else 'MANUAL'}"
        mode_surf = f.render(mode_text, True, COLOR_TEXT)
        screen.blit(mode_surf, (10, SCREEN_SIZE[1] - 46))
        
        # Update control hints
        footer = f.render("Controls: M toggle mode | Click to set target (AUTO) | arrows/ZQSD move (MANUAL) | A/E rotate | SPACE stop | R reset | ESC quit", 
                         True, COLOR_TEXT)
        screen.blit(footer, (10, SCREEN_SIZE[1] - 26))
        
        # Draw the coordinate system (optional)
        draw_coordinate_system(screen)
        
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
