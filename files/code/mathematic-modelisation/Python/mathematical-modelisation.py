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
SCREEN_SIZE = (900, 700)
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
# Pygame rendering helperss
# ---------------------------

def meters_to_pixels(x_m, y_m, screen_center):
    """
    Convert world meters to pixel coordinates, with (0,0) at screen_center.
    y is inverted for screen coordinates.
    """
    px = screen_center[0] + x_m * WORLD_TO_PIX
    py = screen_center[1] - y_m * WORLD_TO_PIX
    return int(px), int(py)


def draw_robot(surface, pose, wheels_angular, requested, params):
    """
    Draw the robot rectangle, heading, and small wheels. Also draw a velocity vector.
    pose: (x, y, theta) in meters and radians
    wheels_angular: array-like [w1..w4] (rad/s)
    requested: (vx_req, vy_req, w_req)
    params: dict with robot dims
    """
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
        ( L/2,  W/2),
        ( L/2, -W/2),
        (-L/2, -W/2),
        (-L/2,  W/2),
    ]
    wheel_size_px = (8, 18)
    for (wx_loc, wy_loc), w_ang in zip(wheel_positions, wheels_angular):
        # wheel center in world
        wx_world = x + (wx_loc * math.cos(theta) - wy_loc * math.sin(theta))
        wy_world = y + (wx_loc * math.sin(theta) + wy_loc * math.cos(theta))
        rect_center = meters_to_pixels(wx_world, wy_world, screen_center)
        rect = pygame.Rect(0, 0, wheel_size_px[0], wheel_size_px[1])
        rect.center = rect_center
        pygame.draw.rect(surface, COLOR_WHEEL, rect)
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
        f"wheel w (rad/s): {', '.join(f'{wi:.2f}' for wi in wheels_angular)}",
    ]
    for i, line in enumerate(info_lines):
        surf = f.render(line, True, COLOR_TEXT)
        surface.blit(surf, (10, 10 + 18 * i))


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

    # initial robot pose (world frame)
    pose = [0.0, 0.0, 0.0]  # x(m), y(m), theta(rad)
    requested = [0.0, 0.0, 0.0]  # vx, vy, omega (in robot frame)

    params = {'width': ROBOT_WIDTH, 'length': ROBOT_LENGTH}

    running = True
    paused = False
    last_time = time.time()

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
                # optional wheel radius scale not implemented interactive; keep simple
        # continuous key state for smooth control
        keys = pygame.key.get_pressed()
        # linear increments per second
        inc_lin = 1.5 * dt
        inc_ang = 2.0 * dt
        # English-friendly keys: arrow keys and WASD/ZQSD
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

        # Helper footnote
        font = pygame.font.get_default_font()
        f = pygame.font.Font(font, 14)
        footer = f.render("Controls: arrows/ZQSD translate | A/E rotate | SPACE stop | R reset | ESC quit", True, COLOR_TEXT)
        screen.blit(footer, (10, SCREEN_SIZE[1] - 26))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
