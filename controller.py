# controller.py

import math

# Simple local-yaw PID controller. Position error in global frame
# is rotated into local yaw frame for X/Y. Z is handled directly in world frame.

# Store integrals and previous errors to implement PID over time
pid_data = {
    'xy': {'int_x': 0.0, 'int_y': 0.0, 'prev_x': 0.0, 'prev_y': 0.0},
    'z':  {'int_z': 0.0, 'prev_z': 0.0},
    'yaw': {'int_yaw': 0.0, 'prev_yaw': 0.0}
}

# PID gains - K_u is measured to be 1.2 and T_u to be 2.4s
Kp_xy, Ki_xy, Kd_xy = 0.5, 0.02, 0.05
Kp_z,  Ki_z,  Kd_z  = 0.5, 0.02, 0.05
Kp_yaw, Ki_yaw, Kd_yaw = 0.5, 0.02, 0.05

# Command limits to prevent extreme velocity or yaw rate
MAX_VEL_XY   = 2.0
MAX_VEL_Z    = 1.4
MAX_YAW_RATE = 3.0

def clamp(val, mn, mx):
    return max(mn, min(val, mx))

def angle_difference(tgt, cur):
    # Signed difference, ensuring result is in (-pi, pi)
    diff = (tgt - cur + math.pi) % (2 * math.pi) - math.pi
    return diff

def controller(state, target_pos, dt):
    """
    Single-layer PID controller that outputs local velocity commands.
    
    state: [x, y, z, roll, pitch, yaw]
    target_pos: (tx, ty, tz, tyaw)
    dt: time step (seconds)
    Returns: (vx_local, vy_local, vz_local, yaw_rate)
    """
    if dt <= 0:
        return (0.0, 0.0, 0.0, 0.0)

    # Current position/orientation
    x, y, z, _, _, yaw = state
    # Desired position/orientation
    tx, ty, tz, tyaw = target_pos

    # --- Position errors in world frame ---
    exw = tx - x
    eyw = ty - y
    ezw = tz - z

    # --- Rotate X/Y errors into drone's local-yaw frame ---
    cos_neg_yaw = math.cos(-yaw)
    sin_neg_yaw = math.sin(-yaw)
    exl = exw * cos_neg_yaw - eyw * sin_neg_yaw
    eyl = exw * sin_neg_yaw + eyw * cos_neg_yaw

    # --- X/Y local PID ---
    pid_data['xy']['int_x'] += exl * dt
    pid_data['xy']['int_y'] += eyl * dt
    dx = (exl - pid_data['xy']['prev_x']) / dt
    dy = (eyl - pid_data['xy']['prev_y']) / dt
    pid_data['xy']['prev_x'] = exl
    pid_data['xy']['prev_y'] = eyl

    vx = Kp_xy * exl + Ki_xy * pid_data['xy']['int_x'] + Kd_xy * dx
    vy = Kp_xy * eyl + Ki_xy * pid_data['xy']['int_y'] + Kd_xy * dy

    # --- Z PID (stay in world frame) ---
    pid_data['z']['int_z'] += ezw * dt
    dz = (ezw - pid_data['z']['prev_z']) / dt
    pid_data['z']['prev_z'] = ezw
    vz = Kp_z * ezw + Ki_z * pid_data['z']['int_z'] + Kd_z * dz

    # --- Yaw PID (angle -> yaw rate) ---
    eyaw = angle_difference(tyaw, yaw)
    pid_data['yaw']['int_yaw'] += eyaw * dt
    dyaw = (eyaw - pid_data['yaw']['prev_yaw']) / dt
    pid_data['yaw']['prev_yaw'] = eyaw
    vyaw = Kp_yaw * eyaw + Ki_yaw * pid_data['yaw']['int_yaw'] + Kd_yaw * dyaw

    # --- Clamp velocity and yaw rate ---
    vx   = clamp(vx,   -MAX_VEL_XY,   MAX_VEL_XY)
    vy   = clamp(vy,   -MAX_VEL_XY,   MAX_VEL_XY)
    vz   = clamp(vz,   -MAX_VEL_Z,    MAX_VEL_Z)
    vyaw = clamp(vyaw, -MAX_YAW_RATE, MAX_YAW_RATE)

    return (vx, vy, vz, vyaw)
