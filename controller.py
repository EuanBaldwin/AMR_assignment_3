# controller.py

import math

# PID integrator and previous-error storage
pid_data = {
    'xy': {'int_x': 0.0, 'int_y': 0.0, 'prev_x': 0.0, 'prev_y': 0.0},
    'z':  {'int_z': 0.0, 'prev_z': 0.0},
    'yaw': {'int_yaw': 0.0, 'prev_yaw': 0.0},
}

# Position→Velocity gains for X/Y in local-yaw frame, Z in global frame
Kp_xy, Ki_xy, Kd_xy = 1.0, 0.0, 0.0
Kp_z,  Ki_z,  Kd_z  = 1.0, 0.0, 0.0

# Yaw→YawRate gains
Kp_yaw, Ki_yaw, Kd_yaw = 1.0, 0.0, 0.0

# Velocity and yaw-rate saturation
MAX_VEL_XY   = 1.0
MAX_VEL_Z    = 0.7
MAX_YAW_RATE = 1.5

def clamp(val, mn, mx):
    return max(mn, min(val, mx))

def angle_difference(tgt, cur):
    diff = (tgt - cur + math.pi) % (2 * math.pi) - math.pi
    return diff

def controller(state, target_pos, dt):
    """Single-layer PID using local-yaw frame for X/Y, simple PIDs for Z and yaw."""
    if dt <= 0:
        return (0.0, 0.0, 0.0, 0.0)

    x, y, z, _, _, yaw = state
    tx, ty, tz, tyaw = target_pos

    exw = tx - x
    eyw = ty - y
    ezw = tz - z

    # Rotate X/Y errors into local yaw frame
    cy = math.cos(-yaw)
    sy = math.sin(-yaw)
    exl = exw * cy - eyw * sy
    eyl = exw * sy + eyw * cy

    pid_data['xy']['int_x'] += exl * dt
    pid_data['xy']['int_y'] += eyl * dt

    dx = (exl - pid_data['xy']['prev_x']) / dt
    dy = (eyl - pid_data['xy']['prev_y']) / dt
    pid_data['xy']['prev_x'] = exl
    pid_data['xy']['prev_y'] = eyl

    vx = Kp_xy * exl + Ki_xy * pid_data['xy']['int_x'] + Kd_xy * dx
    vy = Kp_xy * eyl + Ki_xy * pid_data['xy']['int_y'] + Kd_xy * dy

    pid_data['z']['int_z'] += ezw * dt
    dz = (ezw - pid_data['z']['prev_z']) / dt
    pid_data['z']['prev_z'] = ezw
    vz = Kp_z * ezw + Ki_z * pid_data['z']['int_z'] + Kd_z * dz

    eyaw = angle_difference(tyaw, yaw)
    pid_data['yaw']['int_yaw'] += eyaw * dt
    dyaw = (eyaw - pid_data['yaw']['prev_yaw']) / dt
    pid_data['yaw']['prev_yaw'] = eyaw
    vyaw = Kp_yaw * eyaw + Ki_yaw * pid_data['yaw']['int_yaw'] + Kd_yaw * dyaw

    vx   = clamp(vx,   -MAX_VEL_XY,   MAX_VEL_XY)
    vy   = clamp(vy,   -MAX_VEL_XY,   MAX_VEL_XY)
    vz   = clamp(vz,   -MAX_VEL_Z,    MAX_VEL_Z)
    vyaw = clamp(vyaw, -MAX_YAW_RATE, MAX_YAW_RATE)

    return (vx, vy, vz, vyaw)
