# controller.py

import math

# Local-yaw PID controller.
# Global position errors are rotated into the drone's local frame for X/Y control,
# while Z is controlled directly in the world frame.

# Store integrals and previous errors for the PID controller
pid_state = {
    'xy': {'integral_x': 0.0, 'integral_y': 0.0, 'prev_error_x': 0.0, 'prev_error_y': 0.0},
    'z':  {'integral_z': 0.0, 'prev_error_z': 0.0},
    'yaw': {'integral_yaw': 0.0, 'prev_error_yaw': 0.0}
}

# PID gains - K_u is measured to be 1.2 and T_u to be 2.4s
kp_xy, ki_xy, kd_xy = 0.5, 0.2, 0.05
kp_z,  ki_z,  kd_z  = 0.5, 0.2, 0.05
kp_yaw, ki_yaw, kd_yaw = 0.5, 0.2, 0.05

# Command limits to prevent extreme velocity or yaw rate
MAX_VEL_XY   = 2.0
MAX_VEL_Z    = 1.4
MAX_YAW_RATE = 3.0

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def angle_difference(target, current):
    # Signed difference, ensuring result is in (-pi, pi)
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    return diff

def controller(state, target, dt):
    """
    Single-layer PID controller that outputs local velocity commands.
    
    state: [x, y, z, roll, pitch, yaw]
    target: (target_x, target_y, target_z, target_yaw)
    dt: time step (seconds)
    
    Returns: (velocity_x_local, velocity_y_local, velocity_z, yaw_rate)
    """
    if dt <= 0:
        return (0.0, 0.0, 0.0, 0.0)

    # Current position and orientation
    x, y, z, _, _, yaw = state
    # Desired position and orientation
    target_x, target_y, target_z, target_yaw = target

    # --- Position errors in world frame ---
    error_world_x = target_x - x
    error_world_y = target_y - y
    error_world_z = target_z - z

    # --- Rotate X/Y errors into the drone's local frame ---
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    error_local_x = error_world_x * cos_yaw - error_world_y * sin_yaw
    error_local_y = error_world_x * sin_yaw + error_world_y * cos_yaw

    # --- X/Y local PID ---
    pid_state['xy']['integral_x'] += error_local_x * dt
    pid_state['xy']['integral_y'] += error_local_y * dt
    derivative_x = (error_local_x - pid_state['xy']['prev_error_x']) / dt
    derivative_y = (error_local_y - pid_state['xy']['prev_error_y']) / dt
    pid_state['xy']['prev_error_x'] = error_local_x
    pid_state['xy']['prev_error_y'] = error_local_y

    vel_x = kp_xy * error_local_x + ki_xy * pid_state['xy']['integral_x'] + kd_xy * derivative_x
    vel_y = kp_xy * error_local_y + ki_xy * pid_state['xy']['integral_y'] + kd_xy * derivative_y

    # --- Z PID (world frame) ---
    pid_state['z']['integral_z'] += error_world_z * dt
    derivative_z = (error_world_z - pid_state['z']['prev_error_z']) / dt
    pid_state['z']['prev_error_z'] = error_world_z
    vel_z = kp_z * error_world_z + ki_z * pid_state['z']['integral_z'] + kd_z * derivative_z

    # --- Yaw PID (angle -> yaw rate) ---
    error_yaw = angle_difference(target_yaw, yaw)
    pid_state['yaw']['integral_yaw'] += error_yaw * dt
    derivative_yaw = (error_yaw - pid_state['yaw']['prev_error_yaw']) / dt
    pid_state['yaw']['prev_error_yaw'] = error_yaw
    yaw_rate = kp_yaw * error_yaw + ki_yaw * pid_state['yaw']['integral_yaw'] + kd_yaw * derivative_yaw

    # --- Clamp velocity and yaw rate ---
    vel_x   = clamp(vel_x,   -MAX_VEL_XY,   MAX_VEL_XY)
    vel_y   = clamp(vel_y,   -MAX_VEL_XY,   MAX_VEL_XY)
    vel_z   = clamp(vel_z,   -MAX_VEL_Z,    MAX_VEL_Z)
    yaw_rate = clamp(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)

    return (vel_x, vel_y, vel_z, yaw_rate)
