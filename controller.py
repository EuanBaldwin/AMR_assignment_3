# controller.py

import math

###############################################################################
# Refined Cascade PID Controller (3-layer) for x, y, z plus single-loop yaw   #
#                                                                             #
# This example includes:                                                      #
#   - Outer loop: position -> velocity reference                              #
#   - Middle loop: velocity -> acceleration reference                         #
#   - Inner loop: acceleration -> velocity command                            #
#   - Single-loop yaw control (angle -> yaw rate).                            #
#                                                                             #
# After trying these initial PID values, be prepared to tune them:            #
#   - Start with only P-gains (keeping I and D at zero).                      #
#   - Increase P-gains carefully until you see fast enough response but no    #
#     large oscillations. Then, introduce small I-gains if needed to reduce   #
#     steady-state errors, and only add D-gains if you need more damping.     #
###############################################################################

# -------------- Storage for Integrators, Last-Error, etc. -------------------
pid_data = {
    'x': {
        # Position PID
        'pos_err_int': 0.0, 'pos_err_prev': 0.0,
        # Velocity PID
        'vel_err_int': 0.0, 'vel_err_prev': 0.0,
        # Acceleration PID
        'acc_err_int': 0.0, 'acc_err_prev': 0.0,
        # For measured velocity & acceleration
        'last_pos': None,
        'last_vel': 0.0,
    },
    'y': {
        'pos_err_int': 0.0, 'pos_err_prev': 0.0,
        'vel_err_int': 0.0, 'vel_err_prev': 0.0,
        'acc_err_int': 0.0, 'acc_err_prev': 0.0,
        'last_pos': None,
        'last_vel': 0.0,
    },
    'z': {
        'pos_err_int': 0.0, 'pos_err_prev': 0.0,
        'vel_err_int': 0.0, 'vel_err_prev': 0.0,
        'acc_err_int': 0.0, 'acc_err_prev': 0.0,
        'last_pos': None,
        'last_vel': 0.0,
    },
    'yaw': {
        'yaw_err_int': 0.0,
        'yaw_err_prev': 0.0,
        'last_yaw': None
    }
}

# --------------------------- Example PID Gains -------------------------------
# Outer loop: position -> velocity
Kp_pos = {'x': 0.8, 'y': 0.8, 'z': 1.0}
Ki_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
Kd_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

# Middle loop: velocity -> acceleration
Kp_vel = {'x': 0.6, 'y': 0.6, 'z': 0.8}
Ki_vel = {'x': 0.05, 'y': 0.05, 'z': 0.0}
Kd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}

# Inner loop: acceleration -> final velocity command
Kp_acc = {'x': 0.3, 'y': 0.3, 'z': 0.4}
Ki_acc = {'x': 0.02, 'y': 0.02, 'z': 0.0}
Kd_acc = {'x': 0.0, 'y': 0.0, 'z': 0.0}

# Single-loop yaw: angle -> yaw rate
Kp_yaw = 1.0
Ki_yaw = 0.0
Kd_yaw = 0.0

# ------------------------ Command Saturation Limits --------------------------
# (Adjust these based on the drone's capabilities and safety limits)
MAX_VELOCITY_XY = 1.0   # m/s
MAX_VELOCITY_Z  = 0.7   # m/s  (often keep vertical velocity smaller)
MAX_YAW_RATE    = 1.0   # rad/s

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# ----------------------------------------------------------------------------
def controller(state, target_pos, dt):
    """
    Cascade PID controller to achieve desired x,y,z position and yaw angle.
    
    Args:
        state: [pos_x, pos_y, pos_z, roll, pitch, yaw] (SI units/radians)
        target_pos: (tgt_x, tgt_y, tgt_z, tgt_yaw)
        dt: time step in seconds

    Returns:
        (vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd)
         - velocity commands in x,y,z (m/s) and yaw rate (rad/s)
    """

    # --- Extract states ---
    current_x   = state[0]
    current_y   = state[1]
    current_z   = state[2]
    current_yaw = state[5]

    # --- Extract targets ---
    target_x   = target_pos[0]
    target_y   = target_pos[1]
    target_z   = target_pos[2]
    target_yaw = target_pos[3]

    # If dt is invalid, return zero commands
    if dt <= 0.0:
        return (0.0, 0.0, 0.0, 0.0)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 1) Estimate current velocity & acceleration via finite differences
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vx_meas = estimate_velocity('x', current_x, dt)
    vy_meas = estimate_velocity('y', current_y, dt)
    vz_meas = estimate_velocity('z', current_z, dt)

    ax_meas = estimate_acceleration('x', vx_meas, dt)
    ay_meas = estimate_acceleration('y', vy_meas, dt)
    az_meas = estimate_acceleration('z', vz_meas, dt)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 2) Outer loop: position -> velocity reference
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vx_ref = pid_position('x', target_x, current_x, dt)
    vy_ref = pid_position('y', target_y, current_y, dt)
    vz_ref = pid_position('z', target_z, current_z, dt)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 3) Middle loop: velocity -> acceleration reference
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ax_ref = pid_velocity('x', vx_ref, vx_meas, dt)
    ay_ref = pid_velocity('y', vy_ref, vy_meas, dt)
    az_ref = pid_velocity('z', vz_ref, vz_meas, dt)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 4) Inner loop: acceleration -> final velocity command
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vx_cmd = pid_acceleration('x', ax_ref, ax_meas, vx_meas, dt)
    vy_cmd = pid_acceleration('y', ay_ref, ay_meas, vy_meas, dt)
    vz_cmd = pid_acceleration('z', az_ref, az_meas, vz_meas, dt)

    # Apply saturations to velocity commands
    vx_cmd = clamp(vx_cmd, -MAX_VELOCITY_XY, MAX_VELOCITY_XY)
    vy_cmd = clamp(vy_cmd, -MAX_VELOCITY_XY, MAX_VELOCITY_XY)
    vz_cmd = clamp(vz_cmd, -MAX_VELOCITY_Z,  MAX_VELOCITY_Z)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 5) Yaw control (single-loop)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    yaw_rate_cmd = pid_yaw_single_loop(target_yaw, current_yaw, dt)
    yaw_rate_cmd = clamp(yaw_rate_cmd, -MAX_YAW_RATE, MAX_YAW_RATE)

    return (vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd)

# -----------------------------------------------------------------------------
#                    Helper / sub-controller functions
# -----------------------------------------------------------------------------
def estimate_velocity(axis_id, current_pos, dt):
    """
    Use finite difference to estimate velocity on the chosen axis.
    """
    if pid_data[axis_id]['last_pos'] is None:
        # First call, cannot estimate velocity reliably
        pid_data[axis_id]['last_pos'] = current_pos
        return 0.0
    
    pos_prev = pid_data[axis_id]['last_pos']
    vel_est = (current_pos - pos_prev) / dt
    pid_data[axis_id]['last_pos'] = current_pos
    return vel_est

def estimate_acceleration(axis_id, current_vel, dt):
    """
    Use finite difference to estimate acceleration on the chosen axis.
    """
    vel_prev = pid_data[axis_id]['last_vel']
    acc_est = (current_vel - vel_prev) / dt
    pid_data[axis_id]['last_vel'] = current_vel
    return acc_est

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                      Outer Position PID
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def pid_position(axis_id, target_pos, current_pos, dt):
    kp = Kp_pos[axis_id]
    ki = Ki_pos[axis_id]
    kd = Kd_pos[axis_id]

    error = target_pos - current_pos
    d_error = (error - pid_data[axis_id]['pos_err_prev']) / dt

    pid_data[axis_id]['pos_err_int'] += error * dt
    pid_data[axis_id]['pos_err_prev'] = error

    vel_ref = (kp * error
               + ki * pid_data[axis_id]['pos_err_int']
               + kd * d_error)
    return vel_ref

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                      Middle Velocity PID
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def pid_velocity(axis_id, vel_ref, vel_meas, dt):
    kp = Kp_vel[axis_id]
    ki = Ki_vel[axis_id]
    kd = Kd_vel[axis_id]

    error = vel_ref - vel_meas
    d_error = (error - pid_data[axis_id]['vel_err_prev']) / dt

    pid_data[axis_id]['vel_err_int'] += error * dt
    pid_data[axis_id]['vel_err_prev'] = error

    acc_ref = (kp * error
               + ki * pid_data[axis_id]['vel_err_int']
               + kd * d_error)
    return acc_ref

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                      Inner Acceleration PID
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def pid_acceleration(axis_id, acc_ref, acc_meas, vel_meas, dt):
    kp = Kp_acc[axis_id]
    ki = Ki_acc[axis_id]
    kd = Kd_acc[axis_id]

    error = acc_ref - acc_meas
    d_error = (error - pid_data[axis_id]['acc_err_prev']) / dt

    pid_data[axis_id]['acc_err_int'] += error * dt
    pid_data[axis_id]['acc_err_prev'] = error

    velocity_increment = (kp * error
                          + ki * pid_data[axis_id]['acc_err_int']
                          + kd * d_error)
    # Final velocity is measured velocity + increment
    return vel_meas + velocity_increment

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                          Yaw Single-Loop PID
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def pid_yaw_single_loop(target_yaw, current_yaw, dt):
    yaw_error = angle_difference(target_yaw, current_yaw)

    kp = Kp_yaw
    ki = Ki_yaw
    kd = Kd_yaw

    prev_err = pid_data['yaw']['yaw_err_prev']
    pid_data['yaw']['yaw_err_int'] += yaw_error * dt

    d_error = (yaw_error - prev_err) / dt
    pid_data['yaw']['yaw_err_prev'] = yaw_error

    yaw_rate_cmd = (kp * yaw_error
                    + ki * pid_data['yaw']['yaw_err_int']
                    + kd * d_error)
    return yaw_rate_cmd

def angle_difference(target_angle, current_angle):
    """
    Returns smallest signed difference between two angles in radians,
    handling 2*pi wrap-around.
    """
    diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
    return diff
