from pymavlink import mavutil
import time

# STEP 1: Connect to MAVLink (adjust port if needed)
print("[INFO] Connecting to MAVLink on tcp:127.0.0.1:5762...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# STEP 2: Wait for heartbeat (confirms connection to autopilot)
print("[INFO] Waiting for heartbeat...")
master.wait_heartbeat()
print(f"[SUCCESS] Heartbeat received from system {master.target_system}, component {master.target_component}")

# STEP 3: Set flight mode to POSHOLD (mode 16)
print("[ACTION] Setting mode to POSHOLD...")
master.mav.command_long_send(
    master.target_system,              # Target system ID
    master.target_component,           # Target component ID
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command ID to set mode
    0,                                 # Confirmation
    1,                                 # Mode flag (1 for custom mode)
    16,                                # POSHOLD mode number in ArduCopter
    0, 0, 0, 0, 0                      # Unused parameters
)
print("[SUCCESS] Mode change command sent.")

# STEP 4: Send yaw command
# Rotate to 90 degrees (East) at 15°/sec, CW, absolute
desiredYaw = 90
yawSpeed = 15
direction = 1         # 1 = CW, -1 = CCW
isRelative = 0        # 0 = absolute, 1 = relative

print(f"[ACTION] Sending yaw command to {desiredYaw}°, speed {yawSpeed}°/s")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Yaw control command
    0,                # Confirmation
    desiredYaw,       # Target yaw angle (deg)
    yawSpeed,         # Speed to rotate (deg/s)
    direction,        # Direction: 1=CW, -1=CCW
    isRelative,       # Relative(1) or absolute(0)
    0, 0, 0           # Unused
)
print("[SUCCESS] Yaw command sent.")
