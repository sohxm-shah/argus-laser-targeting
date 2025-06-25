from pymavlink import mavutil
import time

connectionString = '/dev/ttyUSB0'
baudRate = 57600

print("Connecting to Pixhawk...") #pixhawk connection
master = mavutil.mavlink_connection(connectionString, baud=baudRate)
master.wait_heartbeat()
print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

def getCurrentMode():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        0, 0, 0, 0, 0, 0
    )
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    modeName = mavutil.mode_string_v10(msg)
    print(f"Current mode: {modeName}")
    return modeName

def setMode(modeName):
    modeMapping = master.mode_mapping()
    if modeName not in modeMapping:
        print(f"Mode {modeName} is not available in mode mapping.")
        print(f"Available modes: {list(modeMapping.keys())}")
        return

    modeId = modeMapping[modeName]
    print(f"Setting mode to {modeName} (mode ID: {modeId})")

    master.set_mode(modeId)

    while True:
        currentMode = getCurrentMode()
        if currentMode == modeName:
            print(f"Mode successfully set to {modeName}")
            break
        time.sleep(1)

getCurrentMode()
setMode("POSHOLD")
