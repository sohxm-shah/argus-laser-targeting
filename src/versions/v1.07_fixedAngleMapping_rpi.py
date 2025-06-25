import socket
import json
from ultralytics import YOLO
import cv2
import time
from datetime import datetime
import csv  # for logging gestures
from pathlib import Path
from pymavlink import mavutil


'''import RPi.GPIO as GPIO

LASER_PIN = 17
SERVO_PIN = 12  # GPIO12(PWM0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servoPwm = GPIO.PWM(SERVO_PIN, 50)  # 50hz pwm for servo
servoPwm.start(0)
'''

model = YOLO("models/argus_model.pt")
model.model.names = {0: "balloon"}

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error! The webcam could not be opened.")
    exit()

frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

CAMERA_HORIZONTAL_FOV = 62.2  # degrees
CAMERA_VERTICAL_FOV = 48.8    # degrees

print("[INFO] Connecting to MAVLink on TCP port 5762...") #setting up mavlink
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

print("[INFO] Waiting for heartbeat...")
master.wait_heartbeat()
print(f"[SUCCESS] Heartbeat received from system {master.target_system}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #udp socket for gestures
sock.bind(("0.0.0.0", 4210))
print("Awaiting gesture packets...")

previousTime = time.time()

gestureCooldown = 1.0
lastGestureTime = 0

Path("logs").mkdir(exist_ok=True)  # logging gestures with timestamp in a csv file
logFile = open("logs/aim_log.csv", mode='a', newline='')
csvWriter = csv.writer(logFile)
csvWriter.writerow(["Time", "Gesture", "cx", "cy"])

def mappingValue(x, inMin, inMax, outMin, outMax):
    x = max(min(x, inMax), inMin)
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

#fov based mapping from cx, cy to servo angles
def mapToServoAngle(cx, cy):
    xAngle = ((cx - frameWidth / 2) / frameWidth) * CAMERA_HORIZONTAL_FOV
    yAngle = ((cy - frameHeight / 2) / frameHeight) * CAMERA_VERTICAL_FOV

    panAngle = xAngle + 90
    tiltAngle = yAngle + 90

    panAngle = max(min(panAngle, 180), 0)
    tiltAngle = max(min(tiltAngle, 180), 0)

    return panAngle, tiltAngle

def setServoAngle(angle):
    angle = max(min(angle, 180), 0)
    dutyCycle = 2 + (angle / 18)
#    servoPwm.ChangeDutyCycle(dutyCycle)
    print(f"    Servo Angle Set: {angle}°")

def fireLaser(duration=1.0):
    print("Laser ON")
#    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(duration)
#    GPIO.output(LASER_PIN, GPIO.LOW)
    print("Laser OFF")

def sendYawCommand(yawAngle): #function to send calculated yaw angle to the the pixhawk
    print(f"[MAVLINK] Sending yaw command: {yawAngle:.2f}°")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,  # Confirmation
        yawAngle,  # Param 1: target angle
        10,        # Param 2: yaw speed deg/s
        1,         # Param 3: direction (1=cw)
        0,         # Param 4: relative offset (0 = absolute)
        0, 0, 0    # Param 5-7: unused
    )

# killswitch mechanism
killedStatus = False
previousGesture = None

def handleAim(boxes):
    if boxes is not None and boxes.xyxy is not None and len(boxes.xyxy) > 0:
        logTime = datetime.now().strftime("%H:%M:%S")
        print(f"\n[{logTime}] Target Acquired! Logging and Aiming...\n")

        for i, box in enumerate(boxes.xyxy):
            x1, y1, x2, y2 = box[:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            panAngle, tiltAngle = mapToServoAngle(cx, cy)

            print(f"Target {i + 1}")
            print(f"    Center     : ({cx}, {cy})")
            print(f"    Pan Angle  : {panAngle:.2f}° (Yaw)")
            print(f"    Tilt Angle : {tiltAngle:.2f}° (Servo)")

            setServoAngle(tiltAngle)
            sendYawCommand(panAngle)

            csvWriter.writerow([logTime, "Aim", cx, cy])
    else:
        print("NO targets found for Aiming.")

def handleShoot():
    print("SHOOT command received! Firing laser...")
    fireLaser()
    csvWriter.writerow([datetime.now().strftime("%H:%M:%S"), "Shoot", "", ""])

def handleKillswitch():
    global killedStatus
    killedStatus = True
    print("KILLSWITCH ACTIVATED — system locked.")

def handleKillReset():  # to re-enable the system
    global killedStatus
    killedStatus = False
    print("SYSTEM UNLOCKED — gesture control re-enabled.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to get the frame")
        break

    results = model.predict(source=frame, conf=0.80, verbose=False)
    boxes = results[0].boxes
    annotatedFrame = results[0].plot()

    if boxes is not None and boxes.xyxy is not None and len(boxes.xyxy) > 0:
        for i, box in enumerate(boxes.xyxy):
            x1, y1, x2, y2 = box[:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.circle(annotatedFrame, (cx, cy), 5, (0, 0, 0), -1)
            cv2.putText(annotatedFrame, f"({cx}, {cy})", (cx - 40, cy + 22),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            cv2.putText(annotatedFrame, f"Target {i + 1}", (cx - 30, cy + 40),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 200, 255), 1)

    currentTime = time.time()
    fps = 1 / (currentTime - previousTime)
    previousTime = currentTime

    cv2.putText(annotatedFrame, f"FPS: {int(fps)}", (10, 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 2)

    currentTimeLabel = datetime.now().strftime("%H:%M:%S")
    (textW, _), _ = cv2.getTextSize(currentTimeLabel, cv2.FONT_HERSHEY_DUPLEX, 0.5, 3)
    xPos = frameWidth - textW - 10
    cv2.putText(annotatedFrame, currentTimeLabel, (xPos, 25),
                cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)

    try:
        sock.settimeout(0.01)
        data, addr = sock.recvfrom(1024)
        decoded = data.decode('utf-8')
        parsed = json.loads(decoded)
        gesture = parsed.get("gesture")

        currentTime = time.time()
        if gesture and gesture != previousGesture and (currentTime - lastGestureTime) > gestureCooldown:
            print(f"Gesture Received: {gesture}")
            previousGesture = gesture
            lastGestureTime = currentTime

            if killedStatus and gesture != "KillReset":
                print("Ignored due to KILLSWITCH.")
            elif gesture == "Aim":
                handleAim(boxes)
            elif gesture == "Shoot":
                handleShoot()
            elif gesture == "Killswitch":
                handleKillswitch()
            elif gesture == "KillReset":
                handleKillReset()

    except socket.timeout:
        pass

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    cv2.imshow("A.R.G.U.S. — Gesture Control", annotatedFrame)

cap.release()
cv2.destroyAllWindows()
logFile.close()

#servoPwm.stop()
#GPIO.cleanup()