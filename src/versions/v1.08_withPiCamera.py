import socket
import json
from ultralytics import YOLO
import cv2
import numpy as np
import time
from datetime import datetime
import csv  # for logging gestures
from pathlib import Path
from pymavlink import mavutil
from picamera2 import Picamera2

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

cam = Picamera2() #initializing PiCamera2
cam.configure(cam.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
cam.start()

frameWidth = 640
frameHeight = 480

CAMERA_HORIZONTAL_FOV = 62.2  # degrees
CAMERA_VERTICAL_FOV = 48.8    # degrees

print("[INFO] Connecting to MAVLink on TCP port...")
master = mavutil.mavlink_connection('tcp:192.168.189.18:5762')

print("[INFO] Waiting for heartbeat...")
master.wait_heartbeat()
print(f"[SUCCESS] Heartbeat received from system {master.target_system}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 4210))
print("Awaiting gesture packets...")

previousTime = time.time()
gestureCooldown = 1.0
lastGestureTime = 0
Path("logs").mkdir(exist_ok=True)
logFile = open("logs/aim_log.csv", mode='a', newline='')
csvWriter = csv.writer(logFile)
csvWriter.writerow(["Time", "Gesture", "cx", "cy"])

def mappingValue(x, inMin, inMax, outMin, outMax):
    x = max(min(x, inMax), inMin)
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def mapToServoAngle(cx, cy):
    xAngle = ((cx - frameWidth / 2) / frameWidth) * CAMERA_HORIZONTAL_FOV
    yAngle = ((cy - frameHeight / 2) / frameHeight) * CAMERA_VERTICAL_FOV
    panAngle = max(min(xAngle + 90, 180), 0)
    tiltAngle = max(min(yAngle + 90, 180), 0)
    return panAngle, tiltAngle

def setServoAngle(angle):
    angle = max(min(angle, 180), 0)
    dutyCycle = 2 + (angle / 18)
#    servoPwm.ChangeDutyCycle(dutyCycle)
    print(f"    Servo Angle Set: {angle}Â°")

def fireLaser(duration=1.0):
    print("Laser ON")
#    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(duration)
#    GPIO.output(LASER_PIN, GPIO.LOW)
    print("Laser OFF")

def sendYawCommand(yawAngle):
    print(f"[MAVLINK] Sending yaw command: {yawAngle:.2f}Â°")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yawAngle, 10, 1, 0, 0, 0, 0
    )

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
            print(f"    Pan Angle  : {panAngle:.2f}Â° (Yaw)")
            print(f"    Tilt Angle : {tiltAngle:.2f}Â° (Servo)")
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
    print("KILLSWITCH ACTIVATED! System locked.")

def handleKillReset():
    global killedStatus
    killedStatus = False
    print("SYSTEM UNLOCKED! Gesture control re-enabled.")

while True:
    frame = cam.capture_array()
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

            if gesture == "Killswitch":
                handleKillswitch()
            else:
                if killedStatus:
                    print("SYSTEM UNLOCKED! Kill reset triggered automatically.")
                    handleKillReset()
                if gesture == "Aim":
                    handleAim(boxes)
                elif gesture == "Shoot":
                    handleShoot()

    except socket.timeout:
        pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("A.R.G.U.S. Detection Window", annotatedFrame)

cam.close()
cv2.destroyAllWindows()
logFile.close()
#servoPwm.stop()
#GPIO.cleanup()