import socket
import json
from ultralytics import YOLO
import cv2
import time
from datetime import datetime
import csv #for logging gestures
'''import RPi.GPIO as GPIO

LASER_PIN = 17
SERVO_PIN = 12  #GPIO12(PWM0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servoPwm = GPIO.PWM(SERVO_PIN, 50)  #50hz pwm for servo
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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 4210))
print("Awaiting gesture packets...")

previousTime = time.time()

gestureCooldown = 1.0
lastGestureTime = 0

logFile = open("logs/aim_log.csv", mode='a', newline='')
csvWriter = csv.writer(logFile)
csvWriter.writerow(["Time", "Gesture", "cx", "cy"])

def mappingValue(x, inMin, inMax, outMin, outMax):
    x = max(min(x, inMax), inMin)
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def setServoAngle(angle):
    angle = max(min(angle, 180), 0)
    dutyCycle = 2 + (angle / 18)  # Map 0–180° to ~2–12% duty
#    servoPwm.ChangeDutyCycle(dutyCycle)
    print(f"    Servo Angle Set: {angle}°")

def fireLaser(duration=1.0):
    print("Laser ON")
#    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(duration)
#    GPIO.output(LASER_PIN, GPIO.LOW)
    print("Laser OFF")

#killswitch mechanism
killedStatus = False
previousGesture = None

def handleAim(boxes):
    if boxes is not None and boxes.xyxy is not None and len(boxes.xyxy) > 0:
        logTime = datetime.now().strftime("%H:%M:%S")
        print(f"\n[{logTime}] Target Acquired! Logging Targets: \n")

        for i, box in enumerate(boxes.xyxy):
            x1, y1, x2, y2 = box[:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            panAngle = mappingValue(cx, 0, frameWidth, 0, 180)   # Placeholder if yaw needed
            tiltAngle = mappingValue(cy, 0, frameHeight, 0, 180)

            print(f"Target {i + 1}")
            print(f"    Center : ({cx}, {cy})")
            setServoAngle(tiltAngle)

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

def handleKillReset():
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

    #logic for receiving gestures
    try:
        sock.settimeout(0.01)
        data, addr = sock.recvfrom(1024)
        decoded = data.decode('utf-8')
        parsed = json.loads(decoded)
        gesture = parsed.get("gesture")

        currentTime = time.time()
        # MODIFIED: Add cooldown and kill reset check
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
