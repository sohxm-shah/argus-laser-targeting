from ultralytics import YOLO
import cv2
import time

model = YOLO("models/argus_model.pt")
model.model.names = {0: "balloon"}

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error! The webcam could not be opened.")
    exit()

def mapping_value(x, in_min, in_max, out_min, out_max): #range mapping function
    x = max(min(x, in_max), in_min) #converting the obtained pixels into servo angle
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def servo_output(pan_angle, tilt_angle):
    pan_angle = int(max(min(pan_angle, 180), 0))
    tilt_angle = int(max(min(tilt_angle, 180), 0))
    print(f"Servo Target: Pan: {pan_angle}°, Tilt: {tilt_angle}°")

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) #frame size for mapping
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to get the frame")
        break

    results = model.predict(source=frame, conf=0.80, verbose=False)
    annotated_frame = results[0].plot()

    boxes = results[0].boxes
    if boxes is not None and boxes.xyxy is not None:
        for box in boxes.xyxy:
            x1, y1, x2, y2 = box[:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            pan_angle = mapping_value(cx, 0, frame_width, 0, 180) #getting servo values from obtained pixels
            tilt_angle = mapping_value(cy, 0, frame_height, 0, 180)

            servo_output(pan_angle, tilt_angle)

            cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 0), -1)
            label = f"({cx}, {cy})"
            cv2.putText(annotated_frame, label, (cx - 40, cy + 22),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)


    previous_time = time.time()
    current_time = time.time()
    fps = 1 / (current_time - previous_time)
    previous_time = current_time

    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 1)

    cv2.imshow("A.R.G.U.S. Detection + Servo Mapping", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
