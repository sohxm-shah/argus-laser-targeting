from ultralytics import YOLO
import cv2
import time

model = YOLO("models/argus_model.pt")
model.model.names = {0: "balloon"}

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error! The webcam could not be opened.")
    exit()

previous_time = time.time()

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
            print(f"Target Center: ({int(cx)}, {int(cy)})")

            cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 0), -1)  #drawing the center point
            center_marker = f"({cx}, {cy})" #marking the center point
            cv2.putText(annotated_frame, center_marker, (cx - 40, cy + 22),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)

    current_time = time.time()
    fps = 1 / (current_time - previous_time)
    previous_time = current_time

    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 1)

    cv2.imshow("A.R.G.U.S. Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()