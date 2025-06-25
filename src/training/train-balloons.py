from ultralytics import YOLO
model = YOLO("yolov8n.pt")

model.train(
    data="Balloon-1/data.yaml",
    epochs=30,
    imgsz=640,
    batch=8
)
