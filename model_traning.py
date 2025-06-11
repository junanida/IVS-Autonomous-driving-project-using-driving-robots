from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(
    data="./data.yaml",
    epochs=75,
    imgsz=640,
    batch=16,
    name="traffic_light_colab"
)