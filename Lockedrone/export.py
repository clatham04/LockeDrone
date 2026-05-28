from ultralytics import YOLO

# This downloads the base model and compiles it into the NCNN folder
model = YOLO("yolo11n.pt")
model.export(format="ncnn")