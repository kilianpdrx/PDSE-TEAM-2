from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("yolo11n-seg.pt")

# Export the model
model.export(format="imx")  # exports with PTQ quantization by default

# # Load the exported model
# imx_model = YOLO("yolov8n_imx_model")

# # Run inference
# results = imx_model("https://ultralytics.com/images/bus.jpg")