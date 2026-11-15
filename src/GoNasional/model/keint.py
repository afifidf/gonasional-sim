from ultralytics import YOLO

model = YOLO("/home/aroc2/BismilahNasional/src/GoNasional/model/gkeep_sementara.pt")

model.export(format="openvino")