# One object box from the YOLO detector, in pixels of the frame it ran on.
string label          # class name from the model: bola, tiang_gawang, titik_penalti, ...
float32 confidence
float32 x1            # left
float32 y1            # top
float32 x2            # right
float32 y2            # bottom
