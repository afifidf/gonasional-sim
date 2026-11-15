import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String , Float32
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
from .kalman_filter import KalmanFilter2D
from .ema_filter import DataFilter
from sensor_msgs.msg import JointState
# from .landmark import hitung_bearing
from .main_trigono import DistanceEstimator
# from .Trigonometri import hitung, get_offset_angle,dfov_to_hv_fov
import math


kf = KalmanFilter2D()
# window_size=1 -> median dilewati, murni EMA
ema_x = DataFilter(window_size=1, alpha=0.2)
ema_y = DataFilter(window_size=1, alpha=0.2)

SET_CONF = 0.6
IMG_WIDTH = 450
IMG_HEIGHT = 337

DFOV_DEG      = 78.0     # diagonal FOV dari datasheet kamera
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
ASPECT_W      = 4        # 640x480 -> rasio 4:3
ASPECT_H      = 3
# TARGET_FPS    = 25


class VisionYolo(Node):
    def __init__(self):
        super().__init__('usb_cam_yolo')

        self.bridge = CvBridge()
        self.Distance = DistanceEstimator()
    
        #MODEL HARUS DI CONVERT DI PC YANG AKAN DI GUNAKAN!!!!! WAJIB !!!!

        self.model = YOLO(  
            "/home/aroc/BismilahNasional/src/GoNasional/model/nbaru_openvino_model",
            task="detect"
        )

        # FPS
        self.prev_time = 0.0
        self.fps = 0.0

        # Publisher hasil deteksi
        self.obj_pub = self.create_publisher(
            String,
            '/obj_detect',
            10
        )
        self.create_subscription(JointState, '/robotis/goal_joint_states', self.joint_derajat, 10)  

        # Subscriber kamera
        self.create_subscription(
            Image,
            '/usb_cam_node/image_raw',
            self.image_callback,
            1
        )
        #publis jarak dengan kamera 
        self.jarak_pub = self.create_publisher(
            Float32, 
            '/vision/jarak_kamera'
            , 10)
        
        self.hasil_filter = self.create_publisher(
            String,
            '/Hasil/Filter',
            10
        )
        # self.hfov, self.vfov = dfov_to_hv_fov(DFOV_DEG, ASPECT_W, ASPECT_H)
        # self.get_logger().info(f"[INFO] dFOV={DFOV_DEG}°  ->  HFOV={self.hfov:2f}°  VFOV={self.vfov:.2f}°")

        self.get_logger().info("✅ YOLO OpenVINO ROS2 Node Jalan BROOOOOOOO")
        
        self.FOCAL_LENGTH = 900   # hasil kalibrasi (contoh)
        self.BALL_DIAMETER = 5  # cm
        self.KALIBRASI = 7 # cm
        self.derajat_kamera = 0.0

    def joint_derajat(self, msg: JointState):
            for i, name in enumerate(msg.name):
                if name == 'head_tilt':
                    self.derajat_kamera = msg.position[i]

    def image_callback(self, msg):
        # if time.time() - self.prev_time< 1.0 / TARGET_FPS:
        #     return
        try:
            # =========================
            # IMAGE CONVERT
            # =========================
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
            

            frame = cv2.resize(frame, (IMG_WIDTH, IMG_HEIGHT))

            # =========================
            # YOLO INFERENCE
            # =========================
            results = self.model(
                frame,
                imgsz=480,
                conf=SET_CONF,
                device="cpu",
                verbose=False
            )

            r = results[0]
            annotated_frame = r.plot()

            
            # landmark
            # landmark = hitung_bearing(frame, r)

            # servo_head = self.derajat_kamera

            # for obj in landmark:
            #     hasil = self.Distance.hitung(
            #         servo_head,
            #         obj["vertical"]
            #     )
            #     if hasil is None:
            #         continue
                

                # self.get_logger().info(
                #     f"{obj['label']} | "
                #     f"H={math.degrees(obj['horizontal']):.2f} | "
                #     f"V={obj['vertical']:.2f} | "
                #     f"jarak={round(hasil['distance_robot'], 2)} cm"
                #     f"servo head= {servo_head}"
                # )

            best_conf = 0.0
            best_cx = None
            best_cy = None

            # =========================
            # AMBIL DETEKSI TERBAIK (BOLA)
            # =========================
            if r.boxes is not None and len(r.boxes) > 0:

                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    # label = model.names[cls_id]
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = r.names[cls_id]

                    if class_name != "bola":
                        continue

                    if conf > best_conf:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        
                        jarak = self.vision_jarak(x1, y1, x2, y2)
                        jarak_final = jarak - self.KALIBRASI
                        # self.get_logger().info(f"Distance: {jarak_final:.2f} cm")
                        # time.sleep(1.0)

                        if jarak_final is not None:
                            msg = Float32()
                            msg.data = float(jarak_final)
                            self.jarak_pub.publish(msg)

                        best_conf = conf
                        best_cx = cx
                        best_cy = cy

            # =========================
            # PUBLISH
            # =========================
            if best_cx is not None:
                # self.get_logger().info(
                #     f"BEST DETECTION ||||| conf={best_conf:.2f} | center=({best_cx},{best_cy})"
                # )
                # kf.predict()
                # best_cx,best_cy =kf.update(best_cx,best_cy)
                best_cx = int(round(ema_x.update(best_cx)))
                best_cy = int(round(ema_y.update(best_cy)))
                # self.get_logger().info(
                #     f"BEST DETECTION FILTER ||||| conf={best_conf:.2f} | center=({best_cx},{best_cy})"
                # )
                msg_out = String()
                msg_out.data = f"{best_cx},{best_cy}"
                self.obj_pub.publish(msg_out)
            ########################
            #FILTER
                kf.predict()
                x,y =kf.update(best_cx,best_cy)
                msg_out_filter = String()
                msg_out_filter.data= f"{x},{y}"
                self.hasil_filter.publish(msg_out_filter)   
        

        
            # =========================
            # HITUNG FPS
            # =========================
            curr_time = time.time()
            if self.prev_time != 0:
                self.fps = 1.0 / (curr_time - self.prev_time)
            self.prev_time = curr_time

            # cv2.putText(
            #     annotated_frame,
            #     f"FPS: {self.fps:.2f}",
            #     (20, 40),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2
            # )
            # degree = math.degrees(self.derajat_kamera)
            self.get_logger().info(f"FPS: {self.fps:.2f}")
            # self.get_logger().info(f"Jarak: {self.derajat_kamera:.2f}")
            # self.get_logger().info(f"degrees: {degree:.2f}")
            # offset_x, offset_y = get_offset_angle(
            #     (x1, y1, x2, y2), FRAME_WIDTH, FRAME_HEIGHT, self.hfov,self.vfov
            # )
            # self.get_logger().info(f"{class_name}: offset_x={offset_x:.2f} deg, offset_y={offset_y:.2f} deg")

            # R2D = math.degrees(self.derajat_kamera)
            # kalibrasi = R2D + 90

            # hasil = hitung(kalibrasi-5)
            # hasil2= hitung(kalibrasi-5 - offset_y)
            # self.get_logger().info(f"Derajat = {R2D}")
            # self.get_logger().info(f"Hasill = {hasil}")
            # self.get_logger().info(f"Hasil2 = {hasil2}")

            # =========================
            # DISPLAY
            # =========================
            cv2.imshow("YOLO OpenVINO ROS2", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def vision_jarak(self, x1, y1, x2, y2):
        # hitung diameter pixel (pakai lebar bbox)
        pixel_width = x2 - x1

        if pixel_width <= 0:
            return None

        distance = (self.FOCAL_LENGTH * self.BALL_DIAMETER) / pixel_width
        # self.get_logger().info(f"Distance: {distance:.2f} cm") 
        return distance
    
def main():
    rclpy.init()
    node = VisionYolo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()