import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Usaremos Point para enviar coordenadas (x, dist)
import cv2
import torch
import numpy as np

class VisionSillas(Node):
    def __init__(self):
        super().__init__('vision_sillas_node')
        
        # Suscripción a la cámara del robot (MVSim suele usar /camera/image_raw o /image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.listener_callback,
            10)
            
        # Publicador: Enviará coordenadas de la silla detectada
        # x: -1 (izquierda) a 1 (derecha), 0 es centro
        # z: área de la silla (para estimar distancia)
        self.publisher_ = self.create_publisher(Point, '/silla_detectada', 10)
        
        # Cargar YOLOv5
        self.get_logger().info("Cargando modelo YOLOv5 (Buscando Sillas)...")
        # Usamos 'yolov5s' normal que descarga de internet si no está local
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        except:
            # Fallback si tienes el archivo local como en tu código original
            self.model = torch.hub.load('.', 'custom', path='yolov5s.pt', source='local')

        self.model.classes = [56]  # CLASE 56 = SILLA en COCO Dataset
        self.model.conf = 0.5      # Confianza mínima

    def listener_callback(self, msg):
        try:
            height = msg.height
            width = msg.width
            # Conversión manual ROS Image -> OpenCV (para evitar líos con cv_bridge si no lo tienes)
            img_data = np.array(msg.data, dtype=np.uint8)
            img = img_data.reshape((height, width, 3))
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
            return

        # Detección YOLO
        results = self.model(img)
        detections = results.xyxy[0].cpu().numpy() # [x1, y1, x2, y2, conf, cls]

        msg_point = Point()
        hay_silla = False
        
        # Buscamos la silla con mayor confianza
        best_conf = 0
        best_center = 0
        best_area = 0

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            
            # Calcular centro y área
            center_x = (x1 + x2) / 2
            area = (x2 - x1) * (y2 - y1)
            
            if conf > best_conf:
                best_conf = conf
                # Normalizamos X entre -1 (izq) y 1 (der)
                best_center = (center_x - (width / 2)) / (width / 2)
                best_area = area
                hay_silla = True
                
                # Dibujar bounding box para debug
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        if hay_silla:
            msg_point.x = float(best_center) # Dirección
            msg_point.z = float(best_area)   # "Distancia" inversa (más grande = más cerca)
            self.publisher_.publish(msg_point)
            # self.get_logger().info(f"Silla en X: {best_center:.2f}, Area: {best_area}")

        # Mostrar imagen (Opcional, consume recursos)
        cv2.imshow("Vision Robot", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionSillas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
