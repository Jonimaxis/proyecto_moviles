#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image  # para recibir la imagen
from cv_bridge import CvBridge     # para convertir la imagen
from ultralytics import YOLO
import cv2

class SillaDetector(Node):
    def __init__(self):
        super().__init__('detector_sillas_node')
        
        # Publicaremos en el topic '/estado_silla'
        self.publisher_ = self.create_publisher(String, 'estado_silla', 10)
        
        # Herramienta de conversión
        self.bridge = CvBridge()

        # SUSCRIPCIÓN: Escuchamos el topic compartido '/image_raw'
        # Ya no nos conectamos a la IP aquí, eso lo hace el driver
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )
        
        # Cargamos YOLO
        self.model = YOLO('yolov8n.pt')
        
        self.get_logger().info('Detector de Sillas (Subscriber) Iniciado.')

    def hay_superposicion(self, caja_persona, caja_silla):
        px1, py1, px2, py2 = caja_persona
        sx1, sy1, sx2, sy2 = caja_silla

        x_inter_1 = max(px1, sx1)
        y_inter_1 = max(py1, sy1)
        x_inter_2 = min(px2, sx2)
        y_inter_2 = min(py2, sy2)

        if x_inter_2 < x_inter_1 or y_inter_2 < y_inter_1:
            return False 
        return True

    def listener_callback(self, msg):
        # 1. Convertir mensaje ROS a imagen OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        # 2. Detectar con YOLO (Personas=0, Sillas=56)
        results = self.model.predict(frame, verbose=False, classes=[0, 56], conf=0.4)
        
        personas = []
        sillas = []

        for r in results:
            for box in r.boxes:
                coords = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                if cls_id == 0:
                    personas.append(coords)
                elif cls_id == 56:
                    sillas.append(coords)

        # 3. Lógica de Ocupación
        sillas_ocupadas = 0

        for silla in sillas:
            ocupada_esta_vez = False
            for persona in personas:
                if self.hay_superposicion(persona, silla):
                    ocupada_esta_vez = True
                    sillas_ocupadas += 1
                    break 
            
            # Dibujar
            color = (0, 0, 255) if ocupada_esta_vez else (0, 255, 0)
            sx1, sy1, sx2, sy2 = map(int, silla)
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), color, 3)
            cv2.putText(frame, "OCUPADA" if ocupada_esta_vez else "LIBRE", 
                        (sx1, sy1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 4. Publicar Mensaje
        msg_str = String()
        if sillas_ocupadas > 0:
            msg_str.data = f"ocupada"
        else:
            msg_str.data = "libre"

        self.publisher_.publish(msg_str)
        
        # 5. Mostrar Video
        cv2.imshow("Detector Sillas YOLO", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SillaDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
