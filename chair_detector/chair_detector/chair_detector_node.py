#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ChairDetector(Node):
    def __init__(self):
        super().__init__('chair_detector')
        
        # --- 1. TOPIC DE SALIDA ESTANDARIZADO ---
        # Cambiado de 'estado_silla' a '/chair_status'
        self.publisher_ = self.create_publisher(String, '/chair_status', 10)
        
        self.bridge = CvBridge()

        # --- 2. SUSCRIPCIÓN ---
        # Escuchamos el topic compartido '/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )
        
        # Cargamos YOLOv8 (Asegúrate de tener internet la primera vez para descargar el modelo)
        self.get_logger().info('Cargando modelo YOLOv8...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('Detector de Sillas Iniciado.')

    def check_overlap(self, box_persona, box_silla):
        """ Comprueba si la caja de la persona toca la de la silla """
        px1, py1, px2, py2 = box_persona
        sx1, sy1, sx2, sy2 = box_silla

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

        # 2. Detectar con YOLO (Personas=0, Sillas=56 en COCO dataset)
        results = self.model.predict(frame, verbose=False, classes=[0, 56], conf=0.4)
        
        personas = []
        sillas = []

        # Extraer coordenadas
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
        silla_detectada_visual = False # Para saber si pintar caja o no

        for silla in sillas:
            ocupada_esta_vez = False
            for persona in personas:
                # Usamos la función de superposición
                if self.check_overlap(persona, silla):
                    ocupada_esta_vez = True
                    sillas_ocupadas += 1
                    break 
            
            # --- DIBUJAR PARA DEBUG (Opcional, consume algo de CPU) ---
            color = (0, 0, 255) if ocupada_esta_vez else (0, 255, 0) # Rojo=Ocupada, Verde=Libre
            label = "OCCUPIED" if ocupada_esta_vez else "FREE"
            
            sx1, sy1, sx2, sy2 = map(int, silla)
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), color, 3)
            cv2.putText(frame, label, (sx1, sy1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 4. Publicar Mensaje ESTANDARIZADO
        # IMPORTANTE: Enviamos en inglés para que coincida con el resto del sistema
        msg_str = String()
        
        if sillas_ocupadas > 0:
            msg_str.data = "occupied"
        else:
            msg_str.data = "free"

        self.publisher_.publish(msg_str)
        
        # 5. Mostrar Video (Solo funcionará si tienes monitor en el PC 2)
        cv2.imshow("YOLO Chair Detector", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ChairDetector()
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
