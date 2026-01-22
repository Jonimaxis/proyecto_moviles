#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DriverIP(Node):
    def __init__(self):
        super().__init__('driver_ip')
        
        # --- 1. PARÁMETROS DE CONFIGURACIÓN ---
        # Definimos valores por defecto, pero se pueden cambiar al lanzar el nodo
        self.declare_parameter('ip', '192.168.1.63')
        self.declare_parameter('port', '8080')
        
        # Obtenemos los valores actuales
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value
        
        # Construimos la URL
        # Nota: Hay que asegurarse de que la app 'IP Webcam' usa '/video' o '/shot.jpg' según config
        self.url = f'http://{ip}:{port}/video'
        
        # --- 2. PUBLICADOR ---
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # ~20 FPS
        self.bridge = CvBridge()
        
        self.get_logger().info(f"Intentando conectar a: {self.url} ...")
        self.cap = cv2.VideoCapture(self.url)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"FALLO DE CONEXIÓN. Verifica la IP y que la app esté abierta.")
        else:
            self.get_logger().info(f"CONECTADO EXITOSAMENTE a la cámara.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convertir y publicar
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_optical_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error conversión: {e}")
        else:
            # Si falla la lectura, intentamos reconectar
            # self.get_logger().warning("Perdida conexión con cámara...")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DriverIP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
