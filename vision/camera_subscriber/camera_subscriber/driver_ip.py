#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DriverIPCam(Node):
    def __init__(self):
        super().__init__('driver_ip_node')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 FPS
        self.bridge = CvBridge()
        
        # --- TU CONFIGURACIÓN ---
        ip_movil = '192.168.1.63'  
        puerto = '8080'
        url = f'http://{ip_movil}:{puerto}/video'
        # ------------------------

        self.cap = cv2.VideoCapture(url)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo conectar a: {url}")
        else:
            self.get_logger().info(f"Conectado a cámara IP: {url}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convertir y publicar
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Error leyendo frame de la IP Webcam")

def main(args=None):
    rclpy.init(args=args)
    node = DriverIPCam()
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
