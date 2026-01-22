import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Necesario para recibir órdenes
from cv_bridge import CvBridge
import cv2
import os

class DriverVideo(Node):
    def __init__(self):
        super().__init__('driver_video_file')
        
        # --- CONFIGURACIÓN ---
        self.video_path = '/home/administrador/Downloads/video_real.mp4'
        
        # Publicador de imagen
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # --- NUEVO: SUSCRIPTOR DE CONTROL ---
        # Escuchamos órdenes en el topic /control_video
        self.sub_control = self.create_subscription(
            String, 
            '/control_video', 
            self.control_callback, 
            10)

        # Estado inicial: False = PAUSA, True = REPRODUCIENDO
        self.reproduciendo = False 

        # Timer (30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'NO se pudo abrir el video: {self.video_path}')
        else:
            self.get_logger().info(f'Video cargado. Estado inicial: PAUSA. Esperando orden...')

    def control_callback(self, msg):
        orden = msg.data.lower().strip()
        
        if orden == 'play':
            self.reproduciendo = True
            self.get_logger().info('ORDEN RECIBIDA: PLAY ▶')
        elif orden == 'pause':
            self.reproduciendo = False
            self.get_logger().info('ORDEN RECIBIDA: PAUSE ⏸')
        elif orden == 'reset':
            self.reproduciendo = False
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info('ORDEN RECIBIDA: RESET ⏮ (Video rebobinado y pausado)')

    def timer_callback(self):
        # Si estamos en pausa, no hacemos nada
        if not self.reproduciendo:
            return

        ret, frame = self.cap.read()
        
        # Bucle infinito: Si termina, vuelve a empezar
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
        
        if ret:
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error conversion: {e}')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DriverVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
