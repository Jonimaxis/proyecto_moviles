#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os

class VideoController(Node):
    def __init__(self):
        super().__init__('video_controller')
        
        # --- 1. PARÁMETRO DE RUTA DE VIDEO ---
        # Esto permite cambiar el video sin tocar el código usando --ros-args -p video_path:="ruta"
        default_path = '/home/administrador/Downloads/video_real.mp4'
        self.declare_parameter('video_path', default_path)
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        
        # --- 2. PUBLICADOR ---
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # --- 3. SUSCRIPTOR DE CONTROL ---
        # Antes '/control_video', ahora '/video_control'
        self.sub_control = self.create_subscription(
            String, 
            '/video_control', 
            self.control_callback, 
            10)

        # Estado inicial: False = PAUSA, True = REPRODUCIENDO
        self.is_playing = False 

        # Timer (Aprox 30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.cv_bridge = CvBridge()
        
        # Intentar abrir video
        if not os.path.exists(self.video_path):
             self.get_logger().error(f'ARCHIVO NO ENCONTRADO: {self.video_path}')
        
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Error abriendo video: {self.video_path}')
        else:
            self.get_logger().info(f'Video cargado: {self.video_path}')
            self.get_logger().info('Estado inicial: PAUSA. Esperando comando en /video_control (play/pause/reset)')

    def control_callback(self, msg):
        command = msg.data.lower().strip()
        
        if command == 'play':
            self.is_playing = True
            self.get_logger().info('COMMAND: PLAY ▶')
        elif command == 'pause':
            self.is_playing = False
            self.get_logger().info('COMMAND: PAUSE ⏸')
        elif command == 'reset':
            self.is_playing = False
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info('COMMAND: RESET ⏮')

    def timer_callback(self):
        if not self.is_playing:
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
    node = VideoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
