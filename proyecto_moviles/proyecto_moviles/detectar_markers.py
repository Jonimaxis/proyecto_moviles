#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS nodes
from sensor_msgs.msg import Image  # ROS Image message type
from geometry_msgs.msg import Twist  # ROS Twist message type
from cv_bridge import CvBridge  # Bridge between ROS and OpenCV images
from std_msgs.msg import Bool
import cv2  # OpenCV for image processing
import cv2.aruco as aruco  # ArUco marker detection library


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # CvBridge instance for converting ROS images to OpenCV
        self.bridge = CvBridge()
        # --- CONFIGURACIÓN DE LOS 5 ARUCOS ---
        # IDs que queremos vigilar
        self.target_ids = [0, 1, 2, 3, 4] 
        
        # Diccionario para guardar los publicadores (un topic por cada ID)
        self.aruco_publishers = {}

        # Crear un topic para cada ID automáticamente
        # Se llamarán: /aruco/id_0, /aruco/id_1, etc.
        for id_num in self.target_ids:
            topic_name = f'aruco/id_{id_num}'
            self.aruco_publishers[id_num] = self.create_publisher(Bool, topic_name, 10)
        
        self.get_logger().info(f"Vigilando Arucos IDs: {self.target_ids}")
        # -------------------------------------
        # Subscription to image data topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # Ensure this topic is correct
            self.listener_callback,
            10
        )

        # Publisher for robot velocity commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Log the initialization of the node
        self.get_logger().info("CameraNode initialized and subscribing to 'image_raw'.")

        # ArUco marker detection setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

    def generate_twist(self, linear, angular):
        """
        Helper function to create a Twist message for robot control.
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    def process_marker_position(self, marker_center_x, marker_center_y, img_center_x, img_center_y):
        """
        Determines the robot's movement based on marker position relative to image center.
        """
        linear_speed = 0.0
        angular_speed = 0.0

        # Adjust forward/backward motion based on vertical position
        if marker_center_y < img_center_y - 10:
            linear_speed = 0.5  # Move forward
            self.get_logger().info("Marker above center: moving forward.")
        elif marker_center_y > img_center_y + 10:
            linear_speed = -0.5  # Move backward
            self.get_logger().info("Marker below center: moving backward.")

        # Adjust rotational motion based on horizontal position
        if marker_center_x < img_center_x - 10:
            angular_speed = 0.5  # Rotate clockwise
            self.get_logger().info("Marker left of center: rotating clockwise.")
        elif marker_center_x > img_center_x + 10:
            angular_speed = -0.5  # Rotate counterclockwise
            self.get_logger().info("Marker right of center: rotating counterclockwise.")

        return linear_speed, angular_speed

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # 1. Preparar imagen y detector (Tu corrección de OpenCV 4.9)
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, aruco_params)
        
        # 2. Detectar
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        # 3. Procesar qué IDs hemos visto
        detected_list = []
        if ids is not None:
            detected_list = ids.flatten().tolist() # Convertir a lista simple [0, 2, ...]

        # 4. Publicar estado para CADA ID que vigilamos
        for target_id in self.target_ids:
            msg_bool = Bool()
            
            if target_id in detected_list:
                msg_bool.data = True
                # Opcional: Imprimir en consola solo si detectamos
                # self.get_logger().info(f"¡Aruco {target_id} DETECTADO!")
            else:
                msg_bool.data = False
            
            # Publicar en el topic correspondiente
            self.aruco_publishers[target_id].publish(msg_bool)

        # (Opcional) Mostrar imagen en pantalla si tienes interfaz gráfica
        # cv2.imshow("Camera", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    """
    Main function to initialize the node and start spinning.
    """
    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
