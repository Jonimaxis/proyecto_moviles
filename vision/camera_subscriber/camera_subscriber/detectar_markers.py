#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# Importamos Float para datos complejos e Int para ver solo los IDs
from std_msgs.msg import Float32MultiArray, Int32MultiArray 
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.target_ids = [0, 1, 2, 3, 4]
        

        #Envía [id] para ver en terminal qué aruco es
        self.publisher_ids = self.create_publisher(Int32MultiArray, '/aruco_ids', 10)
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.get_logger().info("Nodo ArUco iniciado (Doble publicación).")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        # Listas para guardar la información
        lista_compleja = [] # [id, x, y, id, x, y...]
        lista_simple = []   # [id, id...]

        if ids is not None:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                if marker_id in self.target_ids:
                    # Calcular centro
                    c = marker_corner.reshape((4, 2))
                    cX = int((c[0][0] + c[2][0]) / 2.0)
                    cY = int((c[0][1] + c[2][1]) / 2.0)
                    
                    # 1. Llenamos la lista compleja para YOLO
                    lista_compleja.extend([float(marker_id), float(cX), float(cY)])
                    
                    # 2. Llenamos la lista simple para el usuario (convertimos a int)
                    lista_simple.append(int(marker_id))
                    
                    # Dibujar
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # --- PUBLICACIÓN ---


        # publicar IDs Limpios (Int) -> Para ver en terminal
        msg_ids = Int32MultiArray()
        msg_ids.data = lista_simple
        self.publisher_ids.publish(msg_ids)
        
        # Visualizar
        cv2.imshow("ArUco Node", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
