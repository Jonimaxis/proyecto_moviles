#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray 
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # FILTRO: Solo haremos caso a estos IDs (Mesas 1 a 5)
        self.target_ids = [0, 1, 2, 3, 4]
        
        # Publicamos la lista de IDs detectados (Ej: [0] o [1, 2])
        self.publisher_ids = self.create_publisher(Int32MultiArray, '/detected_aruco_ids', 10)
        
        # --- SUSCRIPTOR ---
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.listener_callback, 
            10)

        # --- CONFIGURACIÓN ARUCO ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        
        # Creamos el objeto detector (Compatible con OpenCV nuevo)
        self.detector_obj = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.get_logger().info("Nodo ArUco Detector iniciado.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error cv_bridge: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detectar marcadores
        corners, ids, _ = self.detector_obj.detectMarkers(gray)

        lista_simple = []   # Lista final de IDs válidos

        if ids is not None:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                
                # Solo procesamos si es una de nuestras mesas
                if marker_id in self.target_ids:
                    
                    # Añadir a la lista para publicar
                    lista_simple.append(int(marker_id))
                    
                    # --- VISUALIZACIÓN ---
                    # Dibujamos el cuadrado verde
                    c = marker_corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = c
                    
                    # Convertir a enteros para dibujar
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # Líneas del cuadrado
                    cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                    
                    # Escribir el ID
                    cv2.putText(cv_image, str(marker_id), (topLeft[0], topLeft[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- PUBLICACIÓN ---
        msg_ids = Int32MultiArray()
        msg_ids.data = lista_simple
        self.publisher_ids.publish(msg_ids)
        
        # Ver en pantalla (Solo PC 2)
        cv2.imshow("ArUco Detector", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
