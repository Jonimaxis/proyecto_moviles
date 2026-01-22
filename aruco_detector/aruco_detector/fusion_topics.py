#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_topics')

        # --- 1. SUSCRIPTORES ---
        
        # A. IDs detectados por ArUco (Int32MultiArray)
        self.sub_ids = self.create_subscription(
            Int32MultiArray,
            '/detected_aruco_ids',
            self.ids_callback,
            10)
        
        # B. Estado detectado por YOLO (String: "occupied" / "free")
        self.sub_status = self.create_subscription(
            String,
            '/chair_status',
            self.status_callback,
            10)

        # --- 2. PUBLICADOR ---
        # Publicará la combinación "ID:ESTADO"
        # Topic estandarizado: /chair_status_unified
        self.publisher_ = self.create_publisher(String, '/chair_status_unified', 10)

        # Variable para guardar el último estado conocido por YOLO
        self.current_status = "unknown"

    def status_callback(self, msg):
        """
        Simplemente actualizamos la variable cada vez que YOLO dice algo.
        No publicamos aquí para no saturar si no hay ArUco visible.
        """
        self.current_status = msg.data

    def ids_callback(self, msg):
        """
        Este es el disparador principal. Si vemos IDs, publicamos su estado.
        """
        detected_ids = msg.data
        
        # Si la lista de IDs está vacía, no hacemos nada
        if not detected_ids:
            return

        # Recorremos todos los IDs visibles (por si ve la mesa 1 y la 2 a la vez)
        for chair_id in detected_ids:
            
            # Crear el mensaje unificado
            msg_out = String()
            
            # Formato estándar: "ID:STATUS" (Ej: "5:occupied")
            msg_out.data = f"{chair_id}:{self.current_status}"
            
            self.publisher_.publish(msg_out)
            
            # Log para depuración
            # self.get_logger().info(f'Fusión Publicada: {msg_out.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
