#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class LandmarkPublisher(Node):
    def __init__(self):
        super().__init__('landmark_publisher')

        # --- 1. SUSCRIPCIÓN ---
        # Escuchamos el estado unificado ("ID:occupied" o "ID:free")
        self.subscription = self.create_subscription(
            String,
            '/chair_status_unified',
            self.listener_callback,
            10)

        # --- 2. PUBLICADOR ---
        # Enviamos los cubos de colores a RViz
        self.publisher_markers = self.create_publisher(MarkerArray, '/chair_markers', 10)
        
        # Timer para republicar constantemente (1 Hz) y que no desaparezcan de RViz
        self.timer = self.create_timer(1.0, self.publish_markers)

        # --- 3. CONFIGURACIÓN DEL MAPA ---
        # Coordenadas fijas de las mesas en tu mapa (X, Y)
        # IDs: 0, 1, 2, 3, 4
        self.chair_poses = {
            0: (-20.7, -10.0),
            1: (-21.5, -6.6),
            2: (-18.0, -8.86),
            3: (-18.3, -5.23),
            4: (-15.5, -7.11)
        }
        
        # Estado inicial de colores: Gris (Desconocido)
        # Formato RGBA: (0.5, 0.5, 0.5, 1.0)
        self.chair_colors = {id_silla: (0.5, 0.5, 0.5, 1.0) for id_silla in self.chair_poses}

    def listener_callback(self, msg):
        try:
            # Esperamos formato "ID:STATUS" (Ej: "1:free" o "1:occupied")
            data_str = msg.data.split(':')
            if len(data_str) != 2:
                return
            
            id_silla = int(data_str[0])
            status = data_str[1].strip() # Importante quitar espacios

            if id_silla in self.chair_colors:
                # --- LÓGICA DE COLORES (EN INGLÉS) ---
                if status == 'free':
                    # VERDE
                    self.chair_colors[id_silla] = (0.0, 1.0, 0.0, 1.0)
                elif status == 'occupied':
                    # ROJO
                    self.chair_colors[id_silla] = (1.0, 0.0, 0.0, 1.0)
                
                # Actualizar marcadores inmediatamente al recibir cambio
                self.publish_markers()
                
        except ValueError:
            pass

    def publish_markers(self):
        marker_array = MarkerArray()

        for id_silla, (x, y) in self.chair_poses.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "chairs"
            marker.id = id_silla
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Posición del mapa
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.5 # Altura (para que no esté en el suelo)

            # Orientación (Neutro)
            marker.pose.orientation.w = 1.0

            # Tamaño del cubo (0.5 metros)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            # Aplicar color actual
            r, g, b, a = self.chair_colors[id_silla]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker_array.markers.append(marker)

        self.publisher_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
