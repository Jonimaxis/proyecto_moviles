import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class VisualizadorMesas(Node):
    def __init__(self):
        super().__init__('visualizador_mesas')

        # Suscripción al topic que une ID y Estado
        self.subscription = self.create_subscription(
            String,
            '/silla_unificada',
            self.listener_callback,
            10)

        # Publicador de marcadores para RViz
        self.publisher_markers = self.create_publisher(MarkerArray, '/marcadores_sillas', 10)
        
        # Timer para republicar los marcadores constantemente (1 Hz)
        self.timer = self.create_timer(1.0, self.publicar_marcadores)

        # --- COORDENADAS REALES (X, Y) ---
        self.posiciones = {
            0: (-20.7, -10.0),
            1: (-21.5, -6.6),
            2: (-18.0, -8.86),
            3: (-18.3, -5.23), # Asumido -5.23 por la coma
            4: (-15.5, -7.11)
        }
        
        # Estado inicial de colores: Gris (0.5, 0.5, 0.5)
        # Formato: (R, G, B, Alpha)
        self.colores = {id_silla: (0.5, 0.5, 0.5, 1.0) for id_silla in self.posiciones}

    def listener_callback(self, msg):
        try:
            # Esperamos formato "ID:ESTADO" (ej: "1:libre")
            data_str = msg.data.split(':')
            if len(data_str) != 2:
                return
            
            id_silla = int(data_str[0])
            estado = data_str[1].strip()

            if id_silla in self.posiciones:
                if estado == 'libre':
                    # VERDE
                    self.colores[id_silla] = (0.0, 1.0, 0.0, 1.0)
                elif estado == 'ocupada':
                    # ROJO
                    self.colores[id_silla] = (1.0, 0.0, 0.0, 1.0)
                
                # Forzamos publicación inmediata al detectar cambio
                self.publicar_marcadores()
                
        except ValueError:
            pass

    def publicar_marcadores(self):
        marker_array = MarkerArray()

        for id_silla, (x, y) in self.posiciones.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sillas"
            marker.id = id_silla
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Posición exacta
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.25 # Altura sobre el suelo

            # Orientación
            marker.pose.orientation.w = 1.0

            # Tamaño del cubo (0.5 metros)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            # Color actual
            r, g, b, a = self.colores[id_silla]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker_array.markers.append(marker)

        self.publisher_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizadorMesas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
