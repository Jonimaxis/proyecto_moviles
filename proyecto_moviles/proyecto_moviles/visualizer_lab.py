import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ==========================================
# CONFIGURACIÓN: 1 ID ARUCO POR MESA
# ==========================================
# Hemos usado los IDs que tenías en tu código anterior (0, 6, 11, 12, 19).
# Si has cambiado las etiquetas físicas a 1, 2, 3, 4, 5, cambia los números aquí.

TABLE_CONFIG = {
    "Mesa_1": {
        "pos": [0.8, 2.0, 0.6],   # Posición X, Y, Z (sobre la mesa)
        "aruco_id": 0             # ID compartido por todas las sillas de esta mesa
    },
    "Mesa_2": {
        "pos": [-1.0, 0.3, 0.6],
        "aruco_id": 1
    },
    "Mesa_3": {
        "pos": [1.0, -0.15, 0.6],
        "aruco_id": 2
    },
    "Mesa_4": {
        "pos": [-1.0, -1.8, 0.6],
        "aruco_id": 3
    },
    "Mesa_5": {
        "pos": [1.2, -2.2, 0.6],
        "aruco_id": 4
    }
}

class TableVisualizer(Node):
    def __init__(self):
        super().__init__('table_visualizer_node')

        # Suscripción al estado detectado (viene de detectar_markers.py)
        # Espera mensajes formato "ID:ESTADO" (ej: "0:ocupada")
        self.create_subscription(String, '/estado_silla', self.silla_callback, 10)

        # Publicador de Marcadores para RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Diccionario para guardar el ÚLTIMO estado detectado de cada ID
        # Formato: { id_aruco: "ocupada" | "libre" }
        self.detected_states = {}
        
        # Timer (2 Hz)
        self.create_timer(0.5, self.publish_markers)

        self.get_logger().info("Visualizador de Mesas (Código Único) iniciado.")

    def silla_callback(self, msg):
        try:
            data = msg.data.split(':')
            if len(data) == 2:
                aruco_id = int(data[0])
                estado = data[1] # "ocupada", "libre", "unknown"
                
                # Guardamos directamente el estado asociado a ese ID de ArUco
                self.detected_states[aruco_id] = estado
        except ValueError:
            pass

    def get_color(self, aruco_id):
        """
        Devuelve el color (R, G, B, A) según el último estado conocido del ID.
        """
        estado = self.detected_states.get(aruco_id, "unknown")
        
        if estado == "ocupada":
            return (1.0, 0.0, 0.0, 1.0) # Rojo intenso
        elif estado == "libre":
            return (0.0, 1.0, 0.0, 1.0) # Verde intenso
        else:
            return (0.6, 0.6, 0.6, 0.5) # Gris transparente (Sin datos)

    def publish_markers(self):
        marker_array = MarkerArray()
        id_counter = 0 

        for name, info in TABLE_CONFIG.items():
            
            # --- MARCADOR DEL CUBO (La Mesa) ---
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tables_status"
            marker.id = id_counter
            id_counter += 1
            
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Posición fija definida en TABLE_CONFIG
            marker.pose.position.x = float(info["pos"][0])
            marker.pose.position.y = float(info["pos"][1])
            marker.pose.position.z = float(info["pos"][2])
            marker.pose.orientation.w = 1.0
            
            # Dimensiones del marcador
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.05 # Plano, como un mantel
            
            # Color según el estado del ID ArUco asociado
            r, g, b, a = self.get_color(info["aruco_id"])
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker_array.markers.append(marker)

            # --- MARCADOR DE TEXTO (Nombre de la Mesa) ---
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "tables_names"
            text.id = id_counter
            id_counter += 1
            
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(info["pos"][0])
            text.pose.position.y = float(info["pos"][1])
            text.pose.position.z = float(info["pos"][2]) + 0.3
            text.scale.z = 0.2
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
            text.text = f"{name} (ID {info['aruco_id']})"
            
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TableVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
