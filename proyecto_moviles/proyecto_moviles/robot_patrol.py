import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- FUNCIÓN AUXILIAR (Para no depender de tf_transformations) ---
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convierte ángulos de Euler (roll, pitch, yaw) a cuaternión [x, y, z, w]
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# --- CONFIGURACIÓN DEL MAPA ---
# ==========================================
# CONFIGURACIÓN DEL MAPA (ADAPTADO A LÍMITES REALES)
# Límites Mapa: X[-2.3, 2.3], Y[-3.3, 3.3]
# ==========================================

LAB_MAP = {
    # --- MESA 1 (Arriba Derecha) ---
    "Mesa_1_Norte": { 
        # Antes Y=3.2 (muy al borde). Bajamos a 2.7 para dejar sitio.
        "coords_visita": [0.8, 2.7, -1.57], 
        "sillas": { 0: {"estado": "unknown", "marker_pos": [0.8, 2.4, 0.5]} }
    },
    "Mesa_1_Sur": { 
        "coords_visita": [0.8, 1.0, 1.57],  
        "sillas": { 1: {"estado": "unknown", "marker_pos": [0.8, 1.4, 0.5]} }
    },
    "Mesa_1_Este": { 
        # Antes X=2.0. Pared en 2.3. Ajustado a 1.7
        "coords_visita": [1.7, 2.0, 3.14],  
        "sillas": { 2: {"estado": "unknown", "marker_pos": [1.4, 2.0, 0.5]} }
    },
    "Mesa_1_Oeste": { 
        "coords_visita": [-0.2, 2.0, 0.0],  
        "sillas": { 3: {"estado": "unknown", "marker_pos": [0.2, 2.0, 0.5]} }
    },

    # --- MESA 2 (Arriba Izquierda) ---
    "Mesa_2_Norte": {
        "coords_visita": [-1.0, 1.4, -1.57],
        "sillas": { 4: {"estado": "unknown", "marker_pos": [-1.0, 0.9, 0.5]} }
    },
    "Mesa_2_Sur": {
        "coords_visita": [-1.0, -0.8, 1.57],
        "sillas": { 5: {"estado": "unknown", "marker_pos": [-1.0, -0.3, 0.5]} }
    },
    "Mesa_2_Este": { 
        "coords_visita": [-0.4, 0.3, 3.14],
        "sillas": { 6: {"estado": "unknown", "marker_pos": [-0.6, 0.3, 0.5]} }
    },
    "Mesa_2_Oeste": {
        # Antes X=-2.2. Pared en -2.3. CRÍTICO. Movido a -1.7
        "coords_visita": [-1.7, 0.3, 0.0],
        "sillas": { 7: {"estado": "unknown", "marker_pos": [-1.4, 0.3, 0.5]} }
    },

    # --- MESA 3 (Centro Derecha) ---
    "Mesa_3_Norte": {
        "coords_visita": [1.0, 0.9, -1.57],
        "sillas": { 8: {"estado": "unknown", "marker_pos": [1.0, 0.45, 0.5]} }
    },
    "Mesa_3_Sur": {
        "coords_visita": [1.0, -1.2, 1.57],
        "sillas": { 9: {"estado": "unknown", "marker_pos": [1.0, -0.75, 0.5]} }
    },
    "Mesa_3_Este": {
        # Antes X=2.2. Pared en 2.3. Movido a 1.7
        "coords_visita": [1.7, -0.15, 3.14],
        "sillas": { 10: {"estado": "unknown", "marker_pos": [1.4, -0.15, 0.5]} }
    },
    "Mesa_3_Oeste": {
        "coords_visita": [0.0, -0.15, 0.0],
        "sillas": { 11: {"estado": "unknown", "marker_pos": [0.4, -0.15, 0.5]} }
    },

    # --- MESA 4 (Abajo Izquierda) ---
    "Mesa_4_Norte": {
        "coords_visita": [-1.0, -0.7, -1.57],
        "sillas": { 12: {"estado": "unknown", "marker_pos": [-1.0, -1.2, 0.5]} }
    },
    "Mesa_4_Sur": {
        # Antes Y=-3.0. Pared en -3.3. Justo, pero seguro mover a -2.7
        "coords_visita": [-1.0, -2.7, 1.57],
        "sillas": { 13: {"estado": "unknown", "marker_pos": [-1.0, -2.4, 0.5]} }
    },
    "Mesa_4_Este": {
        "coords_visita": [-0.4, -1.8, 3.14],
        "sillas": { 14: {"estado": "unknown", "marker_pos": [-0.6, -1.8, 0.5]} }
    },
    "Mesa_4_Oeste": {
        # Antes X=-2.2. Pared -2.3. Movido a -1.7
        "coords_visita": [-1.7, -1.8, 0.0],
        "sillas": { 15: {"estado": "unknown", "marker_pos": [-1.4, -1.8, 0.5]} }
    },

    # --- MESA 5 (Abajo Derecha) ---
    "Mesa_5_Norte": {
        "coords_visita": [1.2, -1.1, -1.57],
        "sillas": { 16: {"estado": "unknown", "marker_pos": [1.2, -1.6, 0.5]} }
    },
    "Mesa_5_Sur": {
        # Antes Y=-3.4. ¡FUERA DE MAPA! Movido a -2.8
        "coords_visita": [1.2, -2.8, 1.57],
        "sillas": { 17: {"estado": "unknown", "marker_pos": [1.2, -2.5, 0.5]} }
    },
    "Mesa_5_Este": {
        # Antes X=2.4. ¡FUERA DE MAPA! Movido a 1.8
        "coords_visita": [1.8, -2.2, 3.14],
        "sillas": { 18: {"estado": "unknown", "marker_pos": [1.6, -2.2, 0.5]} }
    },
    "Mesa_5_Oeste": {
        "coords_visita": [0.2, -2.2, 0.0],
        "sillas": { 19: {"estado": "unknown", "marker_pos": [0.6, -2.2, 0.5]} }
    },
}

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # Suscripción al nodo de visión
        self.create_subscription(String, '/estado_silla', self.vision_callback, 10)
            
        # Publicador de Marcadores (RViz)
        self.marker_pub = self.create_publisher(MarkerArray, '/sillas_markers', 10)
        
        # Timer para refrescar marcadores
        self.create_timer(1.0, self.publish_markers)

    def vision_callback(self, msg):
        try:
            data = msg.data.split(':')
            if len(data) != 2: return
            
            silla_id = int(data[0])
            estado = data[1]
            
            for mesa in LAB_MAP.values():
                if silla_id in mesa["sillas"]:
                    if mesa["sillas"][silla_id]["estado"] != estado:
                        mesa["sillas"][silla_id]["estado"] = estado
                        self.get_logger().info(f"Silla {silla_id} marcada como {estado}")
                    break
        except ValueError:
            pass

    def publish_markers(self):
        marker_array = MarkerArray()
        for mesa_data in LAB_MAP.values():
            for id_silla, info in mesa_data["sillas"].items():
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "estado_sillas"
                marker.id = id_silla
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(info["marker_pos"][0])
                marker.pose.position.y = float(info["marker_pos"][1])
                marker.pose.position.z = float(info["marker_pos"][2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
                marker.color.a = 1.0 
                
                estado = info["estado"]
                if estado == "libre":
                    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # Verde
                elif estado == "ocupada":
                    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # Rojo
                else:
                    marker.color.r = 0.7; marker.color.g = 0.7; marker.color.b = 0.7 # Gris
                
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    patrol_node = PatrolNode()
    
    print("Esperando a Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 Activo. Iniciando patrulla.")

    while rclpy.ok():
        for nombre_mesa, datos in LAB_MAP.items():
            
            # 1. Configurar meta
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            
            bx, by, byaw = datos["coords_visita"]
            goal_pose.pose.position.x = bx
            goal_pose.pose.position.y = by
            
            q = quaternion_from_euler(0, 0, byaw) # Usamos nuestra función manual
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
            
            # 2. Navegar
            patrol_node.get_logger().info(f"Yendo a {nombre_mesa}...")
            navigator.goToPose(goal_pose)
            
            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol_node, timeout_sec=0.1)
                
            # 3. Comprobar llegada y Escanear
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                patrol_node.get_logger().info(f"Llegada. Escaneando...")
                start_time = time.time()
                while time.time() - start_time < 5.0:
                    rclpy.spin_once(patrol_node, timeout_sec=0.1)
            else:
                patrol_node.get_logger().warn("Fallo en la navegación. Saltando punto.")

    navigator.lifecycleShutdown()
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
