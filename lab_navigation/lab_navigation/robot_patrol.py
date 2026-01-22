import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- FUNCIÓN AUXILIAR ---
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convierte ángulos de Euler (roll, pitch, yaw) a cuaternión [x, y, z, w]
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# ==========================================
# CONFIGURACIÓN DE MAPAS
# ==========================================

# 1. Mapa Simulador
LAB_MAP_SIM = {
    "Mesa_1": {"coords_visita": [0.8, 2.9, -1.57]},
    "Mesa_2": {"coords_visita": [0.0, 0.3, 3.14]},
    "Mesa_3": {"coords_visita": [-0.1, -0.15, 0.0]},
    "Mesa_4": {"coords_visita": [-1.0, -0.7, -1.57]},
    "Mesa_5": {"coords_visita": [0.0, -2.2, 0.0]},
}

# 2. Mapa Real (Tus coordenadas capturadas con RViz)
LAB_MAP_REAL = {
    "Punto_A": {"coords_visita": [-1.28196, 2.42512, -0.525745, 0.850642]},
    "Punto_B": {"coords_visita": [0.437005, 3.5021, 0.985641, 0.168854]},
    "Punto_C": {"coords_visita": [1.31297, 4.13313, -0.56754, 0.823346]},
    "Punto_D": {"coords_visita": [2.78601, 4.05749, 0.849627, 0.527383]},
    "Punto_E": {"coords_visita": [2.62247, 4.07205, -0.0722217, 0.997389]}
}

# --- SELECCIÓN DEL MAPA ---
# Cambiar dependiendo del entorno.
MAPA_ACTUAL = LAB_MAP_REAL 

class PatrolNode(Node):
    def __init__(self):
        super().__init__('robot_patrol')

def main():
    rclpy.init()
    
    # Inicializamos el navegador de Nav2
    navigator = BasicNavigator()
    patrol_node = PatrolNode()
    
    print("Esperando a Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 Activo. Iniciando patrulla.")

    # Bucle infinito de patrulla
    while rclpy.ok():
        for nombre_mesa, datos in MAPA_ACTUAL.items():
            
            # --- 1. Preparar el objetivo (Goal) ---
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            
            coords = datos["coords_visita"]
            
            # Lógica para distinguir coordenadas de Simulador (x,y,yaw) vs Real (x,y,z,w)
            if len(coords) == 3:
                # SIMULACION: Convertir Euler a Cuaternion
                bx, by, byaw = coords
                goal_pose.pose.position.x = float(bx)
                goal_pose.pose.position.y = float(by)
                q = quaternion_from_euler(0, 0, byaw)
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]
                
            elif len(coords) == 4:
                # REAL: Usar Cuaternion directo
                goal_pose.pose.position.x = float(coords[0])
                goal_pose.pose.position.y = float(coords[1])
                goal_pose.pose.orientation.z = float(coords[2])
                goal_pose.pose.orientation.w = float(coords[3])
            
            # --- 2. Enviar al robot ---
            patrol_node.get_logger().info(f"Yendo a {nombre_mesa}...")
            navigator.goToPose(goal_pose)
            
            # Esperar mientras se mueve
            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol_node, timeout_sec=0.1)
                
            # --- 3. Llegada y Pausa ---
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                patrol_node.get_logger().info(f"Llegada a {nombre_mesa}. Esperando detección visual...")
                
                # PAUSA CRÍTICA:
                # Damos 5 segundos al robot quieto para que el OTRO PC (Visión)
                # tenga tiempo de ver el ArUco y la persona y actualizar RViz.
                time.sleep(5.0) 
            else:
                patrol_node.get_logger().warn(f"Fallo al ir a {nombre_mesa}. Saltando punto.")

    navigator.lifecycleShutdown()
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
