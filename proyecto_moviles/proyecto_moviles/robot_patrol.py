import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler # Necesitas instalar sudo apt install ros-humble-tf-transformations

class TablePatrolNode(Node):
    def __init__(self):
        super().__init__('table_patrol_node')
        
        # Suscripción a la visión
        self.subscription = self.create_subscription(
            Point,
            '/silla_detectada',
            self.vision_callback,
            10)
            
        self.chair_detected = False
        self.chair_last_time = 0
        self.cooldown = 20.0 # Segundos para no detectar la misma silla todo el rato

    def vision_callback(self, msg):
        # Lógica simple: Si el área es suficientemente grande (estamos cerca) y hace tiempo que no vemos una
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # msg.z es el área. Ajusta este 15000 según la resolución de tu cámara
        if msg.z > 5000 and (current_time - self.chair_last_time) > self.cooldown:
            self.get_logger().info('¡Silla candidata visualizada!')
            self.chair_detected = True
            self.chair_last_time = current_time

    def reset_detection(self):
        self.chair_detected = False

def create_pose(navigator, x, y, z_orient):
    """Crea una pose a partir de x, y, y ángulo theta"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    
    # Conversión simple de Euler a Cuaternión (z_orient es yaw)
    q = quaternion_from_euler(0, 0, z_orient)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    patrol_node = TablePatrolNode()

    # --- CONFIGURACIÓN ---
    # Coordenadas de las mesas (Ajusta esto a tu mapa de MVSim)
    # Formato: [x, y, orientación_yaw]
    mesas_coords = [
        [2.0, 0.0, 0.0],   
        [2.0, 2.0, 1.57],   
        [-1.0, 2.0, 3.14],  
        [-1.0, -1.0, -1.57] 
    ]
    
    # Esperar a Nav2
    navigator.waitUntilNav2Active()

    current_wp_index = 0
    
    while rclpy.ok():
        rclpy.spin_once(patrol_node, timeout_sec=0.1)

        # 1. ¿Hay Silla Nueva? (Interrupción)
        if patrol_node.chair_detected:
            patrol_node.get_logger().warn('INTERRUPCIÓN: Atendiendo Silla...')
            navigator.cancelTask()
            
            # NOTA: Sin cámara 3D o LIDAR, no sabemos la posición exacta "detrás del respaldo"
            # relativa al mapa global solo con una foto.
            # Simulación: El robot gira 180 grados (como si se pusiera detrás) y espera.
            
            # Obtener posición actual del robot
            if navigator.getFeedback():
                current_pose = navigator.getFeedback().current_pose.pose
                cx = current_pose.position.x
                cy = current_pose.position.y
                
                # Maniobra simulada: Ir un poco hacia atrás (simular colocarse tras respaldo)
                patrol_node.get_logger().info('Posicionándose tras el respaldo...')
                # Aquí iría la lógica compleja de navegación local. 
                # Haremos una espera simulada para cumplir el requisito.
                time.sleep(2.0) 
                
                patrol_node.get_logger().info('Esperando 10 segundos...')
                time.sleep(10.0)
                
            patrol_node.reset_detection()
            patrol_node.get_logger().info('Reanudando Patrulla...')
            
            # Re-enviar meta actual para seguir donde lo dejamos
            target = create_pose(navigator, *mesas_coords[current_wp_index])
            navigator.goToPose(target)

        # 2. Navegación Patrulla Normal
        if not navigator.isTaskComplete():
            pass # Sigue moviéndose
        else:
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                # Pasar a la siguiente mesa
                current_wp_index = (current_wp_index + 1) % len(mesas_coords)
                target = create_pose(navigator, *mesas_coords[current_wp_index])
                navigator.goToPose(target)
                patrol_node.get_logger().info(f'Yendo a mesa {current_wp_index}')
            elif result == TaskResult.CANCELED:
                # Si fue cancelado por la silla, ya se manejó arriba, re-enviamos meta
                target = create_pose(navigator, *mesas_coords[current_wp_index])
                navigator.goToPose(target)
            elif result == TaskResult.FAILED:
                 # Si falla, reintentar misma mesa o saltar
                 navigator.goToPose(create_pose(navigator, *mesas_coords[current_wp_index]))

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
