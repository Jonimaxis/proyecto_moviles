import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

class UnificadorNode(Node):
    def __init__(self):
        super().__init__('unificador_node')

        # 1. Suscriptores
        # Se suscribe a los IDs (Int32MultiArray)
        self.sub_ids = self.create_subscription(
            Int32MultiArray,
            '/aruco_ids',
            self.ids_callback,
            10)
        
        # Se suscribe al estado (String)
        self.sub_estado = self.create_subscription(
            String,
            '/estado_silla',
            self.estado_callback,
            10)

        # 2. Publicador
        # Publicará el resultado combinado en un nuevo topic
        self.publisher_ = self.create_publisher(String, '/silla_unificada', 10)

        # Variables para guardar los últimos datos recibidos
        self.ultimo_id = None
        self.ultimo_estado = "desconocido"

    def ids_callback(self, msg):
        # Verificamos si la lista de IDs no está vacía
        if msg.data:
            self.ultimo_id = msg.data[0] # Tomamos el primer ID detectado
            self.publicar_combinacion()

    def estado_callback(self, msg):
        self.ultimo_estado = msg.data
        self.publicar_combinacion()

    def publicar_combinacion(self):
        # Solo publicamos si ya hemos recibido al menos un ID
        if self.ultimo_id is not None:
            msg = String()
            # Aquí creamos el formato "ID:ESTADO" (ej: "1:libre")
            msg.data = f"{self.ultimo_id}:{self.ultimo_estado}"
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publicando: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = UnificadorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
