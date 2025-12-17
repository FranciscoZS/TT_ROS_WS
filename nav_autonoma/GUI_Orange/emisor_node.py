import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .GUI_sio import SocketIOClient
from .events_sio.sio_monitoreo import Monitoreo_RobotEvents
from .events_sio.sio_control_manual import ControlManual_RobotEvents
from nav_autonoma.AudioNode import AudioManagerNode

from nav_autonoma.configRobot import Config



class Emisor_Node(Node):
    def __init__(self):
        super().__init__("socket_emisor")
        
        self.get_logger().info("🚀 Inicializando Nodo Emisor...")
        
        # 1. Crear el cliente 
        self.socket_client = SocketIOClient(node=self, server_url=Config.Emisor_UDP_ADDR)
        


        # 2. Registrar TODOS los módulos de eventos 
        self.get_logger().info("📦 Registrando módulos de eventos...")
        self.socket_client.add_module(Monitoreo_RobotEvents())
        self.socket_client.add_module(ControlManual_RobotEvents())
        #self.socket_client.add_module(AudioManagerNode())

        # 3. Inicia la conexión
        self.get_logger().info("🔌 Iniciando conexión Socket.IO...")
        self.socket_client.start()
        
        # 4. Timer para ping periódico
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info("Nodo Emisor listo")
    
    def timer_callback(self):
        """Envía un ping periódico al servidor"""
        if self.socket_client.connected:
            self.socket_client.emit("ping_robot", {"msg": "robot activo"})
        else:
            self.get_logger().debug("Esperando conexión para enviar ping...")
    
    def destroy_node(self):
        """Limpieza al destruir el nodo"""
        self.get_logger().info("Deteniendo Nodo Emisor...")
        
        # Desconectar Socket.IO limpiamente
        if hasattr(self, 'socket_client'):
            self.socket_client.disconnect()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Emisor_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️ Interrupción del usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()