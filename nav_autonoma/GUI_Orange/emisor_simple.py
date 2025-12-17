#!/usr/bin/env python3
"""
Nodo ROS2 simplificado para comunicación con Socket.IO
Solo maneja control manual del robot
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .GUI_sio import SocketIOClient

from .events_sio.sio_monitoreo import Monitoreo_RobotEvents

from .events_sio.sio_simple import ControlSimpleEvents

# Dirección del servidor Socket.IO
SERVIDOR_IP = "http://192.168.0.200:5000"

class EmisorSimpleNode(Node):
    def __init__(self):
        super().__init__("emisor_simple")
        
        self.get_logger().info("🚀 Inicializando Nodo Emisor Simplificado...")
        
        # Crear cliente Socket.IO
        self.socket_client = SocketIOClient(node=self, server_url=SERVIDOR_IP)
        
        # Registrar SOLO módulo de control manual
        self.get_logger().info("📦 Registrando control manual...")
        self.socket_client.add_module(ControlSimpleEvents())
        self.socket_client.add_module(Monitoreo_RobotEvents())
        
        # Iniciar conexión
        self.get_logger().info("🔌 Conectando a Socket.IO...")
        self.socket_client.start()
        
        # Timer para ping (mantener conexión viva)
        self.timer = self.create_timer(5.0, self.ping_callback)
        
        self.get_logger().info("✅ Emisor simple listo")
    
    def ping_callback(self):
        """Envía ping al servidor cada 5 segundos"""
        if self.socket_client.connected:
            self.socket_client.emit("ping_robot", {"status": "online"})
        else:
            self.get_logger().debug("Esperando conexión...")
    
    def destroy_node(self):
        """Cleanup al cerrar"""
        self.get_logger().info("🛑 Cerrando emisor...")
        
        if hasattr(self, 'socket_client'):
            self.socket_client.disconnect()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EmisorSimpleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️ Interrupción del usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
