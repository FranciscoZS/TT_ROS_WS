#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

from library_opi.pca9685_leds import SimpleRGBPCA9685

class TestLEDNode(Node):
    def __init__(self):
        super().__init__('test_led_node')
        
        # Inicializar controlador LED
        self.led_controller = SimpleRGBPCA9685(node=self)
        
        # Publisher para estado de LEDs (simulado)
        self.led_pub = self.create_publisher(Int32MultiArray, 'led_status', 10)
        
        # Timer para cambiar colores
        self.color_timer = self.create_timer(3.0, self.change_color)
        self.color_index = 0
        self.colors = [
            (100, 0, 0),   # Rojo
            (0, 100, 0),   # Verde
            (0, 0, 100),   # Azul
            (100, 100, 0), # Amarillo
            (0, 100, 100), # Cian
            (100, 0, 100), # Magenta
            (100, 100, 100) # Blanco
        ]
        
        self.get_logger().info('Nodo de prueba LEDs inicializado')

    def change_color(self):
        color = self.colors[self.color_index]
        self.led_controller.set_color(0, 1, 2, color[0], color[1], color[2])
        
        # Publicar estado (canales R, G, B)
        msg = Int32MultiArray()
        msg.data = list(color)
        self.led_pub.publish(msg)
        
        self.get_logger().info(f'LEDs cambiados a: {color}')
        
        # Siguiente color
        self.color_index = (self.color_index + 1) % len(self.colors)

    def destroy_node(self):
        # Apagar LEDs al salir
        self.led_controller.clean_color(0, 1, 2)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestLEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()