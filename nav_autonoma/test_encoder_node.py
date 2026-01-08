#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

from library_opi.encoders import OpticalEncoder


class TestEncoderNode(Node):
    def __init__(self):
        super().__init__('test_encoder_node')
        
        # Inicializar encoders
        self.encoders = [
            OpticalEncoder(pin_a=4, pin_b=6, ppr=1000), # rueda frontal izquierda
            OpticalEncoder(pin_a=9, pin_b=10, ppr=1000), # rueda frontal derecha
            OpticalEncoder(pin_a=13, pin_b=15, ppr=1000), #rueda trasera derecha
            OpticalEncoder(pin_a=16, pin_b=18, ppr=1000) #reda frontal derecha
        ]
        
        # Publisher para datos de encoders
        self.encoder_pub = self.create_publisher(Float32MultiArray, 'encoder_data', 10)
        
        # Timer para publicar datos
        self.timer = self.create_timer(0.1, self.publish_encoder_data)  # 10Hz
        
        self.get_logger().info('Nodo de prueba de encoders inicializado')

    def publish_encoder_data(self):
        # Leer RPM de cada encoder
        #rpms = [enc.get_rpm() for enc in self.encoders]
        counters = [float(enc.counter) for enc in self.encoders] 
        # menos 1 porque la lectura esta invertiuda en las conexiones, 
        # es decir cuanbdo va de frente cuenta en negativos por lo que se puso el menos 1 para que cuente en positivos
        # Crear mensaje
        msg = Float32MultiArray()
        #msg.data = rpms + counters  # Primero 4 valores: RPMs, luego 4 valores: counters
        msg.data = counters
        self.encoder_pub.publish(msg)
        
        # Log cada 2 segundos para no saturar
        if int(time.time()) % 2 == 0:
            #self.get_logger().info(f'Encoders - RPM: {rpms} | Counters: {counters}')
            self.get_logger().info(f'Counters: {counters}')

    def destroy_node(self):
        for enc in self.encoders:
            enc.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()