#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from library_opi.ads1115 import ADS1115

class TestADS1115SimpleNode(Node):
    def __init__(self):
        super().__init__('test_ads1115_simple_node')
        
        # Inicializar ADS1115 (usando ganancia 6.144V para medir 5V)
        self.ads = ADS1115(gain=0x00, node=self)  # PGA_6_144V
        
        if not self.ads.is_initialized():
            self.get_logger().error("No se pudo inicializar ADS1115. Apagando nodo.")
            return
        
        # Publisher para datos ADC (canales 0 y 1)
        self.adc_pub = self.create_publisher(Float32MultiArray, 'adc_voltage', 10)
        
        # Timer para publicar datos (10Hz)
        self.timer = self.create_timer(0.1, self.publish_adc_data)
        
        self.get_logger().info('Nodo ADS1115 Simple inicializado - Publicando canales 0 y 1')

    def publish_adc_data(self):
        if self.ads.is_initialized():
            adc_data,_,_ = self.ads.read_channels_0_1()
            
            # Crear mensaje con voltajes de los canales 0 y 1
            msg = Float32MultiArray()
            msg.data = [
                adc_data['channel_0']['voltage'],
                adc_data['channel_1']['voltage']
            ]
            
            self.adc_pub.publish(msg)
            
            # Log cada segundo (no en cada publicación para no saturar)
            current_time = self.get_clock().now()
            if current_time.nanoseconds % 1000000000 < 100000000:  # ~1 vez por segundo
                self.get_logger().info(
                    f'ADC - CH0: {adc_data["channel_0"]["voltage"]:.3f}V | '
                    f'CH1: {adc_data["channel_1"]["voltage"]:.3f}V'
                )

    def destroy_node(self):
        if self.ads.is_initialized():
            self.ads.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestADS1115SimpleNode()
    
    # Verificar si se inicializó correctamente
    if node.ads.is_initialized():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.get_logger().error("No se pudo inicializar el ADS1115")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()