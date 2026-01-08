#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import time

from library_opi.sdc40_sensor import SCD40_SMBus

class TestSCD40Node(Node):
    def __init__(self):
        super().__init__('test_scd40_node')
        
        # Inicializar sensor
        self.sensor = SCD40_SMBus(node=self)
        
        # Iniciar mediciones periódicas
        self.sensor.start_periodic_measurement()
        
        # Publisher para datos ambientales
        self.env_pub = self.create_publisher(Float64MultiArray, 'environment_data', 10)
        #self.publish_environment_data()

        # Timer para publicar datos
        self.timer = self.create_timer(5.0, self.publish_environment_data)  # 0.2Hz
        


        self.get_logger().info('Nodo de prueba SCD40 inicializado')

    def publish_environment_data(self):
        if self.sensor.read_measurement():
            co2, temp, hum = self.sensor.get_measurement()
            self.get_logger().info(f'SCD40 - CO2: {co2} ppm | Temp: {temp:.2f} °C | Hum: {hum:.2f} %')
            """
            msg = String()
            msg.data = f"CD40 - CO2: {co2} ppm | Temp: {temp:.2f} °C | Hum: {hum:.2f} %"
            self.env_pub.publish(msg)
            """
            
            # Crear mensaje
            msg = Float64MultiArray()
            msg.data = [float(co2), float(temp), float(hum)]
            
            self.env_pub.publish(msg)
            
            # Log cada publicación
            self.get_logger().info(f'SCD40 - CO2: {co2} ppm | Temp: {temp:.2f} °C | Hum: {hum:.2f} %')
            

    def destroy_node(self):
        self.sensor.stop_periodic_measurement()
        self.sensor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestSCD40Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()