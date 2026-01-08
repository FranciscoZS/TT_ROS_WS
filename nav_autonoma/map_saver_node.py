#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import sys
import signal

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        # Listas para guardar trayectoria
        self.path_enc_x = []
        self.path_enc_y = []
        
        self.path_imu_x = []
        self.path_imu_y = []
        
        # Suscribirse a ambas fuentes
        self.create_subscription(Odometry, '/odom', self.enc_callback, 10)
        self.create_subscription(Odometry, '/odom_imu_raw', self.imu_callback, 10)
        
        self.get_logger().info('Grabando trayectorias... Presiona Ctrl+C para guardar el mapa.')

    def enc_callback(self, msg):
        self.path_enc_x.append(msg.pose.pose.position.x)
        self.path_enc_y.append(msg.pose.pose.position.y)

    def imu_callback(self, msg):
        self.path_imu_x.append(msg.pose.pose.position.x)
        self.path_imu_y.append(msg.pose.pose.position.y)

    def save_plot(self):
        self.get_logger().info('Generando gráfica comparativa...')
        
        plt.figure(figsize=(10, 8))
        
        # Trayectoria Encoders (Azul)
        plt.plot(self.path_enc_x, self.path_enc_y, 'b-', label='Odometría (Encoders + Gyro)', linewidth=2)
        
        # Trayectoria IMU Pura (Rojo - Punteada)
        #plt.plot(self.path_imu_x, self.path_imu_y, 'r--', label='IMU Pura (Solo Acelerómetros)', alpha=0.7)
        
        plt.title(f"Comparación de Sensores\nMuestras: {len(self.path_enc_x)}")
        plt.xlabel("X [metros]")
        plt.ylabel("Y [metros]")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        filename = "trayectoria_comparativa.png"
        plt.savefig(filename)
        self.get_logger().info(f'Mapa guardado exitosamente como: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    
    # Manejo robusto de Ctrl+C para asegurar que se guarde la imagen
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()