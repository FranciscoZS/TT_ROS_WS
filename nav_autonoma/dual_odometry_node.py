#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050 import MPU6050
import math

class DualOdometryNode(Node):
    def __init__(self):
        super().__init__('dual_odometry_node')
        
        # Parámetros del robot
        self.wheel_radius = 0.05
        self.lx = 0.15 
        self.ly = 0.14
        
        # Inicializar Hardware (Nota el reduction_ratio=6)
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate() # Importante no mover el robot aquí
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10) # Fusión/Encoders
        self.imu_odom_pub = self.create_publisher(Odometry, '/odom_imu_raw', 10) # Solo IMU
        
        # Variables de estado (Odometría Cinemática)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.create_timer(0.05, self.update) # 20Hz
        self.last_time = self.get_clock().now()

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 1. Obtener datos IMU (Calcula su propia posición interna)
        imu_data = self.imu.update()
        
        # # Log cada 2 segundos para no saturar
        # if int(dt) % 2 == 0:
        #     #self.get_logger().info(f'Encoders - RPM: {rpms} | Counters: {counters}')
        #     self.get_logger().info(f"{self.enc_fl.counter} | {self.enc_fr.counter} | {self.enc_rl.counter} | {self.enc_rr.counter}")


        # 2. Obtener datos Encoders (Cinemática)
        w_fl = self.enc_fl.calculate_rpm() * 0.10472 # Rad/s
        w_fr = self.enc_fr.calculate_rpm() * 0.10472
        w_rl = self.enc_rl.calculate_rpm() * 0.10472
        w_rr = self.enc_rr.calculate_rpm() * 0.10472
        
        # Velocidad lineal ruedas (v = w * r)
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # Cinemática Mecanum
        vx = (-1)*(v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        
        # Fusión para rotación: Usamos Gyro (imu_data['w_z']) en vez de encoders
        # porque los encoders patinan mucho en rotación.
        vth = imu_data['w_z'] 
        
        # Integración Cinemática
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        ALPHA_VY = 0.95 # Filtro suave para Vy
        if not hasattr(self, 'vy_prev'): self.vy_prev = 0.0
        vy = ALPHA_VY * vy + (1.0 - ALPHA_VY) * self.vy_prev
        self.vy_prev = vy

        # --- PUBLICAR ODOMETRÍA CINEMÁTICA (ROBUSTA) ---
        self.publish_odom(self.odom_pub, self.x, self.y, self.th, vx, vy, vth, "odom")
        
        # --- PUBLICAR ODOMETRÍA IMU PURA (EXPERIMENTAL) ---
        # Nota: imu_data ya trae la integración hecha internamente en la clase MPU
        self.publish_odom(self.imu_odom_pub, imu_data['imu_x'], imu_data['imu_y'], imu_data['imu_theta'], 0, 0, 0, "odom_imu")
        
        self.last_time = current_time

    def publish_odom(self, publisher, x, y, th, vx, vy, vth, frame_id):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        
        # Quaternion simple
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)
        
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.angular.z = float(vth)
        
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()