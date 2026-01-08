#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

from library_opi.mpu6050 import MPU6050

class TestMPUNode(Node):
    def __init__(self):
        super().__init__('test_mpu_node')
        
        # Inicializar MPU
        self.mpu = MPU6050(node=self)
        
        # Publisher para datos IMU
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        
        # Timer para publicar datos
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz
        
        self.get_logger().info('Nodo de prueba MPU6050 inicializado')

    def publish_imu_data(self):
        if self.mpu.read_all_data():
            imu_data = self.mpu.get_odometry_data()
            
            # Crear mensaje IMU
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            
            # Velocidad angular (solo eje Z)
            msg.angular_velocity.z = imu_data['angular_velocity_z'] * math.pi / 180.0  # a rad/s
            
            # Aceleración lineal
            msg.linear_acceleration.x = imu_data['accel_x'] * 9.81  # a m/s²
            msg.linear_acceleration.y = imu_data['accel_y'] * 9.81
            msg.linear_acceleration.z = 9.81  # Asumiendo gravedad
            
            # Orientación (solo yaw)
            from geometry_msgs.msg import Quaternion
            angle = imu_data['angle_z'] * math.pi / 180.0  # a rad
            msg.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(angle/2), w=math.cos(angle/2))
            
            self.imu_pub.publish(msg)
            
            # Log cada segundo
            if int(self.get_clock().now().nanoseconds / 1e9) % 1 == 0:
                self.get_logger().info(f'MPU - AngVel: {imu_data["angular_velocity_z"]:.2f} °/s | '
                                      f'Angle: {imu_data["angle_z"]:.2f} ° | '
                                      f'Accel: ({imu_data["accel_x"]:.2f}, {imu_data["accel_y"]:.2f}) g')

def main(args=None):
    rclpy.init(args=args)
    node = TestMPUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()