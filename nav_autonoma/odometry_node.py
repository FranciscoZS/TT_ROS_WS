#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu
import math
import threading

# Importar tus librerías
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050 import MPU6050

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Parámetros (los ajustaremos después con tus valores exactos)
        self.declare_parameter('wheel_radius', 0.05)  # 5cm
        self.declare_parameter('wheel_base_x', 0.35)   # 30cm entre ruedas izquierda/derecha
        self.declare_parameter('wheel_base_y', 0.28)   # 28cm entre ruedas frontales/trasera 
        self.declare_parameter('encoder_ppr', 1000)

        
        # Inicializar hardware
        self.encoders = [
            OpticalEncoder(pin_a=4, pin_b=6, ppr=1000, node=self), # rueda frontal izquierda
            OpticalEncoder(pin_a=9, pin_b=10, ppr=1000, node=self), # rueda frontal derecha
            OpticalEncoder(pin_a=13, pin_b=15, ppr=1000, node=self), #rueda trasera derecha
            OpticalEncoder(pin_a=16, pin_b=18, ppr=1000, node=self) #reda frontal derecha
        ]
        
        self.imu = MPU6050(node=self)
        #El girocipoio esta posicoinado de tal forma que el eje x del mismo est apuntando a enfrente y el y a los costados

        # Calibrar IMU al inicio
        self.imu.calibrate_gyro_drift(3.0)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Variables de odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientación en radianes
        self.vx = 0.0     # Velocidad lineal X
        self.vy = 0.0     # Velocidad lineal Y  
        self.vtheta = 0.0 # Velocidad angular
        
        self.last_time = self.get_clock().now()
        self.odom_lock = threading.Lock()
        
        # Timer de actualización (50Hz)
        self.timer = self.create_timer(0.02, self.update_odometry)
        
        self.get_logger().info('Odometry Node inicializado')

    def update_odometry(self):
        """Actualizar odometría usando encoders + IMU"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
            
        try:
            # Leer IMU
            self.imu.read_all_data()
            imu_data = self.imu.get_odometry_data()
            
            # Leer encoders y calcular RPM
            wheel_rpms = []
            for encoder in self.encoders:
                rpm = encoder.get_rpm()
                wheel_rpms.append(rpm)
            
            # Calcular velocidades de ruedas (m/s)
            wheel_radius = self.get_parameter('wheel_radius').value
            wheel_velocities = [
                (rpm * 2 * math.pi * wheel_radius) / 60.0 for rpm in wheel_rpms
            ]
            
            # Cinemática directa para mecanum
            vx, vy, vtheta = self.mecanum_forward_kinematics(wheel_velocities)
            
            # Fusión sensorial: usar IMU para velocidad angular, encoders para lineal
            vtheta_fused = imu_data['angular_velocity_z'] * math.pi / 180.0  # Convertir a rad/s
            
            # Integrar para obtener posición
            with self.odom_lock:
                self.theta += vtheta_fused * dt
                # Normalizar ángulo entre -pi y pi
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                
                # Transformar velocidades a marco global
                self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
                self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
                
                self.vx = vx
                self.vy = vy
                self.vtheta = vtheta_fused
                
                # Publicar datos
                self.publish_odometry(current_time)
                self.publish_imu_data(current_time, imu_data)
                
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error en odometría: {e}')

    def mecanum_forward_kinematics(self, wheel_velocities):
        """
        Cinemática directa para ruedas mecanum
        wheel_velocities: [v_fl, v_fr, v_rl, v_rr] en m/s
        """
        L = self.get_parameter('wheel_base_x').value / 2.0  # Semi-distancia longitudinal
        W = self.get_parameter('wheel_base_y').value / 2.0  # Semi-distancia transversal
        
        # Matriz de transformación inversa
        vx = (wheel_velocities[0] + wheel_velocities[1] + 
              wheel_velocities[2] + wheel_velocities[3]) / 4.0
        
        vy = (-wheel_velocities[0] + wheel_velocities[1] + 
              wheel_velocities[2] - wheel_velocities[3]) / 4.0
        
        vtheta = (-wheel_velocities[0] + wheel_velocities[1] - 
                  wheel_velocities[2] + wheel_velocities[3]) / (4.0 * (L + W))
        
        return vx, vy, vtheta

    def publish_odometry(self, timestamp):
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Posición
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Orientación (quaternion desde ángulo theta)
        msg.pose.pose.orientation = self.angle_to_quaternion(self.theta)
        
        # Velocidad
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.vtheta
        
        # Covarianzas (valores estimados, ajustar según precisión)
        msg.pose.covariance[0] = 0.1  # x
        msg.pose.covariance[7] = 0.1  # y
        msg.pose.covariance[35] = 0.1 # theta
        
        self.odom_pub.publish(msg)

    def publish_imu_data(self, timestamp, imu_data):
        msg = Imu()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Velocidad angular
        msg.angular_velocity.z = imu_data['angular_velocity_z'] * math.pi / 180.0  # rad/s
        
        # Aceleración lineal
        msg.linear_acceleration.x = imu_data['accel_x'] * 9.81  # m/s²
        msg.linear_acceleration.y = imu_data['accel_y'] * 9.81  # m/s²
        msg.linear_acceleration.z = 9.81  # Gravedad
        
        # Orientación (solo yaw)
        msg.orientation = self.angle_to_quaternion(imu_data['angle_z'] * math.pi / 180.0)
        
        self.imu_pub.publish(msg)

    def angle_to_quaternion(self, angle):
        """Convertir ángulo a quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(angle / 2.0)
        q.w = math.cos(angle / 2.0)
        return q

    def destroy_node(self):
        """Cleanup al cerrar el nodo"""
        for encoder in self.encoders:
            encoder.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()