#!/usr/bin/env python3
"""
Odometría DIFERENCIAL para robot mecanum usando solo 2 encoders (ruedas traseras)
Modelo simplificado para mejor precisión
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
import math
import numpy as np
from collections import deque
import time

class DifferentialOdometry2Encoders(Node):
    def __init__(self):
        super().__init__('differential_odometry_2encoders')
        
        # ========== PARÁMETROS FÍSICOS ==========
        self.declare_parameter('wheel_radius', 0.05)      # Radio de rueda (m)
        self.declare_parameter('wheel_base', 0.30)        # Distancia entre ruedas (m)
        self.declare_parameter('update_rate', 100.0)      # Hz
        self.declare_parameter('encoder_correction', 1.0) # Factor de corrección
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        update_rate = self.get_parameter('update_rate').value
        self.encoder_correction = self.get_parameter('encoder_correction').value
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando encoders diferenciales (2 ruedas)...")
        
        # SOLO 2 ENCODERS: ruedas traseras izquierda y derecha
        # Asegúrate de que estos sean los pines correctos para tus encoders traseros
        self.enc_left = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)   # RL
        self.enc_right = OpticalEncoder(16, 18, reduction_ratio=6, invert=True)   # RR (invertido)
        
        # Resetear contadores iniciales
        # self.enc_left.reset()
        # self.enc_right.reset()
        # time.sleep(0.1)
        
        # IMU
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=3.0)
        
        # ========== VARIABLES DE ESTADO ==========
        self.x = 0.0          # Posición X (m)
        self.y = 0.0          # Posición Y (m)
        self.theta = 0.0      # Orientación (rad)
        
        # ========== HISTORIAL PARA FILTRADO ==========
        self.velocity_buffer = deque(maxlen=5)
        self.omega_buffer = deque(maxlen=5)
        self.gyro_buffer = deque(maxlen=3)
        
        # ========== CONTADORES DE PULSOS ==========
        self.last_left_pulses = 0
        self.last_right_pulses = 0
        self.last_time = self.get_clock().now()
        
        # ========== DIAGNÓSTICO ==========
        self.total_distance = 0.0
        self.pulse_counts = {'left': 0, 'right': 0}
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.vel_pub = self.create_publisher(Twist, '/wheel_velocities', 10)
        self.diagnostics_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        
        self.get_logger().info(f"""
        ✅ Odometría DIFERENCIAL (2 encoders) iniciada:
           - Radio rueda: {self.wheel_radius} m
           - Distancia entre ruedas: {self.wheel_base} m
           - Frecuencia: {update_rate} Hz
           - Factor corrección: {self.encoder_correction}
        """)
    
    def calculate_wheel_velocities(self):
        """Calcula velocidades de las ruedas en m/s"""
        # Obtener RPM
        rpm_left = self.enc_left.calculate_rpm()
        rpm_right = self.enc_right.calculate_rpm()
        
        # DEBUG: Mostrar RPM periódicamente
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
            if self.debug_counter % 50 == 0:
                self.get_logger().info(f"📊 RPM: L={rpm_left:.1f}, R={rpm_right:.1f}")
        else:
            self.debug_counter = 0
        
        # Convertir RPM a rad/s: 1 RPM = 2π/60 rad/s
        w_left = rpm_left * 0.10472
        w_right = rpm_right * 0.10472
        
        # Velocidad lineal de cada rueda (m/s)
        v_left = w_left * self.wheel_radius * self.encoder_correction
        v_right = w_right * self.wheel_radius * self.encoder_correction
        
        return v_left, v_right
    
    def differential_kinematics(self, v_left, v_right):
        """
        Cinemática diferencial:
        v = (v_right + v_left) / 2      # Velocidad lineal del robot
        ω = (v_right - v_left) / L      # Velocidad angular del robot
        """
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        return v, omega
    
    def update_position(self, v, omega, dt):
        """
        Actualiza posición usando modelo diferencial
        """
        if abs(omega) < 0.001:  # Movimiento esencialmente recto
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:  # Movimiento curvilíneo
            # Radio de curvatura
            R = v / omega if abs(omega) > 0.001 else float('inf')
            
            # Cálculo exacto de posición para movimiento circular
            self.x += R * (math.sin(self.theta + omega * dt) - math.sin(self.theta))
            self.y += -R * (math.cos(self.theta + omega * dt) - math.cos(self.theta))
            self.theta += omega * dt
        
        # Normalizar ángulo a [-π, π]
        self.theta = self.normalize_angle(self.theta)
        
        # Acumular distancia total
        self.total_distance += abs(v) * dt
    
    def normalize_angle(self, angle):
        """Normaliza ángulo a [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def fuse_with_imu(self, omega_enc, omega_gyro):
        """
        Fusión simple entre giroscopio y odometría
        """
        # Peso mayor al gyro (0.8) durante rotaciones rápidas
        if abs(omega_gyro) > 0.5:  # rad/s
            alpha = 0.8  # Más confianza en el gyro
        else:
            alpha = 0.3  # Balance normal
        
        omega_fused = alpha * omega_gyro + (1 - alpha) * omega_enc
        
        return omega_fused
    
    def update(self):
        """Ciclo principal de actualización"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.2:  # Reset si hay retraso grande
            dt = 0.01
            self.get_logger().warn(f"⚠️  Retraso en odometría: {dt:.3f}s")
        
        # ========== 1. OBTENER DATOS IMU ==========
        imu_data = self.imu.update()
        omega_gyro_raw = imu_data['w_z']
        
        # Filtrar gyro
        self.gyro_buffer.append(omega_gyro_raw)
        omega_gyro = np.median(list(self.gyro_buffer))
        
        # ========== 2. OBTENER VELOCIDADES DE RUEDAS ==========
        v_left, v_right = self.calculate_wheel_velocities()
        
        # ========== 3. CINEMÁTICA DIFERENCIAL ==========
        v, omega_enc = self.differential_kinematics(v_left, v_right)
        
        # ========== 4. FUSIONAR CON GYRO ==========
        omega_fused = self.fuse_with_imu(omega_enc, omega_gyro)
        
        # ========== 5. ACTUALIZAR POSICIÓN ==========
        self.update_position(v, omega_fused, dt)
        
        # ========== 6. PUBLICAR DATOS ==========
        self.publish_odometry(current_time, v, omega_fused)
        self.publish_diagnostics(v_left, v_right, v, omega_enc, omega_gyro, omega_fused)
        
        # ========== 7. LOG PERIÓDICO ==========
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter % 100 == 0:
                self.get_logger().info(
                    f"📍 Pos: ({self.x:.3f}, {self.y:.3f}) "
                    f"θ: {math.degrees(self.theta):5.1f}° | "
                    f"V: {v:.3f} m/s | "
                    f"ω: {math.degrees(omega_fused):5.1f}°/s | "
                    f"Dist: {self.total_distance:.2f}m"
                )
        else:
            self.log_counter = 0
        
        self.last_time = current_time
    
    def publish_odometry(self, timestamp, v, omega):
        """Publica mensaje de odometría ROS2"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # Posición
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        
        # Orientación (cuaternión)
        theta = self.theta
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Covarianza (valores estimados)
        # Pequeña incertidumbre en posición, mayor en orientación
        msg.pose.covariance[0] = 0.01   # x
        msg.pose.covariance[7] = 0.01   # y
        msg.pose.covariance[14] = 0.001 # z
        msg.pose.covariance[21] = 0.001 # rot x
        msg.pose.covariance[28] = 0.001 # rot y
        msg.pose.covariance[35] = 0.02  # rot z (mayor incertidumbre en yaw)
        
        # Velocidad
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.linear.y = 0.0  # Robot diferencial no tiene velocidad lateral
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = float(omega)
        
        self.odom_pub.publish(msg)
    
    def publish_diagnostics(self, v_left, v_right, v, omega_enc, omega_gyro, omega_fused):
        """Publica datos de diagnóstico"""
        # Publicar velocidades de ruedas
        wheel_msg = Twist()
        wheel_msg.linear.x = v_left
        wheel_msg.linear.y = v_right
        wheel_msg.angular.z = omega_fused
        self.vel_pub.publish(wheel_msg)
        
        # Publicar diagnóstico completo
        diag_msg = Float32MultiArray()
        diag_msg.data = [
            float(self.x),          # 0: posición x
            float(self.y),          # 1: posición y
            float(self.theta),      # 2: orientación
            float(v_left),          # 3: velocidad rueda izquierda
            float(v_right),         # 4: velocidad rueda derecha
            float(v),               # 5: velocidad lineal robot
            float(omega_enc),       # 6: omega de encoders
            float(omega_gyro),      # 7: omega de gyro
            float(omega_fused),     # 8: omega fusionado
            float(self.total_distance),  # 9: distancia total
            float(self.encoder_correction)  # 10: factor corrección
        ]
        self.diagnostics_pub.publish(diag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialOdometry2Encoders()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo diferencial (2 encoders) detenido")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()