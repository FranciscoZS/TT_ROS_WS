#!/usr/bin/env python3
"""
Nodo que publica 3 odometrías simultáneamente para comparación:
1. Solo Encoders (cinemática directa)
2. Solo IMU (integración de velocidades, NO posiciones)
3. EKF Fusionado (mejor de ambos mundos)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
from library_opi.ekf_odometry_complete import ExtendedKalmanFilter
import math
import numpy as np

class TripleOdometryComparison(Node):
    def __init__(self):
        super().__init__('triple_odometry_comparison')
        
        # ========== PARÁMETROS ==========
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx', 0.15)
        self.declare_parameter('ly', 0.14)
        self.declare_parameter('update_rate', 50.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        update_rate = self.get_parameter('update_rate').value
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando sensores...")
        
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=5.0)
        
        # ========== SISTEMA 1: SOLO ENCODERS ==========
        self.enc_x = 0.0
        self.enc_y = 0.0
        self.enc_theta = 0.0
        
        # ========== SISTEMA 2: SOLO IMU ==========
        # IMPORTANTE: Integramos velocidades, NO aceleraciones directamente a posición
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.imu_theta = 0.0
        self.imu_vx = 0.0  # Velocidad en frame del robot
        self.imu_vy = 0.0
        
        # Filtro para velocidades del IMU
        self.imu_vel_decay = 0.95  # Decaimiento para evitar drift
        
        # ========== SISTEMA 3: EKF FUSIONADO ==========
        self.ekf = ExtendedKalmanFilter(self.wheel_radius, self.lx, self.ly)
        
        # Configuración optimizada
        self.ekf.set_process_noise(q_pos=0.0005, q_theta=0.0001, q_vel=0.02)
        self.ekf.set_measurement_noise_encoders(r_vx=0.01, r_vy=0.05, r_omega=0.05)
        self.ekf.set_measurement_noise_gyro(0.0001)
        
        # ========== FILTROS COMPARTIDOS ==========
        self.vel_buffer_size = 5
        self.vx_buffer = []
        self.vy_buffer = []
        self.omega_buffer = []
        
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0
        
        # ========== PUBLISHERS ==========
        self.odom_enc_pub = self.create_publisher(Odometry, '/odom_encoders_only', 10)
        self.odom_imu_pub = self.create_publisher(Odometry, '/odom_encoders_gyro', 10)
        self.odom_ekf_pub = self.create_publisher(Odometry, '/odom_ekf_fused', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        self.last_time = self.get_clock().now()
        
        self.log_counter = 0
        
        self.get_logger().info("✅ Sistema de comparación inicializado")
        self.get_logger().info("📊 Tópicos publicados:")
        self.get_logger().info("   - /odom_encoders_only (azul en visualización)")
        self.get_logger().info("   - /odom_imu_only (rojo en visualización)")
        self.get_logger().info("   - /odom_ekf_fused (verde en visualización)")

    def moving_average(self, buffer, new_value, buffer_size):
        """Media móvil"""
        buffer.append(new_value)
        if len(buffer) > buffer_size:
            buffer.pop(0)
        return np.mean(buffer)

    def update(self):
        """Ciclo principal de comparación"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.5:
            dt = 0.05
        
        # ========== OBTENER DATOS CRUDOS ==========
        
        # IMU
        imu_data = self.imu.update()
        
        # Encoders
        w_fl = self.enc_fl.calculate_rpm() * 0.10472
        w_fr = self.enc_fr.calculate_rpm() * 0.10472
        w_rl = self.enc_rl.calculate_rpm() * 0.10472
        w_rr = self.enc_rr.calculate_rpm() * 0.10472
        
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # Cinemática
        vx_enc = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_enc = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        omega_enc = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
        # Filtrado
        vx_filt = self.moving_average(self.vx_buffer, vx_enc, self.vel_buffer_size)
        vy_filt = self.moving_average(self.vy_buffer, vy_enc, self.vel_buffer_size)
        omega_filt = self.moving_average(self.omega_buffer, omega_enc, self.vel_buffer_size)
        
        # ========== SISTEMA 1: SOLO ENCODERS ==========
        self.update_encoders_only(vx_filt, vy_filt, omega_filt, dt)
        
        # ========== SISTEMA 2: SOLO IMU ==========
        self.update_imu_only(imu_data, dt)
        
        # ========== SISTEMA 3: EKF FUSIONADO ==========
        self.update_ekf_fused(vx_filt, vy_filt, omega_filt, imu_data, dt)
        
        # ========== PUBLICAR ==========
        self.publish_all_odometries(current_time)
        
        # ========== LOGS ==========
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f"\n📊 COMPARACIÓN (t={self.log_counter*dt:.1f}s):\n"
                f"   🔵 Encoders: ({self.enc_x:.3f}, {self.enc_y:.3f}) θ={math.degrees(self.enc_theta):6.1f}°\n"
                f"   🔴 IMU:      ({self.imu_x:.3f}, {self.imu_y:.3f}) θ={math.degrees(self.imu_theta):6.1f}°\n"
                f"   🟢 EKF:      ({self.ekf.state[0]:.3f}, {self.ekf.state[1]:.3f}) θ={math.degrees(self.ekf.state[2]):6.1f}°"
            )
        
        self.last_time = current_time

    def update_encoders_only(self, vx, vy, omega, dt):
        """
        Sistema 1: Solo encoders con cinemática directa
        Ventajas: Sin drift de sensores inerciales
        Desventajas: Patinaje en rotaciones, acumulación de errores
        """
        # Integración simple de velocidades
        delta_x = (vx * math.cos(self.enc_theta) - vy * math.sin(self.enc_theta)) * dt
        delta_y = (vx * math.sin(self.enc_theta) + vy * math.cos(self.enc_theta)) * dt
        delta_theta = omega * dt
        
        self.enc_x += delta_x
        self.enc_y += delta_y
        self.enc_theta += delta_theta
        
        # Normalizar theta
        self.enc_theta = math.atan2(math.sin(self.enc_theta), math.cos(self.enc_theta))

    def update_imu_only(self, imu_data, dt):
        """
        Sistema 2: Solo IMU (integración de velocidades, NO doble integración)
        
        MÉTODO MEJORADO:
        1. Usar gyro para orientación (muy confiable)
        2. Integrar aceleraciones a VELOCIDADES (1 integración, aceptable)
        3. Aplicar decaimiento para evitar drift
        4. Integrar velocidades a posición
        
        Ventajas: Bueno para movimientos dinámicos, no depende de contacto con suelo
        Desventajas: Drift acumulativo en velocidades
        """
        # 1. Orientación del gyro (muy confiable)
        self.imu_theta = imu_data['theta']
        
        # 2. Integrar aceleraciones a velocidades (frame del robot)
        acc_x = imu_data['acc_x']
        acc_y = imu_data['acc_y']
        
        # Solo integrar si NO está estático
        if not imu_data['is_static']:
            self.imu_vx += acc_x * dt
            self.imu_vy += acc_y * dt
        
        # 3. Decaimiento para evitar drift (simula fricción)
        self.imu_vx *= self.imu_vel_decay
        self.imu_vy *= self.imu_vel_decay
        
        # Deadzone
        if abs(self.imu_vx) < 0.01:
            self.imu_vx = 0.0
        if abs(self.imu_vy) < 0.01:
            self.imu_vy = 0.0
        
        # 4. Transformar velocidades a frame global
        vx_global = self.imu_vx * math.cos(self.imu_theta) - self.imu_vy * math.sin(self.imu_theta)
        vy_global = self.imu_vx * math.sin(self.imu_theta) + self.imu_vy * math.cos(self.imu_theta)
        
        # 5. Integrar a posición
        self.imu_x += vx_global * dt
        self.imu_y += vy_global * dt

    def update_ekf_fused(self, vx_enc, vy_enc, omega_enc, imu_data, dt):
        """
        Sistema 3: EKF fusionado (mejor de ambos mundos)
        
        Ventajas: 
        - Compensa patinaje de encoders con IMU
        - Reduce drift del IMU con encoders
        - Estimación óptima estadísticamente
        
        Desventajas: Más complejo computacionalmente
        """
        # Predicción
        self.ekf.predict(dt)
        
        # Actualización con encoders
        if abs(vx_enc) > 0.005 or abs(vy_enc) > 0.005 or abs(omega_enc) > 0.01:
            self.ekf.update_encoders(vx_enc, vy_enc, omega_enc)
        
        # Actualización con gyro (siempre)
        self.ekf.update_gyro(imu_data['w_z'])
        
        # Corrección con aceleraciones (solo si se mueve)
        if not imu_data['is_static'] and (abs(vx_enc) > 0.01 or abs(vy_enc) > 0.01):
            self.ekf.correct_with_accelerations(
                imu_data['acc_x'],
                imu_data['acc_y'],
                dt
            )

    def publish_all_odometries(self, timestamp):
        """Publica las 3 odometrías"""
        # Sistema 1: Encoders
        self.publish_odometry(
            self.odom_enc_pub,
            self.enc_x, self.enc_y, self.enc_theta,
            0.0, 0.0, 0.0,  # No publicamos velocidades para claridad
            timestamp,
            "odom_enc",
            "base_link_enc"
        )
        
        # Sistema 2: IMU
        self.publish_odometry(
            self.odom_imu_pub,
            self.enc_x, self.enc_y, self.imu_theta,
            self.imu_vx, self.imu_vy, 0,
            timestamp,
            "odom_imu",
            "base_link_imu"
        )
        
        # Sistema 3: EKF
        state = self.ekf.get_state()
        self.publish_odometry(
            self.odom_ekf_pub,
            state['x'], state['y'], state['theta'],
            state['vx'], state['vy'], state['omega'],
            timestamp,
            "odom_ekf",
            "base_link_ekf"
        )

    def publish_odometry(self, publisher, x, y, theta, vx, vy, omega, 
                        timestamp, frame_id, child_frame_id):
        """Publica un mensaje de odometría"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.angular.z = float(omega)
        
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TripleOdometryComparison()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Comparación detenida")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
