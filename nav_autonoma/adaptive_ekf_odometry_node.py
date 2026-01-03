#!/usr/bin/env python3
"""
Nodo de odometría con EKF adaptativo
Detecta el modo de movimiento (lineal vs rotación) y ajusta matrices dinámicamente
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
from library_opi.ekf_odometry_complete import ExtendedKalmanFilter
import math
import numpy as np

class AdaptiveEKFOdometryNode(Node):
    def __init__(self):
        super().__init__('adaptive_ekf_odometry_node')
        
        # ========== PARÁMETROS ==========
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx', 0.15)
        self.declare_parameter('ly', 0.14)
        self.declare_parameter('update_rate', 100.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        update_rate = self.get_parameter('update_rate').value
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando hardware...")
        
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=5.0)
        
        # ========== EKF ==========
        self.ekf = ExtendedKalmanFilter(self.wheel_radius, self.lx, self.ly)
        
        # ========== DETECCIÓN DE MODO DE MOVIMIENTO ==========
        self.motion_mode = "STATIC"  # STATIC, LINEAR, ROTATION, MIXED
        self.mode_buffer = []
        self.mode_buffer_size = 10
        
        # Umbrales para detección
        self.LINEAR_VEL_THRESHOLD = 0.02  # m/s
        self.ROTATION_VEL_THRESHOLD = 0.1  # rad/s
        
        # ========== MATRICES ADAPTATIVAS ==========
        # Configuraciones de ruido según modo
        self.noise_configs = {
            'STATIC': {
                'r_vx': 0.1, 'r_vy': 0.1, 'r_omega': 0.1,
                'r_gyro': 0.001, 'q_vel': 0.05
            },
            'LINEAR': {
                'r_vx': 0.01, 'r_vy': 0.05, 'r_omega': 0.1,  # Confiar en encoders para lineal
                'r_gyro': 0.001, 'q_vel': 0.02
            },
            'ROTATION': {
                'r_vx': 0.2, 'r_vy': 0.2, 'r_omega': 0.5,   # NO confiar en encoders
                'r_gyro': 0.0001, 'q_vel': 0.1              # SÍ confiar en gyro
            },
            'MIXED': {
                'r_vx': 0.05, 'r_vy': 0.1, 'r_omega': 0.2,
                'r_gyro': 0.0005, 'q_vel': 0.05
            }
        }
        
        # Aplicar configuración inicial
        self.apply_noise_config('LINEAR')
        
        # ========== FILTROS ==========
        self.vel_buffer_size = 5
        self.vx_buffer = []
        self.vy_buffer = []
        self.omega_enc_buffer = []
        
        self.max_vel_change = 0.5
        self.max_omega_change = 2.0
        
        self.last_vx_enc = 0.0
        self.last_vy_enc = 0.0
        self.last_omega_enc = 0.0
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_pub = self.create_publisher(String, '/motion_mode', 10)
        self.encoder_vel_pub = self.create_publisher(Twist, '/encoder_velocities', 10)
        self.imu_data_pub = self.create_publisher(Float32MultiArray, '/imu_data', 10)
        self.diagnostics_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        self.last_time = self.get_clock().now()
        
        self.log_counter = 0
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info(f"✅ Nodo adaptativo iniciado a {update_rate} Hz")

    def detect_motion_mode(self, vx, vy, omega):
        """
        Detecta el modo de movimiento actual
        
        Returns:
            str: 'STATIC', 'LINEAR', 'ROTATION', o 'MIXED'
        """
        linear_vel = math.sqrt(vx**2 + vy**2)
        angular_vel = abs(omega)
        
        # Clasificar movimiento
        is_moving_linear = linear_vel > self.LINEAR_VEL_THRESHOLD
        is_rotating = angular_vel > self.ROTATION_VEL_THRESHOLD
        
        if not is_moving_linear and not is_rotating:
            mode = "STATIC"
        elif is_rotating and not is_moving_linear:
            mode = "ROTATION"
        elif is_moving_linear and not is_rotating:
            mode = "LINEAR"
        else:
            mode = "MIXED"
        
        # Usar buffer para suavizar cambios de modo
        self.mode_buffer.append(mode)
        if len(self.mode_buffer) > self.mode_buffer_size:
            self.mode_buffer.pop(0)
        
        # Modo más común en el buffer
        if len(self.mode_buffer) >= 3:
            from collections import Counter
            mode_counts = Counter(self.mode_buffer)
            return mode_counts.most_common(1)[0][0]
        
        return mode

    def apply_noise_config(self, mode):
        """Aplica configuración de ruido según el modo"""
        if mode != self.motion_mode:
            config = self.noise_configs[mode]
            
            self.ekf.set_measurement_noise_encoders(
                config['r_vx'],
                config['r_vy'],
                config['r_omega']
            )
            self.ekf.set_measurement_noise_gyro(config['r_gyro'])
            
            # Ajustar Q (proceso)
            self.ekf.set_process_noise(
                q_pos=0.0005,
                q_theta=0.0001,
                q_vel=config['q_vel']
            )
            
            self.motion_mode = mode
            self.get_logger().info(f"🔄 Modo cambiado a: {mode}")

    def filter_velocity(self, value, last_value, max_change):
        """Filtra outliers"""
        delta = abs(value - last_value)
        if delta > max_change:
            return last_value
        return value

    def moving_average(self, buffer, new_value, buffer_size):
        """Media móvil"""
        buffer.append(new_value)
        if len(buffer) > buffer_size:
            buffer.pop(0)
        return np.mean(buffer)

    def update(self):
        """Ciclo principal"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.5:
            dt = 0.05
        
        # ========== IMU ==========
        imu_data = self.imu.update()
        
        # ========== ENCODERS ==========
        w_fl = self.enc_fl.calculate_rpm() * 0.10472
        w_fr = self.enc_fr.calculate_rpm() * 0.10472
        w_rl = self.enc_rl.calculate_rpm() * 0.10472
        w_rr = self.enc_rr.calculate_rpm() * 0.10472
        
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # ========== CINEMÁTICA ==========
        vx_enc_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_enc_raw = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        omega_enc_raw = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
        # ========== FILTRADO ==========
        vx_enc = self.filter_velocity(vx_enc_raw, self.last_vx_enc, self.max_vel_change)
        vy_enc = self.filter_velocity(vy_enc_raw, self.last_vy_enc, self.max_vel_change)
        omega_enc = self.filter_velocity(omega_enc_raw, self.last_omega_enc, self.max_omega_change)
        
        self.last_vx_enc = vx_enc
        self.last_vy_enc = vy_enc
        self.last_omega_enc = omega_enc
        
        vx_enc_filtered = self.moving_average(self.vx_buffer, vx_enc, self.vel_buffer_size)
        vy_enc_filtered = self.moving_average(self.vy_buffer, vy_enc, self.vel_buffer_size)
        omega_enc_filtered = self.moving_average(self.omega_enc_buffer, omega_enc, self.vel_buffer_size)
        
        # ========== DETECCIÓN DE MODO ==========
        current_mode = self.detect_motion_mode(vx_enc_filtered, vy_enc_filtered, imu_data['w_z'])
        self.apply_noise_config(current_mode)
        
        # ========== DEADZONE ==========
        if abs(vx_enc_filtered) < 0.002:
            vx_enc_filtered = 0.0
        if abs(vy_enc_filtered) < 0.002:
            vy_enc_filtered = 0.0
        if abs(omega_enc_filtered) < 0.02:
            omega_enc_filtered = 0.0
        
        # ========== EKF ==========
        self.ekf.predict(dt)
        
        # En modo ROTACIÓN, dar MUY poco peso a encoders
        if current_mode == "ROTATION":
            # Solo actualizar con encoders si hay movimiento significativo
            if abs(vx_enc_filtered) > 0.05 or abs(vy_enc_filtered) > 0.05:
                self.ekf.update_encoders(vx_enc_filtered, vy_enc_filtered, omega_enc_filtered)
        else:
            # En otros modos, usar encoders normalmente
            if abs(vx_enc_filtered) > 0.005 or abs(vy_enc_filtered) > 0.005 or abs(omega_enc_filtered) > 0.01:
                self.ekf.update_encoders(vx_enc_filtered, vy_enc_filtered, omega_enc_filtered)
        
        # SIEMPRE actualizar con gyro (muy confiable)
        self.ekf.update_gyro(imu_data['w_z'])
        
        # Aceleraciones solo en movimiento lineal
        if current_mode == "LINEAR" and not imu_data['is_static']:
            self.ekf.correct_with_accelerations(
                imu_data['acc_x'],
                imu_data['acc_y'],
                dt
            )
        
        # ========== ESTADO ==========
        state = self.ekf.get_state()
        
        # ========== DISTANCIA ==========
        dx = state['x'] - self.last_x
        dy = state['y'] - self.last_y
        self.total_distance += math.sqrt(dx**2 + dy**2)
        self.last_x = state['x']
        self.last_y = state['y']
        
        # ========== PUBLICAR ==========
        self.publish_odometry(state, current_time)
        self.publish_mode(current_mode)
        self.publish_diagnostics(state, vx_enc_filtered, vy_enc_filtered, omega_enc_filtered, imu_data)
        
        # ========== LOGS ==========
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f"📍 [{current_mode:8s}] Pos: ({state['x']:.3f}, {state['y']:.3f}) "
                f"θ: {math.degrees(state['theta']):6.1f}° | "
                f"Dist: {self.total_distance:.2f}m"
            )
        
        self.last_time = current_time

    def publish_mode(self, mode):
        """Publica el modo de movimiento actual"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def publish_odometry(self, state, timestamp):
        """Publica odometría"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = float(state['x'])
        msg.pose.pose.position.y = float(state['y'])
        msg.pose.pose.position.z = 0.0
        
        theta = state['theta']
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        cov = self.ekf.get_covariance()
        msg.pose.covariance[0] = cov[0]
        msg.pose.covariance[7] = cov[1]
        msg.pose.covariance[35] = cov[2]
        
        msg.twist.twist.linear.x = float(state['vx'])
        msg.twist.twist.linear.y = float(state['vy'])
        msg.twist.twist.angular.z = float(state['omega'])
        
        msg.twist.covariance[0] = cov[3]
        msg.twist.covariance[7] = cov[4]
        msg.twist.covariance[35] = cov[5]
        
        self.odom_pub.publish(msg)

    def publish_diagnostics(self, state, vx_enc, vy_enc, omega_enc, imu_data):
        """Publica diagnósticos"""
        enc_msg = Twist()
        enc_msg.linear.x = vx_enc
        enc_msg.linear.y = vy_enc
        enc_msg.angular.z = omega_enc
        self.encoder_vel_pub.publish(enc_msg)
        
        imu_msg = Float32MultiArray()
        imu_msg.data = [
            float(imu_data['w_z']),
            float(imu_data['theta']),
            float(imu_data['acc_x']),
            float(imu_data['acc_y']),
            1.0 if imu_data['is_static'] else 0.0
        ]
        self.imu_data_pub.publish(imu_msg)
        
        diag_msg = Float32MultiArray()
        cov = self.ekf.get_covariance()
        diag_msg.data = [
            float(state['x']),
            float(state['y']),
            float(state['theta']),
            float(cov[0]),
            float(cov[1]),
            float(cov[2]),
            float(self.total_distance)
        ]
        self.diagnostics_pub.publish(diag_msg)



def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveEKFOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
