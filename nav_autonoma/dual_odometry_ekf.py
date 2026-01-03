#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
from library_opi.ekf_odometry_complete import ExtendedKalmanFilter
import math
import numpy as np

class DualOdometryEKFNode(Node):
    def __init__(self):
        super().__init__('dual_odometry_ekf_node')
        
        # ========== PARÁMETROS DEL ROBOT ==========
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx', 0.15)
        self.declare_parameter('ly', 0.14)
        self.declare_parameter('update_rate', 50.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        update_rate = self.get_parameter('update_rate').value
        
        # ========== INICIALIZAR HARDWARE ==========
        self.get_logger().info("🔧 Inicializando sensores...")
        
        # IMPORTANTE: Ajusta 'invert' según el diagnóstico
        # Ejecuta primero: ros2 run library_opi diagnostic_calibration_node.py
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        # IMU mejorado
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=5.0)
        
        # ========== FILTRO DE KALMAN EXTENDIDO ==========
        self.ekf = ExtendedKalmanFilter(self.wheel_radius, self.lx, self.ly)
        
        # AJUSTES CRÍTICOS PARA REDUCIR RUIDO
        self.ekf.set_process_noise(
            q_pos=0.0005,    # Muy bajo = confiamos en el modelo
            q_theta=0.0001,  # Muy bajo = gyro es muy confiable
            q_vel=0.02       # Bajo = las velocidades no cambian bruscamente
        )
        
        self.ekf.set_measurement_noise_encoders(
            r_vx=0.01,       # Confianza media en vx
            r_vy=0.05,       # Poca confianza en vy (patinaje lateral)
            r_omega=0.05     # Poca confianza (preferimos gyro)
        )
        
        self.ekf.set_measurement_noise_gyro(0.0001)  # Alta confianza en gyro
        
        # ========== FILTROS ADICIONALES ==========
        # Filtro de media móvil para velocidades de encoders
        self.vel_buffer_size = 5
        self.vx_buffer = []
        self.vy_buffer = []
        self.omega_enc_buffer = []
        
        # Detector de outliers (para eliminar picos)
        self.max_vel_change = 0.5  # m/s máximo cambio entre muestras
        self.max_omega_change = 2.0  # rad/s máximo cambio
        
        self.last_vx_enc = 0.0
        self.last_vy_enc = 0.0
        self.last_omega_enc = 0.0
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.encoder_vel_pub = self.create_publisher(Twist, '/encoder_velocities', 10)
        self.imu_data_pub = self.create_publisher(Float32MultiArray, '/imu_data', 10)
        self.diagnostics_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        self.last_time = self.get_clock().now()
        
        # Contador para logs
        self.log_counter = 0
        
        # Variables para diagnóstico
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info(f"✅ Nodo iniciado a {update_rate} Hz")
        self.get_logger().info(f"📊 Publicando en: /odom")
        self.get_logger().info(f"⚠️  Si ves escala incorrecta, ejecuta: ros2 run library_opi diagnostic_calibration_node.py")

    def filter_velocity(self, value, last_value, max_change):
        """Filtra outliers en velocidades"""
        delta = abs(value - last_value)
        if delta > max_change:
            # Outlier detectado, usar valor anterior
            return last_value
        return value

    def moving_average(self, buffer, new_value, buffer_size):
        """Filtro de media móvil"""
        buffer.append(new_value)
        if len(buffer) > buffer_size:
            buffer.pop(0)
        return np.mean(buffer)

    def update(self):
        """Ciclo principal de actualización"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.5:
            dt = 0.05
        
        # ========== 1. OBTENER DATOS DEL IMU ==========
        imu_data = self.imu.update()
        
        # ========== 2. OBTENER VELOCIDADES DE ENCODERS ==========
        # Calcular RPM y convertir a rad/s
        w_fl = self.enc_fl.calculate_rpm() * 0.10472
        w_fr = self.enc_fr.calculate_rpm() * 0.10472
        w_rl = self.enc_rl.calculate_rpm() * 0.10472
        w_rr = self.enc_rr.calculate_rpm() * 0.10472
        
        # Velocidades lineales de las ruedas (v = ω * r)
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # ========== 3. CINEMÁTICA MECANUM ==========
        # CRÍTICO: Verifica estos signos con el diagnóstico
        # Para configuración estándar de mecanum:
        vx_enc_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_enc_raw = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        omega_enc_raw = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
        # ========== 4. FILTRADO DE OUTLIERS ==========
        vx_enc = self.filter_velocity(vx_enc_raw, self.last_vx_enc, self.max_vel_change)
        vy_enc = self.filter_velocity(vy_enc_raw, self.last_vy_enc, self.max_vel_change)
        omega_enc = self.filter_velocity(omega_enc_raw, self.last_omega_enc, self.max_omega_change)
        
        self.last_vx_enc = vx_enc
        self.last_vy_enc = vy_enc
        self.last_omega_enc = omega_enc
        
        # ========== 5. MEDIA MÓVIL ==========
        vx_enc_filtered = self.moving_average(self.vx_buffer, vx_enc, self.vel_buffer_size)
        vy_enc_filtered = self.moving_average(self.vy_buffer, vy_enc, self.vel_buffer_size)
        omega_enc_filtered = self.moving_average(self.omega_enc_buffer, omega_enc, self.vel_buffer_size)
        
        # ========== 6. APLICAR DEADZONE ==========
        # Solo si está casi estático
        if abs(vx_enc_filtered) < 0.01:
            vx_enc_filtered = 0.0
        if abs(vy_enc_filtered) < 0.01:
            vy_enc_filtered = 0.0
        if abs(omega_enc_filtered) < 0.02:
            omega_enc_filtered = 0.0
        
        # ========== 7. FILTRO DE KALMAN EXTENDIDO ==========
        
        # 7.1 Predicción
        self.ekf.predict(dt)
        
        # 7.2 Actualización con encoders (solo si hay movimiento significativo)
        if abs(vx_enc_filtered) > 0.005 or abs(vy_enc_filtered) > 0.005 or abs(omega_enc_filtered) > 0.01:
            self.ekf.update_encoders(vx_enc_filtered, vy_enc_filtered, omega_enc_filtered)
        
        # 7.3 Actualización con gyro (SIEMPRE, es muy confiable)
        self.ekf.update_gyro(imu_data['w_z'])
        
        # 7.4 Corrección con aceleraciones (solo si no está estático Y hay movimiento)
        if not imu_data['is_static'] and (abs(vx_enc_filtered) > 0.01 or abs(vy_enc_filtered) > 0.01):
            self.ekf.correct_with_accelerations(
                imu_data['acc_x'],
                imu_data['acc_y'],
                dt
            )
        
        # ========== 8. OBTENER ESTADO FILTRADO ==========
        state = self.ekf.get_state()
        
        # ========== 9. CALCULAR DISTANCIA RECORRIDA ==========
        dx = state['x'] - self.last_x
        dy = state['y'] - self.last_y
        self.total_distance += math.sqrt(dx**2 + dy**2)
        self.last_x = state['x']
        self.last_y = state['y']
        
        # ========== 10. PUBLICAR ODOMETRÍA ==========
        self.publish_odometry(state, current_time)
        
        # ========== 11. PUBLICAR DATOS DE DIAGNÓSTICO ==========
        self.publish_diagnostics(state, vx_enc_filtered, vy_enc_filtered, omega_enc_filtered, imu_data)
        
        # ========== 12. LOGS PERIÓDICOS ==========
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            cov = self.ekf.get_covariance()
            self.get_logger().info(
                f"📍 Pos: ({state['x']:.3f}, {state['y']:.3f}) "
                f"θ: {math.degrees(state['theta']):.1f}° | "
                f"Dist: {self.total_distance:.2f}m | "
                f"Vel: ({state['vx']:.2f}, {state['vy']:.2f}) "
                f"ω: {math.degrees(state['omega']):.1f}°/s"
            )
            
            # Advertencia si covarianza es muy alta
            if cov[0] > 0.1 or cov[1] > 0.1:
                self.get_logger().warn(f"⚠️  Alta incertidumbre: σ²_x={cov[0]:.3f}, σ²_y={cov[1]:.3f}")
        
        self.last_time = current_time

    def publish_odometry(self, state, timestamp):
        """Publica mensaje de odometría"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # Posición
        msg.pose.pose.position.x = float(state['x'])
        msg.pose.pose.position.y = float(state['y'])
        msg.pose.pose.position.z = 0.0
        
        # Orientación (quaternion)
        theta = state['theta']
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Covarianza de pose
        cov = self.ekf.get_covariance()
        msg.pose.covariance[0] = cov[0]
        msg.pose.covariance[7] = cov[1]
        msg.pose.covariance[35] = cov[2]
        
        # Velocidades
        msg.twist.twist.linear.x = float(state['vx'])
        msg.twist.twist.linear.y = float(state['vy'])
        msg.twist.twist.angular.z = float(state['omega'])
        
        # Covarianza de twist
        msg.twist.covariance[0] = cov[3]
        msg.twist.covariance[7] = cov[4]
        msg.twist.covariance[35] = cov[5]
        
        self.odom_pub.publish(msg)

    def publish_diagnostics(self, state, vx_enc, vy_enc, omega_enc, imu_data):
        """Publica datos de diagnóstico"""
        # Velocidades de encoders
        enc_msg = Twist()
        enc_msg.linear.x = vx_enc
        enc_msg.linear.y = vy_enc
        enc_msg.angular.z = omega_enc
        self.encoder_vel_pub.publish(enc_msg)
        
        # Datos del IMU
        imu_msg = Float32MultiArray()
        imu_msg.data = [
            float(imu_data['w_z']),
            float(imu_data['theta']),
            float(imu_data['acc_x']),
            float(imu_data['acc_y']),
            1.0 if imu_data['is_static'] else 0.0
        ]
        self.imu_data_pub.publish(imu_msg)
        
        # Diagnóstico del EKF
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
    node = DualOdometryEKFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido por usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()