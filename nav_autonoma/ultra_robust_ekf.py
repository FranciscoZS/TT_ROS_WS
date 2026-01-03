#!/usr/bin/env python3
"""
Nodo de odometría ULTRA-ROBUSTO para robots mecanum
Solución agresiva al ruido durante rotaciones
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
from collections import deque

class UltraRobustEKFNode(Node):
    def __init__(self):
        super().__init__('ultra_robust_ekf_node')
        
        # ========== PARÁMETROS ==========
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx', 0.175)
        self.declare_parameter('ly', 0.15)
        self.declare_parameter('update_rate', 100.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        update_rate = self.get_parameter('update_rate').value
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando hardware...")
        
        # IMPORTANTE: Ajusta según diagnóstico
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=5.0)
        
        # ========== EKF ==========
        self.ekf = ExtendedKalmanFilter(self.wheel_radius, self.lx, self.ly)
        
        # ========== DETECCIÓN DE MODO ==========
        self.motion_mode = "STATIC"
        self.mode_history = deque(maxlen=20)  # Buffer más grande
        
        # Umbrales más estrictos
        self.LINEAR_VEL_THRESHOLD = 0.015  # m/s
        self.ROTATION_VEL_THRESHOLD = 0.08  # rad/s (más sensible)
        
        # ========== FILTROS ULTRA-AGRESIVOS ==========
        # Filtro de media móvil adaptativo
        self.vel_buffer_size_linear = 5   # En movimiento lineal
        self.vel_buffer_size_rotation = 15  # Durante rotación (MÁS FILTRADO)
        
        self.vx_buffer = deque(maxlen=15)
        self.vy_buffer = deque(maxlen=15)
        self.omega_enc_buffer = deque(maxlen=15)
        self.omega_gyro_buffer = deque(maxlen=10)
        
        # Filtro de outliers más agresivo
        self.max_vel_change_linear = 0.3     # m/s
        self.max_vel_change_rotation = 0.1   # m/s (muy conservador durante rotación)
        self.max_omega_change = 1.0          # rad/s
        
        # Saturación de velocidades
        self.MAX_LINEAR_VEL = 0.5   # m/s
        self.MAX_ANGULAR_VEL = 4.0  # rad/s (límite físico realista)
        
        # Variables anteriores
        self.last_vx_enc = 0.0
        self.last_vy_enc = 0.0
        self.last_omega_enc = 0.0
        self.last_omega_gyro = 0.0
        
        # ========== RECHAZAR ENCODERS DURANTE ROTACIÓN ==========
        self.rotation_reject_counter = 0
        self.rotation_reject_threshold = 5  # Ignorar encoders por 5 ciclos después de detectar rotación
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_pub = self.create_publisher(String, '/motion_mode', 10)
        self.encoder_vel_pub = self.create_publisher(Twist, '/encoder_velocities', 10)
        self.gyro_vel_pub = self.create_publisher(Twist, '/gyro_velocities', 10)
        self.diagnostics_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        self.last_time = self.get_clock().now()
        
        self.log_counter = 0
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info(f"✅ Sistema ULTRA-ROBUSTO iniciado a {update_rate} Hz")
        self.get_logger().info("🔒 Modo agresivo: encoders rechazados durante rotación")

    def saturate(self, value, max_value):
        """Satura un valor"""
        return np.clip(value, -max_value, max_value)

    def filter_outlier(self, value, last_value, max_change):
        """Filtro de outliers"""
        delta = abs(value - last_value)
        if delta > max_change:
            return last_value
        return value

    def moving_average_adaptive(self, buffer, new_value, mode):
        """Media móvil con tamaño adaptativo según modo"""
        buffer.append(new_value)
        
        if mode == "ROTATION":
            # Durante rotación, usar MÁS muestras
            n_samples = min(self.vel_buffer_size_rotation, len(buffer))
        else:
            n_samples = min(self.vel_buffer_size_linear, len(buffer))
        
        # Usar solo las últimas n_samples
        recent_values = list(buffer)[-n_samples:]
        
        # Media ponderada (más peso a valores recientes)
        weights = np.exp(np.linspace(-1, 0, len(recent_values)))
        weights /= weights.sum()
        
        return np.average(recent_values, weights=weights)

    def detect_motion_mode(self, vx, vy, omega_gyro):
        """
        Detección mejorada de modo de movimiento
        Usa el GYRO para detectar rotación (no los encoders)
        """
        linear_vel = math.sqrt(vx**2 + vy**2)
        angular_vel = abs(omega_gyro)  # Usar gyro, no encoders!
        
        # Clasificar
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
        
        # Buffer más largo para evitar cambios bruscos
        self.mode_history.append(mode)
        
        # Modo más común en las últimas 10 muestras
        if len(self.mode_history) >= 10:
            from collections import Counter
            mode_counts = Counter(list(self.mode_history)[-10:])
            return mode_counts.most_common(1)[0][0]
        
        return mode

    def configure_ekf_for_mode(self, mode):
        """Configura EKF según modo de movimiento"""
        if mode == "ROTATION":
            # ROTACIÓN: Confiar SOLO en gyro
            self.ekf.set_measurement_noise_encoders(
                r_vx=10.0,    # Prácticamente ignorar
                r_vy=10.0,    # Prácticamente ignorar
                r_omega=10.0  # Prácticamente ignorar
            )
            self.ekf.set_measurement_noise_gyro(0.00005)  # Máxima confianza en gyro
            self.ekf.set_process_noise(q_pos=0.001, q_theta=0.00005, q_vel=0.1)
            
        elif mode == "LINEAR":
            # LINEAL: Confiar en encoders y gyro
            self.ekf.set_measurement_noise_encoders(
                r_vx=0.005,   # Alta confianza en vx
                r_vy=0.02,    # Media confianza en vy
                r_omega=0.1   # Poca confianza (preferir gyro)
            )
            self.ekf.set_measurement_noise_gyro(0.0001)
            self.ekf.set_process_noise(q_pos=0.0003, q_theta=0.0001, q_vel=0.015)
            
        elif mode == "MIXED":
            # MIXTO: Balance
            self.ekf.set_measurement_noise_encoders(
                r_vx=0.02,
                r_vy=0.05,
                r_omega=0.5  # Poco peso en omega de encoders
            )
            self.ekf.set_measurement_noise_gyro(0.0001)
            self.ekf.set_process_noise(q_pos=0.0005, q_theta=0.0001, q_vel=0.03)
            
        else:  # STATIC
            # Estático: Alta incertidumbre en mediciones
            self.ekf.set_measurement_noise_encoders(
                r_vx=0.5, r_vy=0.5, r_omega=0.5
            )
            self.ekf.set_measurement_noise_gyro(0.001)
            self.ekf.set_process_noise(q_pos=0.001, q_theta=0.0005, q_vel=0.1)

    def update(self):
        """Ciclo principal ULTRA-ROBUSTO"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.5:
            dt = 0.05
        
        # ========== 1. IMU (SOLO GYRO) ==========
        imu_data = self.imu.update()
        
        # Filtrar gyro con buffer
        self.omega_gyro_buffer.append(imu_data['w_z'])
        omega_gyro_filtered = np.median(list(self.omega_gyro_buffer))  # Mediana es robusta a outliers
        
        # Saturar gyro
        omega_gyro_filtered = self.saturate(omega_gyro_filtered, self.MAX_ANGULAR_VEL)
        
        # ========== 2. ENCODERS ==========
        w_fl = self.enc_fl.calculate_rpm() * 0.10472
        w_fr = self.enc_fr.calculate_rpm() * 0.10472
        w_rl = self.enc_rl.calculate_rpm() * 0.10472
        w_rr = self.enc_rr.calculate_rpm() * 0.10472
        #w_fl = w_rr
        #w_fr = w_rl

        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # ========== 3. CINEMÁTICA (CORREGIDA) ==========
        vx_enc_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        
        # FIX: Invertir signo de vy para corregir dirección
        vy_enc_raw = (-v_fl + v_fr -v_rl + v_rr) / 4.0  # Signo negativo añadido
        
        omega_enc_raw = (-v_fl + v_fr -v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
        # ========== 4. DETECTAR MODO (USA GYRO, NO ENCODERS) ==========
        current_mode = self.detect_motion_mode(vx_enc_raw, vy_enc_raw, omega_gyro_filtered)
        
        # ========== 5. FILTRADO ADAPTATIVO ==========
        # Cambiar límites según modo
        if current_mode == "ROTATION":
            max_change = self.max_vel_change_rotation
            self.rotation_reject_counter = 10  # Rechazar encoders por 10 ciclos
        else:
            max_change = self.max_vel_change_linear
        
        # Filtro de outliers
        vx_enc = self.filter_outlier(vx_enc_raw, self.last_vx_enc, max_change)
        vy_enc = self.filter_outlier(vy_enc_raw, self.last_vy_enc, max_change)
        omega_enc = self.filter_outlier(omega_enc_raw, self.last_omega_enc, self.max_omega_change)
        
        self.last_vx_enc = vx_enc
        self.last_vy_enc = vy_enc
        self.last_omega_enc = omega_enc
        
        # Media móvil adaptativa
        vx_enc_filtered = self.moving_average_adaptive(self.vx_buffer, vx_enc, current_mode)
        vy_enc_filtered = self.moving_average_adaptive(self.vy_buffer, vy_enc, current_mode)
        omega_enc_filtered = self.moving_average_adaptive(self.omega_enc_buffer, omega_enc, current_mode)
        
        # Saturar velocidades
        vx_enc_filtered = self.saturate(vx_enc_filtered, self.MAX_LINEAR_VEL)
        vy_enc_filtered = self.saturate(vy_enc_filtered, self.MAX_LINEAR_VEL)
        omega_enc_filtered = self.saturate(omega_enc_filtered, self.MAX_ANGULAR_VEL)
        
        # Deadzone
        if abs(vx_enc_filtered) < 0.008:
            vx_enc_filtered = 0.0
        if abs(vy_enc_filtered) < 0.008:
            vy_enc_filtered = 0.0
        if abs(omega_enc_filtered) < 0.015:
            omega_enc_filtered = 0.0
        
        # ========== 6. CONFIGURAR EKF SEGÚN MODO ==========
        if current_mode != self.motion_mode:
            self.configure_ekf_for_mode(current_mode)
            self.motion_mode = current_mode
            self.get_logger().info(f"🔄 Modo: {current_mode}")
        
        # ========== 7. EKF ULTRA-ROBUSTO ==========
        self.ekf.predict(dt)
        
        # CRÍTICO: Durante rotación, RECHAZAR encoders completamente
        if current_mode == "ROTATION" or self.rotation_reject_counter > 0:
            # SOLO usar gyro durante rotación
            self.ekf.update_gyro(omega_gyro_filtered)
            
            # NO actualizar con encoders
            if self.rotation_reject_counter > 0:
                self.rotation_reject_counter -= 1
                
        else:
            # Movimiento lineal o mixto: usar encoders + gyro
            if abs(vx_enc_filtered) > 0.005 or abs(vy_enc_filtered) > 0.005:
                self.ekf.update_encoders(vx_enc_filtered, vy_enc_filtered, omega_enc_filtered)
            
            # SIEMPRE actualizar con gyro (es confiable)
            self.ekf.update_gyro(omega_gyro_filtered)
        
        # NO usar aceleraciones del IMU (causan drift)
        
        # ========== 8. ESTADO ==========
        state = self.ekf.get_state()
        
        # ========== 9. DISTANCIA ==========
        dx = state['x'] - self.last_x
        dy = state['y'] - self.last_y
        self.total_distance += math.sqrt(dx**2 + dy**2)
        self.last_x = state['x']
        self.last_y = state['y']
        
        # ========== 10. PUBLICAR ==========
        self.publish_odometry(state, current_time)
        self.publish_mode(current_mode)
        self.publish_diagnostics(state, vx_enc_filtered, vy_enc_filtered, 
                                omega_enc_filtered, omega_gyro_filtered)
        
        self.ms_fr = self.enc_fr.get_velocity(self.wheel_radius)
        self.ms_fl = self.enc_fl.get_velocity(self.wheel_radius)
        self.ms_rr = self.enc_rr.get_velocity(self.wheel_radius)
        self.ms_rl = self.enc_rl.get_velocity(self.wheel_radius)
        
        # ========== 11. LOGS ==========
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            reject_status = f"[REJECT:{self.rotation_reject_counter}]" if self.rotation_reject_counter > 0 else ""
            self.get_logger().info(
                f"📍 [{current_mode:8s}] {reject_status} "
                f"Pos: ({state['x']:.3f}, {state['y']:.3f}) "
                f"θ: {math.degrees(state['theta']):6.1f}° | "
                f"ω_gyro: {math.degrees(omega_gyro_filtered):5.1f}°/s | "
                f"Dist: {self.total_distance:.2f}m"
                f"""
                ==================ENCODERS=================
                fl:{self.enc_fl.counter} - fr:{self.enc_fr.counter} - rl:{self.enc_rl.counter} - rr:{self.enc_rr.counter}
                ================VELOCIDADES================
                fl:{self.ms_fl} - fr:{self.ms_fr} - rl:{self.ms_rl} - rr:{self.ms_rr}
                ===========================================
                """
            )


        
        self.last_time = current_time

    def publish_mode(self, mode):
        """Publica modo de movimiento"""
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

    def publish_diagnostics(self, state, vx_enc, vy_enc, omega_enc, omega_gyro):
        """Publica diagnósticos"""
        # Velocidades encoders
        enc_msg = Twist()
        enc_msg.linear.x = vx_enc
        enc_msg.linear.y = vy_enc
        enc_msg.angular.z = omega_enc
        self.encoder_vel_pub.publish(enc_msg)
        
        # Velocidades gyro
        gyro_msg = Twist()
        gyro_msg.angular.z = omega_gyro
        self.gyro_vel_pub.publish(gyro_msg)
        
        # Diagnóstico EKF
        diag_msg = Float32MultiArray()
        cov = self.ekf.get_covariance()
        diag_msg.data = [
            float(state['x']),
            float(state['y']),
            float(state['theta']),
            float(cov[0]),
            float(cov[1]),
            float(cov[2]),
            float(self.total_distance),
            float(self.rotation_reject_counter)  # Indicador de rechazo
        ]
        self.diagnostics_pub.publish(diag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltraRobustEKFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
