# #!/usr/bin/env python3
# """
# Nodo de odometría adaptativa simple para robots mecanum
# Usa filtros adaptativos según modo de movimiento
# - Encoders para velocidades lineales (vx, vy)
# - MPU6050 para velocidad angular (omega) y orientación (theta)
# """

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32MultiArray, String
# from library_opi.encoders import OpticalEncoder
# from library_opi.mpu6050_improved import MPU6050
# import math
# import numpy as np
# from collections import deque

# class AdaptiveOdometryNode(Node):
#     def __init__(self):
#         super().__init__('adaptive_odometry_node')
        
#         # ========== PARÁMETROS ==========
#         self.declare_parameter('wheel_radius', 0.05)
#         self.declare_parameter('lx', 0.175)
#         self.declare_parameter('ly', 0.15)
#         self.declare_parameter('update_rate', 100.0)
        
#         self.wheel_radius = self.get_parameter('wheel_radius').value
#         self.lx = self.get_parameter('lx').value
#         self.ly = self.get_parameter('ly').value
#         update_rate = self.get_parameter('update_rate').value
        
#         # ========== HARDWARE ==========
#         self.get_logger().info("🔧 Inicializando hardware...")
        
#         self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
#         self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
#         self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
#         self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
#         self.imu = MPU6050(node=self)
#         self.imu.calibrate(duration=5.0)
        
#         # ========== ESTADO DE ODOMETRÍA ==========
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.vx = 0.0
#         self.vy = 0.0
#         self.omega = 0.0
        
#         # ========== DETECCIÓN DE MODO ==========
#         self.motion_mode = "STATIC"
#         self.mode_history = deque(maxlen=15)
        
#         # Umbrales para detección de modo
#         self.LINEAR_VEL_THRESHOLD = 0.015  # m/s
#         self.ROTATION_VEL_THRESHOLD = 0.08  # rad/s
        
#         # ========== FILTROS ADAPTATIVOS ==========
#         # Buffers para promedio móvil
#         self.vx_buffer = deque(maxlen=10)
#         self.vy_buffer = deque(maxlen=10)
#         self.omega_buffer = deque(maxlen=8)
        
#         # Tamaños de buffer según modo
#         self.buffer_size = {
#             'LINEAR': {'vx': 5, 'vy': 5, 'omega': 3},
#             'ROTATION': {'vx': 8, 'vy': 8, 'omega': 5},
#             'MIXED': {'vx': 6, 'vy': 6, 'omega': 4},
#             'STATIC': {'vx': 10, 'vy': 10, 'omega': 8}
#         }
        
#         # Coeficientes para filtro pasa-bajos exponencial (alpha)
#         # alpha más alto = menos filtrado (más respuesta)
#         # alpha más bajo = más filtrado (más suave)
#         self.alpha = {
#             'LINEAR': {'vx': 0.6, 'vy': 0.6, 'omega': 0.7},
#             'ROTATION': {'vx': 0.3, 'vy': 0.3, 'omega': 0.8},  # Confiar más en gyro
#             'MIXED': {'vx': 0.5, 'vy': 0.5, 'omega': 0.7},
#             'STATIC': {'vx': 0.2, 'vy': 0.2, 'omega': 0.3}
#         }
        
#         # Valores filtrados previos (para filtro exponencial)
#         self.vx_filtered = 0.0
#         self.vy_filtered = 0.0
#         self.omega_filtered = 0.0
        
#         # ========== LÍMITES Y SATURACIÓN ==========
#         self.MAX_LINEAR_VEL = 0.5   # m/s
#         self.MAX_ANGULAR_VEL = 4.0  # rad/s
        
#         # Deadzone (valores muy pequeños -> 0)
#         self.DEADZONE_LINEAR = 0.008   # m/s
#         self.DEADZONE_ANGULAR = 0.015  # rad/s
        
#         # ========== FILTRO DE OUTLIERS ==========
#         self.last_vx_raw = 0.0
#         self.last_vy_raw = 0.0
#         self.last_omega_raw = 0.0
        
#         self.max_vel_change = 0.4      # m/s (cambio máximo permitido por ciclo)
#         self.max_omega_change = 1.5    # rad/s
        
#         # ========== PUBLISHERS ==========
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.mode_pub = self.create_publisher(String, '/motion_mode', 10)
#         self.encoder_vel_pub = self.create_publisher(Twist, '/encoder_velocities', 10)
#         self.gyro_vel_pub = self.create_publisher(Twist, '/gyro_velocities', 10)
#         self.diagnostics_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
#         # ========== TIMER ==========
#         self.create_timer(1.0/update_rate, self.update)
#         self.last_time = self.get_clock().now()
        
#         # ========== ESTADÍSTICAS ==========
#         self.log_counter = 0
#         self.total_distance = 0.0
#         self.last_x = 0.0
#         self.last_y = 0.0
        
#         self.get_logger().info(f"✅ Odometría adaptativa iniciada a {update_rate} Hz")
#         self.get_logger().info("📊 Filtrado: Promedio móvil + Pasa-bajos exponencial")

#     def saturate(self, value, max_value):
#         """Limita un valor a ±max_value"""
#         return np.clip(value, -max_value, max_value)

#     def apply_deadzone(self, value, deadzone):
#         """Aplica zona muerta"""
#         return 0.0 if abs(value) < deadzone else value

#     def reject_outlier(self, value, last_value, max_change):
#         """Rechaza valores que cambian demasiado rápido"""
#         delta = value - last_value
#         if abs(delta) > max_change:
#             # Si el cambio es muy grande, limitar el cambio
#             return last_value + np.sign(delta) * max_change
#         return value

#     def exponential_filter(self, new_value, last_filtered, alpha):
#         """
#         Filtro pasa-bajos exponencial (IIR de primer orden)
#         y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
#         """
#         return alpha * new_value + (1 - alpha) * last_filtered

#     def moving_average(self, buffer, new_value, window_size):
#         """Promedio móvil con ventana adaptativa"""
#         buffer.append(new_value)
        
#         # Usar solo las últimas window_size muestras
#         n = min(window_size, len(buffer))
#         recent = list(buffer)[-n:]
        
#         # Promedio ponderado (más peso a valores recientes)
#         if len(recent) > 1:
#             weights = np.linspace(0.5, 1.0, len(recent))
#             weights /= weights.sum()
#             return np.average(recent, weights=weights)
#         else:
#             return recent[0] if recent else 0.0

#     def detect_motion_mode(self, vx, vy, omega):
#         """
#         Detecta el modo de movimiento basándose en velocidades
#         Usa omega del gyro (más confiable que encoders para rotación)
#         """
#         linear_vel = math.sqrt(vx**2 + vy**2)
#         angular_vel = abs(omega)
        
#         is_moving_linear = linear_vel > self.LINEAR_VEL_THRESHOLD
#         is_rotating = angular_vel > self.ROTATION_VEL_THRESHOLD
        
#         if not is_moving_linear and not is_rotating:
#             mode = "STATIC"
#         elif is_rotating and not is_moving_linear:
#             mode = "ROTATION"
#         elif is_moving_linear and not is_rotating:
#             mode = "LINEAR"
#         else:
#             mode = "MIXED"
        
#         # Buffer para evitar cambios bruscos de modo
#         self.mode_history.append(mode)
        
#         # Modo más común en el historial
#         if len(self.mode_history) >= 8:
#             from collections import Counter
#             mode_counts = Counter(list(self.mode_history)[-8:])
#             return mode_counts.most_common(1)[0][0]
        
#         return mode

#     def update(self):
#         """Ciclo principal de actualización"""
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         # Validar dt
#         if dt > 0.5 or dt <= 0:
#             dt = 0.01
        
#         # ========== 1. LEER GYRO (MPU6050) ==========
#         imu_data = self.imu.update()
#         omega_gyro_raw = imu_data['w_z']
        
#         # Saturar gyro
#         omega_gyro_raw = self.saturate(omega_gyro_raw, self.MAX_ANGULAR_VEL)
        
#         # Filtro de outliers para gyro
#         omega_gyro = self.reject_outlier(omega_gyro_raw, self.last_omega_raw, 
#                                          self.max_omega_change)
#         self.last_omega_raw = omega_gyro
        
#         # ========== 2. LEER ENCODERS ==========
#         # Convertir RPM a rad/s
#         w_fl = self.enc_fl.calculate_rpm() * 0.10472
#         w_fr = self.enc_fr.calculate_rpm() * 0.10472
#         w_rl = self.enc_rl.calculate_rpm() * 0.10472
#         w_rr = self.enc_rr.calculate_rpm() * 0.10472
        
#         # Velocidades lineales de cada rueda
#         v_fl = w_fl * self.wheel_radius
#         v_fr = w_fr * self.wheel_radius
#         v_rl = w_rl * self.wheel_radius
#         v_rr = w_rr * self.wheel_radius
        
#         # ========== 3. CINEMÁTICA MECANUM ==========
#         # Velocidad en X (adelante/atrás)
#         vx_enc_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        
#         # Velocidad en Y (lateral) - con corrección de signo
#         vy_enc_raw = (-v_fl + v_fr - v_rl + v_rr) / 4.0
        
#         # Omega de encoders (solo para referencia, NO se usa)
#         omega_enc_raw = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
#         # ========== 4. FILTRO DE OUTLIERS ==========
#         vx_enc = self.reject_outlier(vx_enc_raw, self.last_vx_raw, self.max_vel_change)
#         vy_enc = self.reject_outlier(vy_enc_raw, self.last_vy_raw, self.max_vel_change)
        
#         self.last_vx_raw = vx_enc
#         self.last_vy_raw = vy_enc
        
#         # ========== 5. DETECTAR MODO (con omega del gyro) ==========
#         # Usar valores filtrados previos para detección de modo
#         current_mode = self.detect_motion_mode(self.vx_filtered, self.vy_filtered, 
#                                                self.omega_filtered)
        
#         # ========== 6. FILTRADO ADAPTATIVO ==========
#         # Obtener parámetros según modo
#         alpha_vx = self.alpha[current_mode]['vx']
#         alpha_vy = self.alpha[current_mode]['vy']
#         alpha_omega = self.alpha[current_mode]['omega']
        
#         buffer_vx = self.buffer_size[current_mode]['vx']
#         buffer_vy = self.buffer_size[current_mode]['vy']
#         buffer_omega = self.buffer_size[current_mode]['omega']
        
#         # Paso 1: Promedio móvil
#         vx_avg = self.moving_average(self.vx_buffer, vx_enc, buffer_vx)
#         vy_avg = self.moving_average(self.vy_buffer, vy_enc, buffer_vy)
#         omega_avg = self.moving_average(self.omega_buffer, omega_gyro, buffer_omega)
        
#         # Paso 2: Filtro pasa-bajos exponencial
#         self.vx_filtered = self.exponential_filter(vx_avg, self.vx_filtered, alpha_vx)
#         self.vy_filtered = self.exponential_filter(vy_avg, self.vy_filtered, alpha_vy)
#         self.omega_filtered = self.exponential_filter(omega_avg, self.omega_filtered, alpha_omega)
        
#         # ========== 7. SATURACIÓN Y DEADZONE ==========
#         self.vx_filtered = self.saturate(self.vx_filtered, self.MAX_LINEAR_VEL)
#         self.vy_filtered = self.saturate(self.vy_filtered, self.MAX_LINEAR_VEL)
#         self.omega_filtered = self.saturate(self.omega_filtered, self.MAX_ANGULAR_VEL)
        
#         self.vx_filtered = self.apply_deadzone(self.vx_filtered, self.DEADZONE_LINEAR)
#         self.vy_filtered = self.apply_deadzone(self.vy_filtered, self.DEADZONE_LINEAR)
#         self.omega_filtered = self.apply_deadzone(self.omega_filtered, self.DEADZONE_ANGULAR)
        
#         # ========== 8. INTEGRACIÓN DE ORIENTACIÓN ==========
#         # Integrar omega para obtener theta (orientación)
#         self.theta += self.omega_filtered * dt
        
#         # Normalizar theta a [-π, π]
#         self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
#         # ========== 9. TRANSFORMACIÓN AL MARCO GLOBAL ==========
#         # Convertir velocidades del marco del robot al marco global (odom)
#         cos_theta = math.cos(self.theta)
#         sin_theta = math.sin(self.theta)
        
#         vx_global = self.vx_filtered * cos_theta - self.vy_filtered * sin_theta
#         vy_global = self.vx_filtered * sin_theta + self.vy_filtered * cos_theta
        
#         # ========== 10. INTEGRACIÓN DE POSICIÓN ==========
#         self.x += vx_global * dt
#         self.y += vy_global * dt
        
#         # Actualizar velocidades para el mensaje de odometría
#         self.vx = self.vx_filtered
#         self.vy = self.vy_filtered
#         self.omega = self.omega_filtered
        
#         # ========== 11. ESTADÍSTICAS ==========
#         dx = self.x - self.last_x
#         dy = self.y - self.last_y
#         self.total_distance += math.sqrt(dx**2 + dy**2)
#         self.last_x = self.x
#         self.last_y = self.y
        
#         # ========== 12. PUBLICAR ==========
#         self.publish_odometry(current_time)
#         self.publish_mode(current_mode)
#         self.publish_diagnostics(vx_enc_raw, vy_enc_raw, omega_enc_raw, omega_gyro_raw)
        
#         # ========== 13. LOGS ==========
#         self.log_counter += 1
#         if self.log_counter % 100 == 0:
#             self.get_logger().info(
#                 f"🎯 [{current_mode:8s}] "
#                 f"Pos: ({self.x:.3f}, {self.y:.3f}) "
#                 f"θ: {math.degrees(self.theta):6.1f}° | "
#                 f"v: ({self.vx:.3f}, {self.vy:.3f}) m/s | "
#                 f"ω: {math.degrees(self.omega):5.1f}°/s | "
#                 f"Dist: {self.total_distance:.2f}m"
#             )
            
#             # Info de encoders
#             self.get_logger().info(
#                 f"   Encoders - FL:{self.enc_fl.counter} FR:{self.enc_fr.counter} "
#                 f"RL:{self.enc_rl.counter} RR:{self.enc_rr.counter}"
#             )
        
#         # Cambio de modo
#         if current_mode != self.motion_mode:
#             self.motion_mode = current_mode
#             self.get_logger().info(f"🔄 Cambio de modo: {current_mode}")
        
#         self.last_time = current_time

#     def publish_odometry(self, timestamp):
#         """Publica mensaje de odometría"""
#         msg = Odometry()
#         msg.header.stamp = timestamp.to_msg()
#         msg.header.frame_id = "odom"
#         msg.child_frame_id = "base_link"
        
#         # Posición
#         msg.pose.pose.position.x = float(self.x)
#         msg.pose.pose.position.y = float(self.y)
#         msg.pose.pose.position.z = 0.0
        
#         # Orientación (quaternion)
#         msg.pose.pose.orientation.x = 0.0
#         msg.pose.pose.orientation.y = 0.0
#         msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#         msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
#         # Covarianza de pose (aproximada según modo)
#         if self.motion_mode == "STATIC":
#             cov_xy = 0.001
#             cov_theta = 0.0005
#         elif self.motion_mode == "ROTATION":
#             cov_xy = 0.01
#             cov_theta = 0.001
#         elif self.motion_mode == "LINEAR":
#             cov_xy = 0.005
#             cov_theta = 0.002
#         else:  # MIXED
#             cov_xy = 0.008
#             cov_theta = 0.003
        
#         msg.pose.covariance[0] = cov_xy    # x
#         msg.pose.covariance[7] = cov_xy    # y
#         msg.pose.covariance[35] = cov_theta  # theta
        
#         # Velocidades
#         msg.twist.twist.linear.x = float(self.vx)
#         msg.twist.twist.linear.y = float(self.vy)
#         msg.twist.twist.angular.z = float(self.omega)
        
#         # Covarianza de velocidad
#         msg.twist.covariance[0] = 0.01   # vx
#         msg.twist.covariance[7] = 0.01   # vy
#         msg.twist.covariance[35] = 0.005  # omega
        
#         self.odom_pub.publish(msg)

#     def publish_mode(self, mode):
#         """Publica modo de movimiento"""
#         msg = String()
#         msg.data = mode
#         self.mode_pub.publish(msg)

#     def publish_diagnostics(self, vx_enc_raw, vy_enc_raw, omega_enc_raw, omega_gyro_raw):
#         """Publica diagnósticos"""
#         # Velocidades de encoders (sin filtrar)
#         enc_msg = Twist()
#         enc_msg.linear.x = vx_enc_raw
#         enc_msg.linear.y = vy_enc_raw
#         enc_msg.angular.z = omega_enc_raw
#         self.encoder_vel_pub.publish(enc_msg)
        
#         # Velocidades de gyro (sin filtrar)
#         gyro_msg = Twist()
#         gyro_msg.angular.z = omega_gyro_raw
#         self.gyro_vel_pub.publish(gyro_msg)
        
#         # Diagnósticos generales
#         diag_msg = Float32MultiArray()
#         diag_msg.data = [
#             float(self.x),
#             float(self.y),
#             float(self.theta),
#             float(self.vx_filtered),
#             float(self.vy_filtered),
#             float(self.omega_filtered),
#             float(self.total_distance),
#             float(len(self.vx_buffer))  # Tamaño actual del buffer
#         ]
#         self.diagnostics_pub.publish(diag_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = AdaptiveOdometryNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("🛑 Nodo detenido")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Nodo de odometría SIMPLE y ROBUSTO para mecanum
Basado en diagnóstico exitoso - SIN rechazo agresivo de encoders
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

class SimpleRobustOdometryNode(Node):
    def __init__(self):
        super().__init__('simple_robust_odometry')
        
        # ========== PARÁMETROS ==========
        self.declare_parameter('wheel_radius', 0.072)  # ⚠️ CORREGIDO de 0.05
        self.declare_parameter('lx', 0.175)
        self.declare_parameter('ly', 0.15)
        self.declare_parameter('update_rate', 100.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        update_rate = self.get_parameter('update_rate').value
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando hardware...")
        
        # Configuración que funcionó en diagnóstico (invert=False para todos)
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=5.0)
        
        # ========== ESTADO ==========
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ========== FILTROS MODERADOS ==========
        # Buffers para promedio móvil (tamaño fijo)
        self.vx_buffer = deque(maxlen=8)
        self.vy_buffer = deque(maxlen=8)
        self.omega_buffer = deque(maxlen=6)
        
        # Coeficiente para filtro exponencial (alpha)
        # 0.5 = balance entre respuesta y suavidad
        self.alpha_linear = 0.5
        self.alpha_angular = 0.6
        
        # Valores filtrados previos
        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.omega_filtered = 0.0
        
        # ========== LÍMITES RAZONABLES ==========
        self.MAX_LINEAR_VEL = 0.6   # m/s
        self.MAX_ANGULAR_VEL = 3.5  # rad/s
        
        # Deadzones moderadas
        self.DEADZONE_LINEAR = 0.010   # 1 cm/s
        self.DEADZONE_ANGULAR = 0.020  # ~1.1°/s
        
        # ========== RECHAZO DE OUTLIERS MODERADO ==========
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0
        
        self.max_vel_change = 0.4    # m/s (moderado)
        self.max_omega_change = 1.2  # rad/s (moderado)
        
        # ========== DETECCIÓN DE MODO ==========
        self.motion_mode = "STATIC"
        self.LINEAR_THRESHOLD = 0.02   # m/s
        self.ROTATION_THRESHOLD = 0.10  # rad/s
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_pub = self.create_publisher(String, '/motion_mode', 10)
        self.vel_pub = self.create_publisher(Twist, '/odom_velocities', 10)
        self.diag_pub = self.create_publisher(Float32MultiArray, '/odom_diagnostics', 10)
        
        # ========== TIMER ==========
        self.create_timer(1.0/update_rate, self.update)
        self.last_time = self.get_clock().now()
        
        self.counter = 0
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info(f"✅ Odometría simple iniciada a {update_rate} Hz")
        self.get_logger().info(f"   Radio rueda: {self.wheel_radius} m")
        self.get_logger().info("   Filtrado: Promedio móvil + Exponencial moderado")

    def saturate(self, value, max_val):
        """Limita valor"""
        return np.clip(value, -max_val, max_val)

    def apply_deadzone(self, value, deadzone):
        """Zona muerta"""
        return 0.0 if abs(value) < deadzone else value

    def reject_outlier(self, new_val, last_val, max_change):
        """Rechaza cambios muy bruscos"""
        delta = new_val - last_val
        if abs(delta) > max_change:
            return last_val + np.sign(delta) * max_change
        return new_val

    def exponential_filter(self, new_val, last_filtered, alpha):
        """Filtro pasa-bajos exponencial"""
        return alpha * new_val + (1.0 - alpha) * last_filtered

    def moving_average(self, buffer, new_val):
        """Promedio móvil simple"""
        buffer.append(new_val)
        if len(buffer) > 0:
            return sum(buffer) / len(buffer)
        return 0.0

    def detect_mode(self, vx, vy, omega):
        """Detecta modo de movimiento"""
        linear_vel = math.sqrt(vx**2 + vy**2)
        angular_vel = abs(omega)
        
        is_linear = linear_vel > self.LINEAR_THRESHOLD
        is_rotating = angular_vel > self.ROTATION_THRESHOLD
        
        if not is_linear and not is_rotating:
            return "STATIC"
        elif is_rotating and not is_linear:
            return "ROTATION"
        elif is_linear and not is_rotating:
            return "LINEAR"
        else:
            return "MIXED"

    def update(self):
        """Ciclo principal"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0 or dt > 0.5:
            dt = 0.01
        
        # ========== 1. LEER ENCODERS ==========
        rpm_fl = self.enc_fl.calculate_rpm()
        rpm_fr = self.enc_fr.calculate_rpm()
        rpm_rl = self.enc_rl.calculate_rpm()
        rpm_rr = self.enc_rr.calculate_rpm()
        
        # Convertir a velocidades lineales
        w_fl = rpm_fl * 0.10472  # RPM to rad/s
        w_fr = rpm_fr * 0.10472
        w_rl = rpm_rl * 0.10472
        w_rr = rpm_rr * 0.10472
        
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # ========== 2. CINEMÁTICA MECANUM ==========
        vx_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_raw = (-v_fl + v_fr - v_rl + v_rr) / 4.0
        
        # ========== 3. LEER GYRO ==========
        imu_data = self.imu.update()
        omega_raw = imu_data['w_z']
        
        # ========== 4. RECHAZO DE OUTLIERS ==========
        vx_safe = self.reject_outlier(vx_raw, self.last_vx, self.max_vel_change)
        vy_safe = self.reject_outlier(vy_raw, self.last_vy, self.max_vel_change)
        omega_safe = self.reject_outlier(omega_raw, self.last_omega, self.max_omega_change)
        
        self.last_vx = vx_safe
        self.last_vy = vy_safe
        self.last_omega = omega_safe
        
        # ========== 5. PROMEDIO MÓVIL ==========
        vx_avg = self.moving_average(self.vx_buffer, vx_safe)
        vy_avg = self.moving_average(self.vy_buffer, vy_safe)
        omega_avg = self.moving_average(self.omega_buffer, omega_safe)
        
        # ========== 6. FILTRO EXPONENCIAL ==========
        self.vx_filtered = self.exponential_filter(vx_avg, self.vx_filtered, self.alpha_linear)
        self.vy_filtered = self.exponential_filter(vy_avg, self.vy_filtered, self.alpha_linear)
        self.omega_filtered = self.exponential_filter(omega_avg, self.omega_filtered, self.alpha_angular)
        
        # ========== 7. SATURACIÓN ==========
        self.vx_filtered = self.saturate(self.vx_filtered, self.MAX_LINEAR_VEL)
        self.vy_filtered = self.saturate(self.vy_filtered, self.MAX_LINEAR_VEL)
        self.omega_filtered = self.saturate(self.omega_filtered, self.MAX_ANGULAR_VEL)
        
        # ========== 8. DEADZONE ==========
        self.vx_filtered = self.apply_deadzone(self.vx_filtered, self.DEADZONE_LINEAR)
        self.vy_filtered = self.apply_deadzone(self.vy_filtered, self.DEADZONE_LINEAR)
        self.omega_filtered = self.apply_deadzone(self.omega_filtered, self.DEADZONE_ANGULAR)
        
        # ========== 9. INTEGRACIÓN DE ORIENTACIÓN ==========
        # SIEMPRE usar gyro (no lo rechazamos nunca)
        self.theta += self.omega_filtered * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # ========== 10. TRANSFORMAR A MARCO GLOBAL ==========
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        
        vx_global = self.vx_filtered * cos_t - self.vy_filtered * sin_t
        vy_global = self.vx_filtered * sin_t + self.vy_filtered * cos_t
        
        # ========== 11. INTEGRACIÓN DE POSICIÓN ==========
        self.x += vx_global * dt
        self.y += vy_global * dt
        
        # ========== 12. DETECTAR MODO ==========
        current_mode = self.detect_mode(self.vx_filtered, self.vy_filtered, self.omega_filtered)
        
        # ========== 13. ESTADÍSTICAS ==========
        dx = self.x - self.last_x
        dy = self.y - self.last_y
        self.total_distance += math.sqrt(dx**2 + dy**2)
        self.last_x = self.x
        self.last_y = self.y
        
        # ========== 14. PUBLICAR ==========
        self.publish_odometry(current_time)
        self.publish_mode(current_mode)
        self.publish_velocities(vx_raw, vy_raw, omega_raw)
        self.publish_diagnostics()
        
        # ========== 15. LOGS ==========
        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"📍 [{current_mode:8s}] "
                f"Pos: ({self.x:.3f}, {self.y:.3f}) "
                f"θ: {math.degrees(self.theta):6.1f}° | "
                f"v: ({self.vx_filtered:.3f}, {self.vy_filtered:.3f}) m/s | "
                f"ω: {math.degrees(self.omega_filtered):5.1f}°/s | "
                f"Dist: {self.total_distance:.2f}m"
            )
        
        if current_mode != self.motion_mode:
            self.motion_mode = current_mode
            self.get_logger().info(f"🔄 Modo: {current_mode}")
        
        self.last_time = current_time

    def publish_odometry(self, timestamp):
        """Publica odometría"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Covarianza básica
        msg.pose.covariance[0] = 0.005   # x
        msg.pose.covariance[7] = 0.005   # y
        msg.pose.covariance[35] = 0.002  # theta
        
        msg.twist.twist.linear.x = float(self.vx_filtered)
        msg.twist.twist.linear.y = float(self.vy_filtered)
        msg.twist.twist.angular.z = float(self.omega_filtered)
        
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.01
        msg.twist.covariance[35] = 0.005
        
        self.odom_pub.publish(msg)

    def publish_mode(self, mode):
        """Publica modo"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def publish_velocities(self, vx_raw, vy_raw, omega_raw):
        """Publica velocidades"""
        msg = Twist()
        msg.linear.x = vx_raw
        msg.linear.y = vy_raw
        msg.angular.z = omega_raw
        self.vel_pub.publish(msg)

    def publish_diagnostics(self):
        """Publica diagnósticos"""
        msg = Float32MultiArray()
        msg.data = [
            float(self.x), float(self.y), float(self.theta),
            float(self.vx_filtered), float(self.vy_filtered), float(self.omega_filtered),
            float(self.total_distance),
            float(len(self.vx_buffer))
        ]
        self.diag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobustOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()