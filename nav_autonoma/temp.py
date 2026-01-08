#!/usr/bin/env python3
"""
Nodo de diagnóstico RAW - SIN FILTROS
Para ver exactamente qué leen los encoders y gyro
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
import math

class RawDiagnosticNode(Node):
    def __init__(self):
        super().__init__('raw_diagnostic_node')
        
        # ========== PARÁMETROS ==========
        self.wheel_radius = 0.04
        self.lx = 0.175
        self.ly = 0.15
        
        # ========== HARDWARE ==========
        self.get_logger().info("🔧 Inicializando hardware...")
        
        # CRÍTICO: Prueba TODAS las combinaciones posibles de inversión
        # Configuración 1 (DEFAULT - lo que tienes ahora)
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        self.imu = MPU6050(node=self)
        self.imu.calibrate(duration=3.0)
        
        # ========== ESTADO SIN FILTROS ==========
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ========== PUBLISHERS ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.raw_data_pub = self.create_publisher(Float32MultiArray, '/raw_encoder_data', 10)
        self.diagnostic_msg_pub = self.create_publisher(String, '/diagnostic_output', 10)
        
        # Timer a 50Hz para ver datos claramente
        self.create_timer(0.02, self.update)
        self.last_time = self.get_clock().now()
        
        self.counter = 0
        self.total_dx = 0.0
        self.total_dy = 0.0
        
        self.get_logger().info("✅ Nodo RAW iniciado - SIN FILTROS")
        self.get_logger().info("⚠️  Todos los datos son CRUDOS")

    def update(self):
        """Actualización SIN FILTROS - datos puros"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        
        # ========== LEER ENCODERS (CRUDOS) ==========
        rpm_fl = self.enc_fl.calculate_rpm()
        rpm_fr = self.enc_fr.calculate_rpm()
        rpm_rl = self.enc_rl.calculate_rpm()
        rpm_rr = self.enc_rr.calculate_rpm()
        
        # Convertir a rad/s
        w_fl = rpm_fl * 0.10472
        w_fr = rpm_fr * 0.10472
        w_rl = rpm_rl * 0.10472
        w_rr = rpm_rr * 0.10472
        
        # Velocidades lineales de cada rueda
        v_fl = w_fl * self.wheel_radius
        v_fr = w_fr * self.wheel_radius
        v_rl = w_rl * self.wheel_radius
        v_rr = w_rr * self.wheel_radius
        
        # ========== LEER GYRO (CRUDO) ==========
        imu_data = self.imu.update()
        omega_gyro = imu_data['w_z']
        
        # ========== CINEMÁTICA DIRECTA ==========
        # FÓRMULAS ESTÁNDAR MECANUM
        vx_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_raw = (-v_fl + v_fr - v_rl + v_rr) / 4.0
        omega_enc_raw = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.lx + self.ly))
        
        # ========== INTEGRACIÓN SIMPLE ==========
        # Usar omega del GYRO (más confiable)
        self.theta += omega_gyro * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Transformar a frame global
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        
        vx_global = vx_raw * cos_theta - vy_raw * sin_theta
        vy_global = vx_raw * sin_theta + vy_raw * cos_theta
        
        # Integrar posición
        dx = vx_global * dt
        dy = vy_global * dt
        
        self.x += dx
        self.y += dy
        
        self.total_dx += abs(dx)
        self.total_dy += abs(dy)
        
        # ========== PUBLICAR ODOMETRÍA ==========
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom_msg.twist.twist.linear.x = float(vx_raw)
        odom_msg.twist.twist.linear.y = float(vy_raw)
        odom_msg.twist.twist.angular.z = float(omega_gyro)
        
        self.odom_pub.publish(odom_msg)
        
        # ========== PUBLICAR DATOS CRUDOS ==========
        raw_msg = Float32MultiArray()
        raw_msg.data = [
            float(rpm_fl), float(rpm_fr), float(rpm_rl), float(rpm_rr),
            float(v_fl), float(v_fr), float(v_rl), float(v_rr),
            float(vx_raw), float(vy_raw), float(omega_enc_raw), float(omega_gyro),
            float(self.x), float(self.y), float(self.theta)
        ]
        self.raw_data_pub.publish(raw_msg)
        
        # ========== LOGS DETALLADOS ==========
        self.counter += 1
        
        if self.counter % 50 == 0:  # Cada segundo (50Hz)
            self.get_logger().info("="*70)
            self.get_logger().info(f"⏱️  t = {self.counter * 0.02:.1f}s")
            self.get_logger().info(f"")
            self.get_logger().info(f"📊 ENCODERS (RPM):")
            self.get_logger().info(f"   FL: {rpm_fl:7.1f}  |  FR: {rpm_fr:7.1f}")
            self.get_logger().info(f"   RL: {rpm_rl:7.1f}  |  RR: {rpm_rr:7.1f}")
            
            self.get_logger().info(f"")
            self.get_logger().info(f"🎯 VELOCIDADES RUEDAS (m/s):")
            self.get_logger().info(f"   v_FL: {v_fl:+7.4f}  |  v_FR: {v_fr:+7.4f}")
            self.get_logger().info(f"   v_RL: {v_rl:+7.4f}  |  v_RR: {v_rr:+7.4f}")
            
            self.get_logger().info(f"")
            self.get_logger().info(f"🚗 CINEMÁTICA ROBOT:")
            self.get_logger().info(f"   VX (adelante): {vx_raw:+7.4f} m/s")
            self.get_logger().info(f"   VY (lateral):  {vy_raw:+7.4f} m/s")
            self.get_logger().info(f"   Omega (ENC):   {omega_enc_raw:+7.4f} rad/s = {math.degrees(omega_enc_raw):+6.1f}°/s")
            self.get_logger().info(f"   Omega (GYRO):  {omega_gyro:+7.4f} rad/s = {math.degrees(omega_gyro):+6.1f}°/s")
            
            self.get_logger().info(f"")
            self.get_logger().info(f"📍 POSICIÓN INTEGRADA:")
            self.get_logger().info(f"   X:     {self.x:+7.3f} m  (Δtotal: {self.total_dx:.3f} m)")
            self.get_logger().info(f"   Y:     {self.y:+7.3f} m  (Δtotal: {self.total_dy:.3f} m)")
            self.get_logger().info(f"   Theta: {math.degrees(self.theta):+7.1f}°")
            
            # DIAGNÓSTICO AUTOMÁTICO
            self.get_logger().info(f"")
            self.get_logger().info(f"🔍 DIAGNÓSTICO:")
            
            # 1. Detectar si todas las ruedas tienen mismo signo
            signs = [1 if v > 0.01 else (-1 if v < -0.01 else 0) 
                    for v in [v_fl, v_fr, v_rl, v_rr]]
            non_zero = [s for s in signs if s != 0]
            
            if len(non_zero) > 0:
                if all(s == non_zero[0] for s in non_zero):
                    self.get_logger().info(f"   ✅ Todas las ruedas mismo signo: {'+' if non_zero[0] > 0 else '-'}")
                else:
                    self.get_logger().info(f"   ❌ RUEDAS CON SIGNOS MEZCLADOS!")
                    self.get_logger().info(f"      Signos: FL={signs[0]:+d} FR={signs[1]:+d} RL={signs[2]:+d} RR={signs[3]:+d}")
                    self.get_logger().info(f"      👉 NECESITAS INVERTIR ALGUNOS ENCODERS")
            
            # 2. Detectar si VX es muy bajo cuando hay movimiento
            wheel_activity = sum(abs(v) for v in [v_fl, v_fr, v_rl, v_rr])
            if wheel_activity > 0.1 and abs(vx_raw) < 0.02:
                self.get_logger().info(f"   ⚠️  Ruedas activas pero VX bajo!")
                self.get_logger().info(f"      Actividad ruedas: {wheel_activity:.3f} m/s")
                self.get_logger().info(f"      VX resultante: {vx_raw:.3f} m/s")
                self.get_logger().info(f"      👉 VELOCIDADES SE CANCELAN - Invertir encoders")
            
            # 3. Detectar drift de theta
            if abs(omega_gyro) < 0.05 and abs(omega_enc_raw) > 0.2:
                self.get_logger().info(f"   ⚠️  Gyro estático pero encoders detectan rotación")
                self.get_logger().info(f"      👉 Ruido en encoders durante movimiento lineal")
            
            self.get_logger().info("="*70)
            self.get_logger().info("")
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*70)
    print("🚨 NODO DE DIAGNÓSTICO RAW (SIN FILTROS)")
    print("="*70)
    print("Este nodo muestra datos CRUDOS sin ningún filtrado.")
    print("Úsalo para:")
    print("  1. Ver si los encoders leen correctamente")
    print("  2. Identificar inversiones necesarias")
    print("  3. Verificar cinemática básica")
    print("")
    print("INSTRUCCIONES:")
    print("  - Observa los logs cada segundo")
    print("  - Mueve el robot ADELANTE y verifica signos")
    print("  - Si VX ≈ 0 con ruedas activas → encoders se cancelan")
    print("="*70)
    print("")
    
    node = RawDiagnosticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()