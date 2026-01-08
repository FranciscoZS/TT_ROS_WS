#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class MapAnalysisNode(Node):
    def __init__(self):
        super().__init__('map_analysis_node')
        
        # Trayectorias
        self.odom_x = []
        self.odom_y = []
        self.odom_theta = []
        self.timestamps = []
        
        # Diagnósticos
        self.cov_x = []
        self.cov_y = []
        self.cov_theta = []
        
        # Velocidades
        self.vx_ekf = []
        self.vy_ekf = []
        self.omega_ekf = []
        
        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            Float32MultiArray, 
            '/odom_diagnostics', 
            self.diagnostics_callback, 
            10
        )
        
        self.start_time = self.get_clock().now()
        self.get_logger().info('📊 Grabando trayectoria y diagnósticos...')
        self.get_logger().info('   Presiona Ctrl+C para generar análisis completo')

    def odom_callback(self, msg):
        """Guarda datos de odometría"""
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        
        # Extraer theta del quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        self.odom_theta.append(theta)
        
        # Velocidades
        self.vx_ekf.append(msg.twist.twist.linear.x)
        self.vy_ekf.append(msg.twist.twist.linear.y)
        self.omega_ekf.append(msg.twist.twist.angular.z)
        
        # Tiempo relativo
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        self.timestamps.append(elapsed)

    def diagnostics_callback(self, msg):
        """Guarda datos de diagnóstico"""
        if len(msg.data) >= 6:
            self.cov_x.append(msg.data[3])
            self.cov_y.append(msg.data[4])
            self.cov_theta.append(msg.data[5])

    def generate_analysis(self):
        """Genera análisis completo con múltiples gráficas"""
        self.get_logger().info('📈 Generando análisis de odometría...')
        
        if len(self.odom_x) < 10:
            self.get_logger().warn('⚠️  Muy pocos datos para analizar')
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Convertir a arrays numpy
        x = np.array(self.odom_x)
        y = np.array(self.odom_y)
        theta = np.array(self.odom_theta)
        t = np.array(self.timestamps)
        
        # Crear figura con 6 subplots
        fig = plt.figure(figsize=(16, 12))
        
        # ========== 1. TRAYECTORIA 2D ==========
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(x, y, 'b-', linewidth=2, label='EKF Odometry')
        ax1.plot(x[0], y[0], 'go', markersize=10, label='Start')
        ax1.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title(f'Trayectoria 2D\nMuestras: {len(x)}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # ========== 2. POSICIÓN vs TIEMPO ==========
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(t, x, 'b-', label='X', linewidth=1.5)
        ax2.plot(t, y, 'r-', label='Y', linewidth=1.5)
        ax2.set_xlabel('Tiempo [s]')
        ax2.set_ylabel('Posición [m]')
        ax2.set_title('Posición vs Tiempo')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # ========== 3. ORIENTACIÓN vs TIEMPO ==========
        ax3 = plt.subplot(2, 3, 3)
        theta_deg = np.degrees(theta)
        ax3.plot(t, theta_deg, 'g-', linewidth=1.5)
        ax3.set_xlabel('Tiempo [s]')
        ax3.set_ylabel('Orientación [°]')
        ax3.set_title('Orientación vs Tiempo')
        ax3.grid(True, alpha=0.3)
        
        # ========== 4. VELOCIDADES ==========
        ax4 = plt.subplot(2, 3, 4)
        if len(self.vx_ekf) > 0:
            vx = np.array(self.vx_ekf)
            vy = np.array(self.vy_ekf)
            ax4.plot(t[:len(vx)], vx, 'b-', label='Vx', linewidth=1.5)
            ax4.plot(t[:len(vy)], vy, 'r-', label='Vy', linewidth=1.5)
            ax4.set_xlabel('Tiempo [s]')
            ax4.set_ylabel('Velocidad [m/s]')
            ax4.set_title('Velocidades Lineales')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        
        # ========== 5. VELOCIDAD ANGULAR ==========
        ax5 = plt.subplot(2, 3, 5)
        if len(self.omega_ekf) > 0:
            omega = np.array(self.omega_ekf)
            omega_deg = np.degrees(omega)
            ax5.plot(t[:len(omega)], omega_deg, 'g-', linewidth=1.5)
            ax5.set_xlabel('Tiempo [s]')
            ax5.set_ylabel('Velocidad Angular [°/s]')
            ax5.set_title('Velocidad Angular')
            ax5.grid(True, alpha=0.3)
        
        # ========== 6. COVARIANZA (INCERTIDUMBRE) ==========
        ax6 = plt.subplot(2, 3, 6)
        if len(self.cov_x) > 0:
            cov_x = np.array(self.cov_x)
            cov_y = np.array(self.cov_y)
            cov_th = np.array(self.cov_theta)
            t_cov = np.linspace(0, t[-1], len(cov_x))
            
            ax6.plot(t_cov, cov_x, 'b-', label='σ²_x', linewidth=1.5)
            ax6.plot(t_cov, cov_y, 'r-', label='σ²_y', linewidth=1.5)
            ax6.plot(t_cov, cov_th*10, 'g-', label='σ²_θ (×10)', linewidth=1.5)
            ax6.set_xlabel('Tiempo [s]')
            ax6.set_ylabel('Covarianza')
            ax6.set_title('Incertidumbre del EKF')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
            ax6.set_yscale('log')
        
        plt.tight_layout()
        
        # Guardar
        filename = f"/home/orangepi/odometry_graf/odometry_analysis_{timestamp}.png"
        plt.savefig(filename, dpi=300)
        self.get_logger().info(f'✅ Análisis guardado: {filename}')
        
        # ========== ESTADÍSTICAS ==========
        self.print_statistics(x, y, theta)
        
        plt.close()

    def print_statistics(self, x, y, theta):
        """Imprime estadísticas de la trayectoria"""
        # Distancia total recorrida
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx**2 + dy**2)
        total_distance = np.sum(distances)
        
        # Rango de movimiento
        x_range = np.max(x) - np.min(x)
        y_range = np.max(y) - np.min(y)
        
        # Error de cierre (closure error)
        closure_error = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
        
        # Rotación total
        total_rotation = np.sum(np.abs(np.diff(theta)))
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('📊 ESTADÍSTICAS DE ODOMETRÍA')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Muestras recolectadas: {len(x)}')
        self.get_logger().info(f'Duración: {self.timestamps[-1]:.2f} s')
        self.get_logger().info(f'Distancia recorrida: {total_distance:.3f} m')
        self.get_logger().info(f'Rango X: {x_range:.3f} m')
        self.get_logger().info(f'Rango Y: {y_range:.3f} m')
        self.get_logger().info(f'Error de cierre: {closure_error:.3f} m')
        self.get_logger().info(f'Rotación total: {np.degrees(total_rotation):.1f}°')
        
        if len(self.cov_x) > 0:
            avg_cov_x = np.mean(self.cov_x)
            avg_cov_y = np.mean(self.cov_y)
            self.get_logger().info(f'Covarianza prom. X: {avg_cov_x:.6f}')
            self.get_logger().info(f'Covarianza prom. Y: {avg_cov_y:.6f}')
        
        self.get_logger().info('='*50 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = MapAnalysisNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛑 Deteniendo grabación...')
        node.generate_analysis()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
