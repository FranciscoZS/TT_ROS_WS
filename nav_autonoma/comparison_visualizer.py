#!/usr/bin/env python3
"""
Visualizador comparativo actualizado:
- Azul: Solo Encoders
- Rojo: Encoders + Gyro (sin EKF)
- Verde: EKF Fusionado
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class ComparisonVisualizerNode(Node):
    def __init__(self):
        super().__init__('comparison_visualizer_node')
        
        # Trayectorias
        self.enc_x = []
        self.enc_y = []
        self.enc_theta = []
        
        self.hybrid_x = []
        self.hybrid_y = []
        self.hybrid_theta = []
        
        self.ekf_x = []
        self.ekf_y = []
        self.ekf_theta = []
        
        self.timestamps = []
        
        # Suscriptores (nombres actualizados)
        self.create_subscription(Odometry, '/odom_encoders_only', self.enc_callback, 10)
        self.create_subscription(Odometry, '/odom_encoders_gyro', self.hybrid_callback, 10)
        self.create_subscription(Odometry, '/odom_ekf_fused', self.ekf_callback, 10)
        
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('📊 Grabando comparación de 3 sistemas...')
        self.get_logger().info('   🔵 Solo Encoders:    /odom_encoders_only')
        self.get_logger().info('   🔴 Encoders + Gyro:  /odom_encoders_gyro')
        self.get_logger().info('   🟢 EKF Fusionado:    /odom_ekf_fused')
        self.get_logger().info('   Presiona Ctrl+C para generar gráficas')

    def enc_callback(self, msg):
        """Guarda datos de encoders"""
        self.enc_x.append(msg.pose.pose.position.x)
        self.enc_y.append(msg.pose.pose.position.y)
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        self.enc_theta.append(theta)

    def hybrid_callback(self, msg):
        """Guarda datos de híbrido (encoders + gyro)"""
        self.hybrid_x.append(msg.pose.pose.position.x)
        self.hybrid_y.append(msg.pose.pose.position.y)
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        self.hybrid_theta.append(theta)

    def ekf_callback(self, msg):
        """Guarda datos de EKF"""
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        self.ekf_theta.append(theta)
        
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        self.timestamps.append(elapsed)

    def generate_comparison(self):
        """Genera análisis comparativo"""
        self.get_logger().info('📈 Generando comparación...')
        
        min_length = min(len(self.enc_x), len(self.hybrid_x), len(self.ekf_x))
        
        if min_length < 10:
            self.get_logger().warn('⚠️  Muy pocos datos')
            return
        
        # Recortar
        enc_x = np.array(self.enc_x[:min_length])
        enc_y = np.array(self.enc_y[:min_length])
        enc_theta = np.array(self.enc_theta[:min_length])
        
        hyb_x = np.array(self.hybrid_x[:min_length])
        hyb_y = np.array(self.hybrid_y[:min_length])
        hyb_theta = np.array(self.hybrid_theta[:min_length])
        
        ekf_x = np.array(self.ekf_x[:min_length])
        ekf_y = np.array(self.ekf_y[:min_length])
        ekf_theta = np.array(self.ekf_theta[:min_length])
        
        t = np.array(self.timestamps[:min_length])
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # ========== GRÁFICAS ==========
        fig = plt.figure(figsize=(18, 10))
        
        # 1. TRAYECTORIAS 2D
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(enc_x, enc_y, 'b-', linewidth=2, label='🔵 Solo Encoders', alpha=0.7)
        ax1.plot(hyb_x, hyb_y, 'r--', linewidth=2, label='🔴 Encoders + Gyro', alpha=0.7)
        ax1.plot(ekf_x, ekf_y, 'g-', linewidth=2.5, label='🟢 EKF Fusionado', alpha=0.9)
        
        ax1.plot(enc_x[0], enc_y[0], 'ko', markersize=8, label='Start')
        ax1.plot(hyb_x[-1], hyb_y[-1], 'ro', markersize=8, label='End (Enc+Gyro)')
        ax1.plot(ekf_x[-1], ekf_y[-1], 'g*', markersize=12, label='End (EKF)')
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title(f'Comparación de Trayectorias\nMuestras: {min_length}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. POSICIÓN X
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(t, enc_x, 'b-', label='Encoders', linewidth=1.5)
        ax2.plot(t, hyb_x, 'r--', label='Enc+Gyro', linewidth=1.5)
        ax2.plot(t, ekf_x, 'g-', label='EKF', linewidth=2)
        ax2.set_xlabel('Tiempo [s]')
        ax2.set_ylabel('X [m]')
        ax2.set_title('Posición X vs Tiempo')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. POSICIÓN Y
        ax3 = plt.subplot(2, 3, 3)
        ax3.plot(t, enc_y, 'b-', label='Encoders', linewidth=1.5)
        ax3.plot(t, hyb_y, 'r--', label='Enc+Gyro', linewidth=1.5)
        ax3.plot(t, ekf_y, 'g-', label='EKF', linewidth=2)
        ax3.set_xlabel('Tiempo [s]')
        ax3.set_ylabel('Y [m]')
        ax3.set_title('Posición Y vs Tiempo')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. ORIENTACIÓN
        ax4 = plt.subplot(2, 3, 4)
        ax4.plot(t, np.degrees(enc_theta), 'b-', label='Encoders', linewidth=1.5)
        ax4.plot(t, np.degrees(hyb_theta), 'r--', label='Enc+Gyro', linewidth=1.5)
        ax4.plot(t, np.degrees(ekf_theta), 'g-', label='EKF', linewidth=2)
        ax4.set_xlabel('Tiempo [s]')
        ax4.set_ylabel('Orientación [°]')
        ax4.set_title('Orientación vs Tiempo')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. ERROR RESPECTO A EKF
        ax5 = plt.subplot(2, 3, 5)
        enc_error = np.sqrt((enc_x - ekf_x)**2 + (enc_y - ekf_y)**2)
        hyb_error = np.sqrt((hyb_x - ekf_x)**2 + (hyb_y - ekf_y)**2)
        
        ax5.plot(t, enc_error, 'b-', label='Error Encoders', linewidth=1.5)
        ax5.plot(t, hyb_error, 'r--', label='Error Enc+Gyro', linewidth=1.5)
        ax5.set_xlabel('Tiempo [s]')
        ax5.set_ylabel('Error [m]')
        ax5.set_title('Error de Posición (respecto a EKF)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. ESTADÍSTICAS
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')
        
        stats_text = self.calculate_statistics(
            enc_x, enc_y, enc_theta,
            hyb_x, hyb_y, hyb_theta,
            ekf_x, ekf_y, ekf_theta,
            t
        )
        
        ax6.text(0.1, 0.9, stats_text, transform=ax6.transAxes,
                fontsize=10, verticalalignment='top',
                fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        filename = f"odometry_comparison_{timestamp}.png"
        plt.savefig(filename, dpi=300)
        self.get_logger().info(f'✅ Comparación guardada: {filename}')
        
        plt.close()

    def calculate_statistics(self, enc_x, enc_y, enc_theta,
                            hyb_x, hyb_y, hyb_theta,
                            ekf_x, ekf_y, ekf_theta, t):
        """Calcula estadísticas"""
        
        enc_dist = np.sum(np.sqrt(np.diff(enc_x)**2 + np.diff(enc_y)**2))
        hyb_dist = np.sum(np.sqrt(np.diff(hyb_x)**2 + np.diff(hyb_y)**2))
        ekf_dist = np.sum(np.sqrt(np.diff(ekf_x)**2 + np.diff(ekf_y)**2))
        
        enc_error = np.sqrt((enc_x - ekf_x)**2 + (enc_y - ekf_y)**2)
        hyb_error = np.sqrt((hyb_x - ekf_x)**2 + (hyb_y - ekf_y)**2)
        
        enc_theta_error = np.abs(enc_theta - ekf_theta)
        hyb_theta_error = np.abs(hyb_theta - ekf_theta)
        
        enc_closure = np.sqrt((enc_x[-1] - enc_x[0])**2 + (enc_y[-1] - enc_y[0])**2)
        hyb_closure = np.sqrt((hyb_x[-1] - hyb_x[0])**2 + (hyb_y[-1] - hyb_y[0])**2)
        ekf_closure = np.sqrt((ekf_x[-1] - ekf_x[0])**2 + (ekf_y[-1] - ekf_y[0])**2)
        
        stats = f"""
╔════════════════════════════════════════╗
║   ESTADÍSTICAS COMPARATIVAS            ║
╠════════════════════════════════════════╣
║ Duración: {t[-1]:.1f} s                    
║ Muestras: {len(ekf_x)}                      
║                                        ║
║ DISTANCIA RECORRIDA:                   ║
║   🔵 Encoders:    {enc_dist:.3f} m            
║   🔴 Enc+Gyro:    {hyb_dist:.3f} m            
║   🟢 EKF:         {ekf_dist:.3f} m            
║                                        ║
║ ERROR MEDIO (vs EKF):                  ║
║   🔵 Encoders:    {np.mean(enc_error):.4f} m      
║   🔴 Enc+Gyro:    {np.mean(hyb_error):.4f} m      
║                                        ║
║ ERROR MÁXIMO (vs EKF):                 ║
║   🔵 Encoders:    {np.max(enc_error):.4f} m       
║   🔴 Enc+Gyro:    {np.max(hyb_error):.4f} m       
║                                        ║
║ ERROR ORIENTACIÓN (RMS):               ║
║   🔵 Encoders:    {np.sqrt(np.mean(enc_theta_error**2))*180/np.pi:.2f}°       
║   🔴 Enc+Gyro:    {np.sqrt(np.mean(hyb_theta_error**2))*180/np.pi:.2f}°       
║                                        ║
║ ERROR DE CIERRE:                       ║
║   🔵 Encoders:    {enc_closure:.4f} m             
║   🔴 Enc+Gyro:    {hyb_closure:.4f} m             
║   🟢 EKF:         {ekf_closure:.4f} m             
║                                        ║
║ CONCLUSIÓN:                            ║
║   El sistema Enc+Gyro debería tener    ║
║   mejor orientación que Encoders       ║
║   (reduce patinaje en rotaciones)      ║
╚════════════════════════════════════════╝
        """
        
        return stats

def main(args=None):
    rclpy.init(args=args)
    node = ComparisonVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛑 Deteniendo...')
        node.generate_comparison()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()