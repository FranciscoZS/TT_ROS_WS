# #!/usr/bin/env python3
# """
# Nodo seguidor de trayectoria SIMPLE para robot mecanum - VERSIÓN CORREGIDA
# Corrige el problema de rotación infinita
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32MultiArray
# from nav_msgs.msg import Odometry
# import math
# import numpy as np
# import matplotlib.pyplot as plt
# from datetime import datetime

# class SimplePathFollower(Node):
#     def __init__(self):
#         super().__init__('simple_path_follower')
        
#         # ========== PARÁMETROS ==========
#         self.declare_parameter('distance_forward', 0.8)  # 30 cm
#         self.declare_parameter('distance_side', 0.6)     # 30 cm
#         self.declare_parameter('rotation_angle', 90.0)   # 90 grados
#         self.declare_parameter('speed_percent', 50)      # % velocidad
#         self.declare_parameter('speed_percent_rotate', 40)      # % velocidad rotación
#         self.declare_parameter('pos_tolerance', 0.02)    # 2 cm
#         self.declare_parameter('angle_tolerance', 5.0)   # 5 grados
        
#         # ========== VALORES ==========
#         self.distance_forward = self.get_parameter('distance_forward').value #/ 3.125
#         self.distance_side = self.get_parameter('distance_side').value #/ 3.125
#         self.rotation_angle = math.radians(self.get_parameter('rotation_angle').value)
#         self.speed_percent = self.get_parameter('speed_percent').value
#         self.speed_percent_rotate = self.get_parameter('speed_percent_rotate').value
#         self.pos_tolerance = self.get_parameter('pos_tolerance').value
#         self.angle_tolerance = math.radians(self.get_parameter('angle_tolerance').value)
        
#         # ========== ESTADO ==========
#         self.state = "IDLE"  # IDLE, FORWARD, ROTATING, SIDE, DONE
#         self.current_pose = None
#         self.start_pose = None
#         self.movement_start_pose = None
#         self.command_active = False
        
#         # ========== TIMEOUTS DE SEGURIDAD ==========
#         self.movement_start_time = None
#         self.FORWARD_TIMEOUT = 100.0  # segundos
#         self.ROTATION_TIMEOUT = 20.0  # segundos
#         self.SIDE_TIMEOUT = 100.0  # segundos
        
#         # ========== DATOS PARA ANÁLISIS ==========
#         self.path_history = []
#         self.errors = []
#         self.start_time = None
        
#         # ========== SUSCRIPTORES ==========
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
        
#         self.cmd_sub = self.create_subscription(
#             String,
#             '/path_control',
#             self.control_callback,
#             10
#         )
        
#         # ========== PUBLICADORES ==========
#         self.motor_pub = self.create_publisher(String, 'motor_command', 10)
#         self.status_pub = self.create_publisher(String, '/path_status', 10)
#         self.error_pub = self.create_publisher(Float32MultiArray, '/path_error', 10)
        
#         # ========== TIMER ==========
#         self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
#         self.get_logger().info(f"""
#         🤖 Seguidor de trayectoria SIMPLE (CORREGIDO) inicializado
#         Trayectoria en L:
#           1. Adelante: {self.distance_forward*100:.0f} cm
#           2. Giro: {math.degrees(self.rotation_angle):.0f}° horario
#           3. Lateral: {self.distance_side*100:.0f} cm
        
#         Comandos por topic /path_control:
#           - "START" : Comenzar
#           - "STOP"  : Detener
#           - "RESET" : Reiniciar
#         """)
        
#     def normalize_angle(self, angle):
#         """Normaliza ángulo a rango [-π, π]"""
#         while angle > math.pi:
#             angle -= 2.0 * math.pi
#         while angle < -math.pi:
#             angle += 2.0 * math.pi
#         return angle
        
#     def odom_callback(self, msg):
#         """Actualiza posición actual"""
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
        
#         q = msg.pose.pose.orientation
#         theta = math.atan2(2.0*(q.w*q.z + q.x*q.y),
#                          1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
#         self.current_pose = {'x': x, 'y': y, 'theta': theta}
        
#         if self.state != "IDLE":
#             self.path_history.append({
#                 'time': self.get_clock().now().nanoseconds / 1e9,
#                 'x': x, 'y': y, 'theta': theta # 'x': x*3.125, 'y': y*3.125, 'theta': theta
#             })
        
    
#     def control_callback(self, msg):
#         """Recibe comandos de control"""
#         cmd = msg.data.strip().upper()
        
#         if cmd == "START" and self.state == "IDLE":
#             self.start_trajectory()
#         elif cmd == "STOP":
#             self.stop_trajectory()
#         elif cmd == "RESET":
#             self.reset_trajectory()
    
#     def start_trajectory(self):
#         """Inicia la trayectoria"""
#         if self.current_pose is None:
#             self.get_logger().warn("⚠️  No hay odometría disponible")
#             return
            
#         self.state = "FORWARD"
#         self.start_time = self.get_clock().now()
#         self.start_pose = self.current_pose.copy()
#         self.movement_start_pose = self.current_pose.copy()
#         self.movement_start_time = self.get_clock().now()
#         self.command_active = False
        
#         self.path_history = []
#         self.errors = []
        
#         self.get_logger().info("🚀 Iniciando trayectoria en L")
#         self.publish_status("STARTED")
    
#     def stop_trajectory(self):
#         """Detiene la trayectoria"""
#         self.send_motor_command("STOP:RELEASE")
#         self.state = "IDLE"
#         self.command_active = False
#         self.get_logger().info("⏹️  Trayectoria detenida")
#         self.publish_status("STOPPED")
    
#     def reset_trajectory(self):
#         """Reinicia todo"""
#         self.send_motor_command("STOP:RELEASE")
#         self.state = "IDLE"
#         self.start_time = None
#         self.start_pose = None
#         self.command_active = False
#         self.path_history = []
#         self.errors = []
#         self.get_logger().info("🔄 Trayectoria reiniciada")
#         self.publish_status("RESET")
    
#     def send_motor_command(self, command):
#         """Envía comando a los motores"""
#         msg = String()
#         msg.data = command
#         self.motor_pub.publish(msg)
#         self.get_logger().debug(f"📤 Motor: {command}")
    
#     def check_timeout(self, max_time):
#         """Verifica si se excedió el timeout"""
#         if self.movement_start_time is None:
#             return False
        
#         elapsed = (self.get_clock().now() - self.movement_start_time).nanoseconds / 1e9
#         if elapsed > max_time:
#             self.get_logger().error(f"⏱️  TIMEOUT! Movimiento excedió {max_time}s")
#             return True
#         return False
    
#     def control_loop(self):
#         """Bucle principal de control"""
#         if self.state == "IDLE" or self.current_pose is None:
#             return
            
#         # Máquina de estados
#         if self.state == "FORWARD":
#             self.execute_forward()
            
#         elif self.state == "ROTATING":
#             self.execute_rotation()
            
#         elif self.state == "SIDE":
#             self.execute_side()
            
#         elif self.state == "DONE":
#             if not self.command_active:
#                 self.send_motor_command("STOP:RELEASE")
#                 self.command_active = True
#                 self.analyze_and_plot()
#                 self.get_logger().info("✅ Trayectoria completada")
#                 self.publish_status("COMPLETED")
        
#         self.publish_error_data()
    
#     def execute_forward(self):
#         """Ejecuta movimiento frontal"""
#         if not self.command_active:
#             cmd = f"FORWARD:PRESS:{self.speed_percent}"
#             self.send_motor_command(cmd)
#             self.command_active = True
#             self.movement_start_pose = self.current_pose.copy()
#             self.movement_start_time = self.get_clock().now()
#             self.get_logger().info(f"▶️  Moviendo adelante {self.distance_forward*100:.0f}cm")
        
#         # Timeout
#         if self.check_timeout(self.FORWARD_TIMEOUT):
#             self.send_motor_command("FORWARD:RELEASE")
#             self.get_logger().error("❌ FORWARD timeout - abortando")
#             self.state = "DONE"
#             return
        
#         # Calcular distancia recorrida
#         dx = self.current_pose['x'] - self.movement_start_pose['x']
#         dy = self.current_pose['y'] - self.movement_start_pose['y']
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Log cada segundo
#         elapsed = (self.get_clock().now() - self.movement_start_time).nanoseconds / 1e9
#         if int(elapsed) % 1 == 0 and elapsed > 0.1:
#             self.get_logger().info(f"   Distancia: {distance*100:.1f}/{self.distance_forward*100:.0f}cm")
        
#         # Verificar si llegó
#         if distance >= self.distance_forward - self.pos_tolerance:
#             self.send_motor_command("FORWARD:RELEASE")
#             self.state = "ROTATING"
#             self.command_active = False
#             self.get_logger().info(f"✓ Adelante completado: {distance*100:.1f}cm")
    
#     def execute_rotation(self):
#         """Ejecuta rotación - VERSIÓN CORREGIDA"""
#         if not self.command_active:
#             cmd = f"CW:PRESS:{self.speed_percent_rotate}"
#             self.send_motor_command(cmd)
#             self.command_active = True
#             self.movement_start_pose = self.current_pose.copy()
#             self.movement_start_time = self.get_clock().now()
#             self.get_logger().info(f"↻ Rotando {math.degrees(self.rotation_angle):.0f}°")
        
#         # Timeout
#         if self.check_timeout(self.ROTATION_TIMEOUT):
#             self.send_motor_command("CW:RELEASE")
#             self.get_logger().error("❌ ROTATION timeout - abortando")
#             self.state = "DONE"
#             return
        
#         # Calcular ángulo girado (CORREGIDO)
#         current_angle = self.current_pose['theta']
#         start_angle = self.movement_start_pose['theta']
        
#         # Diferencia de ángulo
#         angle_diff = current_angle - start_angle
        
#         # Normalizar a [-π, π]
#         angle_diff = self.normalize_angle(angle_diff)
        
#         # Para giro horario (CW), el ángulo debe DISMINUIR (negativo)
#         # Queremos que gire -90° = -π/2
#         angle_rotated_abs = abs(angle_diff)
        
#         # Log cada segundo
#         elapsed = (self.get_clock().now() - self.movement_start_time).nanoseconds / 1e9
#         if int(elapsed * 10) % 5 == 0 and elapsed > 0.1:  # Log cada 0.5s
#             self.get_logger().info(
#                 f"   Ángulo girado: {math.degrees(angle_rotated_abs):.1f}°/{math.degrees(self.rotation_angle):.0f}° "
#                 f"(actual={math.degrees(current_angle):.1f}°, inicio={math.degrees(start_angle):.1f}°)"
#             )
        
#         # CONDICIÓN CORREGIDA: Si el ángulo absoluto girado >= objetivo - tolerancia
#         if angle_rotated_abs >= abs(self.rotation_angle) - self.angle_tolerance:
#             self.send_motor_command("CW:RELEASE")
#             self.state = "SIDE"
#             self.command_active = False
#             self.get_logger().info(f"✓ Rotación completada: {math.degrees(angle_rotated_abs):.1f}°")
    
#     def execute_side(self):
#         """Ejecuta movimiento lateral"""
#         if not self.command_active:
#             cmd = f"FORWARD:PRESS:{self.speed_percent}"
#             self.send_motor_command(cmd)
#             self.command_active = True
#             self.movement_start_pose = self.current_pose.copy()
#             self.movement_start_time = self.get_clock().now()
#             self.get_logger().info(f"⇨ Moviendo lateral {self.distance_side*100:.0f}cm")
        
#         # Timeout
#         if self.check_timeout(self.SIDE_TIMEOUT):
#             self.send_motor_command("RIGHT:RELEASE")
#             self.get_logger().error("❌ SIDE timeout - abortando")
#             self.state = "DONE"
#             return
        
#         # Calcular distancia lateral
#         dx = self.current_pose['x'] - self.movement_start_pose['x']
#         dy = self.current_pose['y'] - self.movement_start_pose['y']
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Log
#         elapsed = (self.get_clock().now() - self.movement_start_time).nanoseconds / 1e9
#         if int(elapsed) % 1 == 0 and elapsed > 0.1:
#             self.get_logger().info(f"   Distancia: {distance*100:.1f}/{self.distance_side*100:.0f}cm")
        
#         if distance >= self.distance_side - self.pos_tolerance:
#             self.send_motor_command("RIGHT:RELEASE")
#             self.state = "DONE"
#             self.command_active = False
            
#             if self.start_time:
#                 elapsed_total = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#                 self.get_logger().info(f"✓ Lateral completado en {elapsed_total:.2f}s")
    
#     def publish_status(self, status):
#         """Publica estado"""
#         msg = String()
#         msg.data = f"{status}|{self.state}|{len(self.path_history)}"
#         self.status_pub.publish(msg)
    
#     def publish_error_data(self):
#         """Publica datos de error"""
#         if len(self.path_history) < 2:
#             return
            
#         current = self.path_history[-1]
        
#         if self.state == "FORWARD":
#             target_x = self.start_pose['x'] + self.distance_forward#*3.125
#             target_y = self.start_pose['y']
#             target_theta = self.start_pose['theta']
#         elif self.state == "ROTATING":
#             target_x = self.start_pose['x'] + self.distance_forward#*3.125
#             target_y = self.start_pose['y']
#             target_theta = self.start_pose['theta'] - self.rotation_angle
#         elif self.state == "SIDE":
#             target_x = self.start_pose['x'] + self.distance_forward#*3.125
#             target_y = self.start_pose['y'] - self.distance_side#*3.125
#             target_theta = self.start_pose['theta'] - self.rotation_angle
#         else:
#             return
        
#         error_x = current['x'] - target_x
#         error_y = current['y'] - target_y
#         error_theta = self.normalize_angle(current['theta'] - target_theta)
        
#         self.errors.append({
#             'time': current['time'],
#             'error_x': error_x,
#             'error_y': error_y,
#             'error_theta': error_theta
#         })
        
#         msg = Float32MultiArray()
#         msg.data = [float(error_x), float(error_y), float(error_theta)]
#         self.error_pub.publish(msg)
    
#     def analyze_and_plot(self):
#         """Analiza resultados y genera gráficas"""
#         if len(self.path_history) < 2:
#             self.get_logger().warn("No hay datos para analizar")
#             return
            
#         self.get_logger().info("📊 Generando análisis...")
        
#         # Trayectoria real
#         real_x = [p['x'] for p in self.path_history]
#         real_y = [p['y'] for p in self.path_history]
        
#         # Trayectoria ideal (L)
#         ideal_x = [
#             self.start_pose['x'],
#             self.start_pose['x'] + self.distance_forward,#*3.125,
#             self.start_pose['x'] + self.distance_forward,#*3.125,
#             self.start_pose['x'] + self.distance_forward#*3.125
#         ]
#         ideal_y = [
#             self.start_pose['y'],
#             self.start_pose['y'],
#             self.start_pose['y'],
#             self.start_pose['y'] - self.distance_side#*3.125
#         ]
        
#         # Crear gráficas
#         fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
#         # 1. Trayectoria
#         ax1 = axes[0, 0]
#         ax1.plot(real_x, real_y, 'b-', label='Real', linewidth=2, alpha=0.7)
#         ax1.plot(ideal_x, ideal_y, 'r--', label='Ideal', linewidth=2)
#         ax1.scatter(ideal_x, ideal_y, c='red', s=100, marker='o', zorder=5)
#         ax1.set_xlabel('X (m)')
#         ax1.set_ylabel('Y (m)')
#         ax1.set_title('Trayectoria Real vs Ideal')
#         ax1.legend()
#         ax1.grid(True)
#         ax1.axis('equal')
        
#         # 2. Errores X, Y
#         ax2 = axes[0, 1]
#         if self.errors:
#             times = [e['time'] - self.errors[0]['time'] for e in self.errors]
#             error_x = [e['error_x'] * 100 for e in self.errors]
#             error_y = [e['error_y'] * 100 for e in self.errors]
            
#             ax2.plot(times, error_x, 'r-', label='Error X', linewidth=1.5)
#             ax2.plot(times, error_y, 'b-', label='Error Y', linewidth=1.5)
#             ax2.set_xlabel('Tiempo (s)')
#             ax2.set_ylabel('Error (cm)')
#             ax2.set_title('Errores de Posición')
#             ax2.legend()
#             ax2.grid(True)
        
#         # 3. Error angular
#         ax3 = axes[1, 0]
#         if self.errors:
#             error_theta = [math.degrees(abs(e['error_theta'])) for e in self.errors]
#             ax3.plot(times, error_theta, 'g-', linewidth=1.5)
#             ax3.set_xlabel('Tiempo (s)')
#             ax3.set_ylabel('Error Angular (°)')
#             ax3.set_title('Error de Orientación')
#             ax3.grid(True)
        
#         # 4. Histograma
#         ax4 = axes[1, 1]
#         if len(self.errors) > 10:
#             final_errors = []
#             for e in self.errors[-10:]:
#                 final_errors.append(math.sqrt(e['error_x']**2 + e['error_y']**2) * 100)
            
#             ax4.hist(final_errors, bins=10, alpha=0.7, color='purple', edgecolor='black')
#             ax4.set_xlabel('Error Final (cm)')
#             ax4.set_ylabel('Frecuencia')
#             ax4.set_title('Distribución de Errores Finales')
#             ax4.grid(True, alpha=0.3)
            
#             mean_err = np.mean(final_errors)
#             std_err = np.std(final_errors)
#             ax4.text(0.05, 0.95, f'Media: {mean_err:.2f} cm\nStd: {std_err:.2f} cm',
#                     transform=ax4.transAxes, verticalalignment='top',
#                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
#         plt.tight_layout()
        
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         filename = f"/home/orangepi/odometry_graf/analisis_L_{timestamp}.png"
#         plt.savefig(filename, dpi=150)
#         plt.close()
        
#         self.show_summary()
#         self.get_logger().info(f"📈 Análisis guardado: {filename}")
    
#     def show_summary(self):
#         """Muestra resumen de resultados"""
#         if len(self.path_history) < 2:
#             return
            
#         final_pose = self.path_history[-1]
#         start_pose = self.path_history[0]
        
#         total_distance = 0
#         for i in range(1, len(self.path_history)):
#             dx = self.path_history[i]['x'] - self.path_history[i-1]['x']
#             dy = self.path_history[i]['y'] - self.path_history[i-1]['y']
#             total_distance += math.sqrt(dx**2 + dy**2)
        
#         target_x = start_pose['x'] + self.distance_forward
#         target_y = start_pose['y'] - self.distance_side
#         target_theta = start_pose['theta'] - self.rotation_angle
        
#         final_error_x = final_pose['x'] - target_x
#         final_error_y = final_pose['y'] - target_y
#         final_error_theta = self.normalize_angle(final_pose['theta'] - target_theta)
#         final_distance_error = math.sqrt(final_error_x**2 + final_error_y**2)
        
#         if self.start_time:
#             elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
#         self.get_logger().info(f"""
#         📋 RESUMEN FINAL:
#         • Posición final: ({final_pose['x']:.3f}, {final_pose['y']:.3f})
#         • Orientación final: {math.degrees(final_pose['theta']):.1f}°
#         • Error posición: {final_distance_error*100:.2f} cm
#         • Error X: {final_error_x*100:.2f} cm
#         • Error Y: {final_error_y*100:.2f} cm
#         • Error ángulo: {math.degrees(abs(final_error_theta)):.2f}°
#         • Distancia recorrida: {total_distance:.3f} m
#         • Tiempo total: {elapsed:.2f} s
#         • Velocidad promedio: {total_distance/elapsed:.3f} m/s
#         """)
    
#     def destroy_node(self):
#         """Limpieza al cerrar"""
#         if self.state != "IDLE":
#             self.send_motor_command("STOP:RELEASE")
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = SimplePathFollower()
#         node.get_logger().info("✅ Seguidor simple listo. Enviar 'START' a /path_control")
#         rclpy.spin(node)
        
#     except KeyboardInterrupt:
#         if 'node' in locals():
#             node.analyze_and_plot()
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# !/usr/bin/env python3
# !/usr/bin/env python3
# !/usr/bin/env python3
"""
Path Follower Unificado con Análisis Completo de Errores
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
import math
import json
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class EnhancedPathFollower(Node):
    def __init__(self):
        super().__init__('enhanced_path_follower')
        
        # ========== PARÁMETROS ==========
        self.declare_parameter('mode', 'RRT')  # 'L' o 'RRT'
        self.declare_parameter('rrt_file', '/home/orangepi/map_test/ruta_grabada.json')
        self.declare_parameter('pos_tolerance', 0.05)    # 5 cm
        self.declare_parameter('angle_tolerance', 5.0)   # 5 grados
        
        self.mode = self.get_parameter('mode').value
        self.rrt_file = self.get_parameter('rrt_file').value
        self.pos_tolerance = self.get_parameter('pos_tolerance').value
        self.angle_tolerance = math.radians(self.get_parameter('angle_tolerance').value)
        
        # ========== ESTADO ==========
        self.state = "IDLE"
        self.current_pose = None
        self.instructions = []
        self.current_instruction_idx = 0
        self.path_history = []
        self.errors = []
        self.start_pose = None
        self.start_time = None
        
        # ========== CONFIGURACIÓN ==========
        if self.mode == 'RRT' and os.path.exists(self.rrt_file):
            self.load_rrt_instructions()
            if self.instructions:
                self.get_logger().info(f"✅ Modo RRT activado - {len(self.instructions)} instrucciones")
            else:
                self.setup_l_mode()
                self.get_logger().info("⚠️ Modo RRT falló, usando modo L")
        else:
            self.setup_l_mode()
            self.get_logger().info("✅ Modo L activado")
        
        # ========== SUSCRIPTORES Y PUBLICADORES ==========
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.control_sub = self.create_subscription(String, '/path_control', self.control_callback, 10)
        self.motor_pub = self.create_publisher(String, 'motor_command', 10)
        self.status_pub = self.create_publisher(String, '/path_status', 10)
        self.error_pub = self.create_publisher(Float32MultiArray, '/path_error', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"""
        🤖 Path Follower Mejorado inicializado
        Modo: {self.mode}
        Instrucciones: {len(self.instructions)}
        """)
    
    def load_rrt_instructions(self):
        """Carga instrucciones desde archivo RRT simple"""
        try:
            self.get_logger().info(f"📂 Cargando: {self.rrt_file}")
            
            with open(self.rrt_file, 'r') as f:
                data = json.load(f)
            
            self.instructions = []
            
            # Formato con campo 'instructions'
            if 'instructions' in data and data['instructions']:
                for instr in data['instructions']:
                    if instr['type'] == 'FORWARD':
                        self.instructions.append({
                            'type': 'FORWARD',
                            'distance': float(instr['distance']),
                            'speed': int(instr.get('speed', 80))
                        })
                    elif instr['type'] == 'ROTATE':
                        self.instructions.append({
                            'type': 'ROTATE',
                            'direction': instr.get('direction', 'CW'),
                            'angle': float(instr.get('angle', 0)),
                            'angle_deg': float(instr.get('angle_deg', math.degrees(float(instr.get('angle', 0))))),
                            'speed': int(instr.get('speed', 40))
                        })
            
            # Formato de ruta directa
            elif 'route' in data and len(data['route']) > 1:
                route = data['route']
                for i in range(len(route) - 1):
                    x1, y1 = route[i]
                    x2, y2 = route[i + 1]
                    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    
                    if distance > 0.01:
                        self.instructions.append({
                            'type': 'FORWARD',
                            'distance': distance,
                            'speed': 80
                        })
            
            self.get_logger().info(f"📊 {len(self.instructions)} instrucciones cargadas")
            
            if self.instructions:
                total_distance = sum(i['distance'] for i in self.instructions if i['type'] == 'FORWARD')
                rotations = sum(1 for i in self.instructions if i['type'] == 'ROTATE')
                self.get_logger().info(f"📏 Distancia total: {total_distance:.2f}m, Rotaciones: {rotations}")
            
            return len(self.instructions) > 0
            
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando RRT: {str(e)}")
            self.instructions = []
            return False
    
    def setup_l_mode(self):
        """Configura modo L tradicional"""
        self.instructions = [
            {'type': 'FORWARD', 'distance': 2.0, 'speed': 50},
            {'type': 'ROTATE', 'direction': 'CW', 'angle': math.radians(90), 'angle_deg': 90, 'speed': 40},
            {'type': 'FORWARD', 'distance': 0.6, 'speed': 50}
        ]
    
    def odom_callback(self, msg):
        """Actualiza odometría"""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            theta = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            
            self.current_pose = {'x': x, 'y': y, 'theta': theta}
            
            if self.state != "IDLE":
                current_time = self.get_clock().now().nanoseconds / 1e9
                self.path_history.append({
                    'time': current_time,
                    'x': x, 'y': y, 'theta': theta
                })
                
                # Calcular errores
                self.calculate_errors()
                
        except Exception as e:
            self.get_logger().error(f"Error en odometría: {e}")
    
    def calculate_errors(self):
        """Calcula errores de posición y orientación"""
        if not self.start_pose or not self.instructions:
            return
        
        current = self.path_history[-1]
        
        # Calcular posición objetivo basada en instrucciones completadas
        target_x, target_y, target_theta = self.calculate_target_pose()
        
        # Calcular errores
        error_x = current['x'] - target_x
        error_y = current['y'] - target_y
        error_theta = self.normalize_angle(current['theta'] - target_theta)
        
        # Almacenar errores
        self.errors.append({
            'time': current['time'],
            'error_x': error_x,
            'error_y': error_y,
            'error_theta': error_theta,
            'target_x': target_x,
            'target_y': target_y,
            'target_theta': target_theta
        })
        
        # Publicar errores
        self.publish_error_data(error_x, error_y, error_theta)
    
    def calculate_target_pose(self):
        """Calcula la pose objetivo actual basada en instrucciones completadas"""
        if not self.start_pose:
            return 0, 0, 0
        
        # Empezar desde la pose inicial
        x = self.start_pose['x']
        y = self.start_pose['y']
        theta = self.start_pose['theta']
        
        # Aplicar instrucciones completadas
        for i in range(min(self.current_instruction_idx, len(self.instructions))):
            instr = self.instructions[i]
            
            if instr['type'] == 'ROTATE':
                if instr['direction'] == 'CW':
                    theta -= instr['angle']
                else:  # CCW
                    theta += instr['angle']
                theta = self.normalize_angle(theta)
            
            elif instr['type'] == 'FORWARD':
                x += instr['distance'] * math.cos(theta)
                y += instr['distance'] * math.sin(theta)
        
        return x, y, theta
    
    def publish_error_data(self, error_x, error_y, error_theta):
        """Publica datos de error"""
        msg = Float32MultiArray()
        msg.data = [float(error_x), float(error_y), float(error_theta)]
        self.error_pub.publish(msg)
    
    def control_callback(self, msg):
        """Recibe comandos de control"""
        cmd = msg.data.strip().upper()
        
        if cmd == "START" and self.state == "IDLE":
            self.start_execution()
        elif cmd == "STOP":
            self.stop_execution()
        elif cmd == "RESET":
            self.reset_execution()
        elif cmd.startswith("LOAD:"):
            filename = cmd[5:]
            if os.path.exists(filename):
                self.rrt_file = filename
                if self.load_rrt_instructions():
                    self.get_logger().info(f"📂 Ruta RRT cargada: {filename}")
                else:
                    self.get_logger().warn(f"⚠️ No se pudieron cargar instrucciones de: {filename}")
    
    def start_execution(self):
        """Inicia la ejecución"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️ No hay odometría disponible")
            return
        
        if not self.instructions:
            self.get_logger().warn("⚠️ No hay instrucciones para ejecutar")
            return
        
        self.state = "EXECUTING"
        self.current_instruction_idx = 0
        self.start_pose = self.current_pose.copy()
        self.start_time = self.get_clock().now()
        self.path_history = []
        self.errors = []
        
        self.get_logger().info(f"🚀 Iniciando ejecución - {len(self.instructions)} instrucciones")
        self.publish_status("STARTED")
    
    def stop_execution(self):
        """Detiene la ejecución"""
        self.send_motor_command("STOP:RELEASE")
        self.state = "IDLE"
        self.get_logger().info("⏹️ Ejecución detenida")
        self.publish_status("STOPPED")
    
    def reset_execution(self):
        """Reinicia todo"""
        self.send_motor_command("STOP:RELEASE")
        self.state = "IDLE"
        self.current_instruction_idx = 0
        self.start_pose = None
        self.start_time = None
        self.path_history = []
        self.errors = []
        self.get_logger().info("🔄 Ejecución reiniciada")
        self.publish_status("RESET")
    
    def send_motor_command(self, command):
        """Envía comando a los motores"""
        msg = String()
        msg.data = command
        self.motor_pub.publish(msg)
    
    def normalize_angle(self, angle):
        """Normaliza ángulo a rango [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        """Bucle principal de control"""
        if self.state == "IDLE" or self.current_pose is None:
            return
        
        if self.state == "EXECUTING":
            self.execute_current_instruction()
        
        self.publish_status(f"EXECUTING|{self.state}|{self.current_instruction_idx}")
    
    def execute_current_instruction(self):
        """Ejecuta la instrucción actual"""
        if self.current_instruction_idx >= len(self.instructions):
            self.complete_execution()
            return
        
        instruction = self.instructions[self.current_instruction_idx]
        
        if instruction['type'] == 'FORWARD':
            self.execute_forward(instruction)
        elif instruction['type'] == 'ROTATE':
            self.execute_rotation(instruction)
    
    def execute_forward(self, instruction):
        """Ejecuta movimiento frontal"""
        # Iniciar movimiento si no se ha hecho
        if not hasattr(self, 'forward_start'):
            self.forward_start = {
                'pose': self.current_pose.copy(),
                'time': self.get_clock().now()
            }
            speed = instruction.get('speed', 80)
            self.send_motor_command(f"FORWARD:PRESS:{speed}")
            self.get_logger().info(f"▶️ Adelante: {instruction['distance']:.2f}m")
        
        # Calcular distancia recorrida
        dx = self.current_pose['x'] - self.forward_start['pose']['x']
        dy = self.current_pose['y'] - self.forward_start['pose']['y']
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        # Log periódico
        elapsed = (self.get_clock().now() - self.forward_start['time']).nanoseconds / 1e9
        if int(elapsed) % 1 == 0:  # Cada 1 segundo
            self.get_logger().info(f"   Distancia: {distance_traveled:.3f}/{instruction['distance']:.3f}m")
        
        # Verificar si completó
        if distance_traveled >= instruction['distance'] - self.pos_tolerance:
            self.send_motor_command("FORWARD:RELEASE")
            delattr(self, 'forward_start')
            self.current_instruction_idx += 1
            self.get_logger().info(f"✓ Adelante completado: {distance_traveled:.3f}m")
    
    def execute_rotation(self, instruction):
        """Ejecuta rotación"""
        # Iniciar rotación si no se ha hecho
        if not hasattr(self, 'rotation_start'):
            self.rotation_start = {
                'pose': self.current_pose.copy(),
                'time': self.get_clock().now()
            }
            direction = instruction.get('direction', 'CW')
            speed = instruction.get('speed', 40)
            self.send_motor_command(f"{direction}:PRESS:{speed}")
            
            angle_deg = instruction.get('angle_deg', math.degrees(instruction['angle']))
            self.get_logger().info(f"↻ Rotando {angle_deg:.1f}° {direction}")
        
        # Calcular ángulo girado
        current_angle = self.current_pose['theta']
        start_angle = self.rotation_start['pose']['theta']
        angle_diff = current_angle - start_angle
        angle_diff = self.normalize_angle(angle_diff)
        
        # Determinar ángulo girado absoluto
        angle_rotated = abs(angle_diff)
        
        # Log periódico
        elapsed = (self.get_clock().now() - self.rotation_start['time']).nanoseconds / 1e9
        if int(elapsed * 2) % 2 == 0:  # Cada 0.5 segundos
            self.get_logger().info(f"   Ángulo: {math.degrees(angle_rotated):.1f}°/{math.degrees(instruction['angle']):.1f}°")
        
        # Verificar si completó
        if angle_rotated >= instruction['angle'] - self.angle_tolerance:
            direction = instruction.get('direction', 'CW')
            self.send_motor_command(f"{direction}:RELEASE")
            delattr(self, 'rotation_start')
            self.current_instruction_idx += 1
            self.get_logger().info(f"✓ Rotación completada: {math.degrees(angle_rotated):.1f}°")
    
    def complete_execution(self):
        """Completa la ejecución"""
        self.send_motor_command("STOP:RELEASE")
        self.state = "COMPLETED"
        
        # Guardar análisis
        if len(self.path_history) > 1:
            self.save_complete_analysis()
        
        self.print_summary()
        self.get_logger().info("✅ Trayectoria completada")
        self.publish_status("COMPLETED")
    
    def publish_status(self, status):
        """Publica estado"""
        msg = String()
        msg.data = f"{status}|{self.state}|{self.current_instruction_idx}/{len(self.instructions)}"
        self.status_pub.publish(msg)
    
    def save_complete_analysis(self):
        """Guarda análisis completo con 4 gráficos"""
        try:
            if len(self.path_history) < 2:
                return
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Crear figura con 4 subgráficos (2x2)
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            
            # ===== SUBPLOT 1: Trayectoria real vs ideal =====
            ax1 = axes[0, 0]
            
            # Trayectoria real
            real_x = [p['x'] for p in self.path_history]
            real_y = [p['y'] for p in self.path_history]
            ax1.plot(real_x, real_y, 'b-', label='Real', linewidth=2, alpha=0.7)
            
            # Trayectoria ideal (reconstruida de instrucciones)
            ideal_x, ideal_y = self.reconstruct_ideal_path()
            ax1.plot(ideal_x, ideal_y, 'r--', label='Ideal', linewidth=2)
            ax1.scatter(ideal_x, ideal_y, c='red', s=50, marker='o', zorder=5)
            
            # Puntos importantes
            ax1.scatter(real_x[0], real_y[0], c='green', s=100, marker='s', label='Inicio Real', zorder=6)
            ax1.scatter(real_x[-1], real_y[-1], c='orange', s=100, marker='s', label='Fin Real', zorder=6)
            
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Trayectoria Real vs Ideal')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')
            
            # ===== SUBPLOT 2: Errores de posición X e Y =====
            ax2 = axes[0, 1]
            
            if self.errors:
                times = [e['time'] - self.errors[0]['time'] for e in self.errors]
                error_x = [e['error_x'] * 100 for e in self.errors]  # cm
                error_y = [e['error_y'] * 100 for e in self.errors]  # cm
                
                ax2.plot(times, error_x, 'r-', label='Error X', linewidth=1.5)
                ax2.plot(times, error_y, 'b-', label='Error Y', linewidth=1.5)
                ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                ax2.set_xlabel('Tiempo (s)')
                ax2.set_ylabel('Error (cm)')
                ax2.set_title('Errores de Posición')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
                
                # Añadir estadísticas
                mean_error_x = np.mean(error_x) if error_x else 0
                mean_error_y = np.mean(error_y) if error_y else 0
                std_error_x = np.std(error_x) if error_x else 0
                std_error_y = np.std(error_y) if error_y else 0
                
                stats_text = f'Media X: {mean_error_x:.2f} cm\n'
                stats_text += f'Media Y: {mean_error_y:.2f} cm\n'
                stats_text += f'Std X: {std_error_x:.2f} cm\n'
                stats_text += f'Std Y: {std_error_y:.2f} cm'
                
                ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
                        fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            # ===== SUBPLOT 3: Error de orientación =====
            ax3 = axes[1, 0]
            
            if self.errors:
                error_theta = [math.degrees(abs(e['error_theta'])) for e in self.errors]
                ax3.plot(times, error_theta, 'g-', linewidth=1.5)
                ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                ax3.set_xlabel('Tiempo (s)')
                ax3.set_ylabel('Error Angular (°)')
                ax3.set_title('Error de Orientación')
                ax3.grid(True, alpha=0.3)
                
                # Estadísticas de error angular
                if error_theta:
                    mean_error_theta = np.mean(error_theta)
                    max_error_theta = np.max(error_theta)
                    final_error_theta = error_theta[-1] if error_theta else 0
                    
                    stats_text = f'Media: {mean_error_theta:.1f}°\n'
                    stats_text += f'Máximo: {max_error_theta:.1f}°\n'
                    stats_text += f'Final: {final_error_theta:.1f}°'
                    
                    ax3.text(0.02, 0.98, stats_text, transform=ax3.transAxes,
                            fontsize=9, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            # ===== SUBPLOT 4: Histograma de errores finales =====
            ax4 = axes[1, 1]
            
            if len(self.errors) > 10:
                # Calcular error de posición total para los últimos puntos
                final_errors = []
                for e in self.errors[-10:]:
                    pos_error = math.sqrt(e['error_x']**2 + e['error_y']**2) * 100  # cm
                    final_errors.append(pos_error)
                
                if final_errors:
                    # Histograma
                    n, bins, patches = ax4.hist(final_errors, bins=10, alpha=0.7, 
                                               color='purple', edgecolor='black')
                    
                    # Línea de media
                    mean_error = np.mean(final_errors)
                    ax4.axvline(x=mean_error, color='red', linestyle='--', linewidth=2, 
                               label=f'Media: {mean_error:.2f} cm')
                    
                    ax4.set_xlabel('Error de Posición Final (cm)')
                    ax4.set_ylabel('Frecuencia')
                    ax4.set_title('Distribución de Errores Finales')
                    ax4.legend()
                    ax4.grid(True, alpha=0.3)
                    
                    # Estadísticas adicionales
                    std_error = np.std(final_errors)
                    ax4.text(0.02, 0.98, f'Std: {std_error:.2f} cm\nN: {len(final_errors)}',
                            transform=ax4.transAxes, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            else:
                ax4.text(0.5, 0.5, 'Datos insuficientes\npara histograma', 
                        ha='center', va='center', transform=ax4.transAxes,
                        fontsize=12, color='gray')
                ax4.set_title('Distribución de Errores Finales')
                ax4.grid(True, alpha=0.3)
            
            # Título general
            fig.suptitle(f'Análisis de Seguimiento - Modo {self.mode} - {timestamp}', 
                        fontsize=16, fontweight='bold')
            
            plt.tight_layout()
            
            # Guardar
            filename = f"/home/orangepi/odometry_graf/analisis_completo_{self.mode}_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close(fig)
            
            self.get_logger().info(f"📈 Análisis completo guardado: {filename}")
            
            # También guardar datos en JSON
            self.save_analysis_data(timestamp)
            
        except Exception as e:
            self.get_logger().error(f"Error guardando análisis completo: {e}")
    
    def reconstruct_ideal_path(self):
        """Reconstruye la trayectoria ideal a partir de instrucciones"""
        if not self.start_pose:
            return [], []
        
        x = [self.start_pose['x']]
        y = [self.start_pose['y']]
        theta = self.start_pose['theta']
        
        for instr in self.instructions:
            if instr['type'] == 'ROTATE':
                if instr['direction'] == 'CW':
                    theta -= instr['angle']
                else:
                    theta += instr['angle']
                theta = self.normalize_angle(theta)
            
            elif instr['type'] == 'FORWARD':
                new_x = x[-1] + instr['distance'] * math.cos(theta)
                new_y = y[-1] + instr['distance'] * math.sin(theta)
                x.append(new_x)
                y.append(new_y)
        
        return x, y
    
    def save_analysis_data(self, timestamp):
        """Guarda datos del análisis en JSON"""
        try:
            if not self.errors:
                return
            
            data = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'mode': self.mode,
                    'instructions': len(self.instructions),
                    'path_points': len(self.path_history),
                    'errors_count': len(self.errors)
                },
                'final_position': {
                    'x': self.path_history[-1]['x'] if self.path_history else 0,
                    'y': self.path_history[-1]['y'] if self.path_history else 0,
                    'theta': self.path_history[-1]['theta'] if self.path_history else 0
                },
                'errors_summary': {
                    'final_error_x': self.errors[-1]['error_x'] if self.errors else 0,
                    'final_error_y': self.errors[-1]['error_y'] if self.errors else 0,
                    'final_error_theta': self.errors[-1]['error_theta'] if self.errors else 0,
                    'final_position_error': math.sqrt(self.errors[-1]['error_x']**2 + self.errors[-1]['error_y']**2) 
                                         if self.errors else 0
                }
            }
            
            filename = f"/home/orangepi/odometry_graf/datos_analisis_{self.mode}_{timestamp}.json"
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f"💾 Datos de análisis guardados: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Error guardando datos: {e}")
    
    def print_summary(self):
        """Imprime resumen completo de la ejecución"""
        if not self.path_history or not self.errors:
            return
        
        final_pose = self.path_history[-1]
        first_pose = self.path_history[0]
        
        # Calcular distancia total recorrida
        total_distance = 0
        for i in range(1, len(self.path_history)):
            dx = self.path_history[i]['x'] - self.path_history[i-1]['x']
            dy = self.path_history[i]['y'] - self.path_history[i-1]['y']
            total_distance += math.sqrt(dx**2 + dy**2)
        
        # Calcular tiempo total
        if self.start_time:
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Calcular errores finales
        final_error_x = self.errors[-1]['error_x']
        final_error_y = self.errors[-1]['error_y']
        final_error_theta = self.errors[-1]['error_theta']
        final_position_error = math.sqrt(final_error_x**2 + final_error_y**2)
        
        # Calcular errores promedio
        avg_error_x = np.mean([abs(e['error_x']) for e in self.errors]) if self.errors else 0
        avg_error_y = np.mean([abs(e['error_y']) for e in self.errors]) if self.errors else 0
        avg_error_theta = np.mean([abs(e['error_theta']) for e in self.errors]) if self.errors else 0
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📊 RESUMEN COMPLETO DE EJECUCIÓN")
        self.get_logger().info("="*60)
        
        self.get_logger().info(f"\n📍 POSICIÓN FINAL:")
        self.get_logger().info(f"  X: {final_pose['x']:.3f} m")
        self.get_logger().info(f"  Y: {final_pose['y']:.3f} m")
        self.get_logger().info(f"  θ: {math.degrees(final_pose['theta']):.1f}°")
        
        self.get_logger().info(f"\n📏 DESPLAZAMIENTO:")
        self.get_logger().info(f"  Distancia total: {total_distance:.3f} m")
        self.get_logger().info(f"  Tiempo total: {elapsed_time:.2f} s")
        self.get_logger().info(f"  Velocidad promedio: {total_distance/elapsed_time:.3f} m/s" if elapsed_time > 0 else "  Velocidad: N/A")
        
        self.get_logger().info(f"\n❌ ERRORES FINALES:")
        self.get_logger().info(f"  Error posición: {final_position_error*100:.2f} cm")
        self.get_logger().info(f"  Error X: {final_error_x*100:.2f} cm")
        self.get_logger().info(f"  Error Y: {final_error_y*100:.2f} cm")
        self.get_logger().info(f"  Error θ: {math.degrees(abs(final_error_theta)):.2f}°")
        
        self.get_logger().info(f"\n📈 ERRORES PROMEDIO:")
        self.get_logger().info(f"  Error X promedio: {avg_error_x*100:.2f} cm")
        self.get_logger().info(f"  Error Y promedio: {avg_error_y*100:.2f} cm")
        self.get_logger().info(f"  Error θ promedio: {math.degrees(avg_error_theta):.2f}°")
        
        self.get_logger().info(f"\n📋 ESTADÍSTICAS:")
        self.get_logger().info(f"  Puntos de trayectoria: {len(self.path_history)}")
        self.get_logger().info(f"  Muestras de error: {len(self.errors)}")
        self.get_logger().info(f"  Instrucciones ejecutadas: {len(self.instructions)}")
        self.get_logger().info(f"  Modo: {self.mode}")
        
        self.get_logger().info("\n" + "="*60)
    
    def destroy_node(self):
        """Limpieza al cerrar"""
        if self.state != "IDLE":
            self.send_motor_command("STOP:RELEASE")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedPathFollower()
        node.get_logger().info("✅ Path Follower mejorado listo")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if 'node' in locals():
            node.get_logger().info("🛑 Interrupción recibida")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()