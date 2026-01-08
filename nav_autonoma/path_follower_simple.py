#!/usr/bin/env python3
"""
Path Follower Unificado con Análisis Completo de Errores
VERSIÓN CORREGIDA: Comando LOAD funcional
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
        self.declare_parameter('mode', 'RRT')
        self.declare_parameter('rrt_file', '/home/orangepi/map_test/ruta_grabada.json')
        self.declare_parameter('pos_tolerance', 0.02)
        self.declare_parameter('angle_tolerance', 2)
        
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
                            'speed': int(instr.get('speed', 50))
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
                            'speed': 50
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
                
                self.calculate_errors()
                
        except Exception as e:
            self.get_logger().error(f"Error en odometría: {e}")
    
    def calculate_errors(self):
        """Calcula errores de posición y orientación"""
        if not self.start_pose or not self.instructions:
            return
        
        current = self.path_history[-1]
        target_x, target_y, target_theta = self.calculate_target_pose()
        
        error_x = current['x'] - target_x
        error_y = current['y'] - target_y
        error_theta = self.normalize_angle(current['theta'] - target_theta)
        
        self.errors.append({
            'time': current['time'],
            'error_x': error_x,
            'error_y': error_y,
            'error_theta': error_theta,
            'target_x': target_x,
            'target_y': target_y,
            'target_theta': target_theta
        })
        
        self.publish_error_data(error_x, error_y, error_theta)
    
    def calculate_target_pose(self):
        """Calcula la pose objetivo actual basada en instrucciones completadas"""
        if not self.start_pose:
            return 0, 0, 0
        
        x = self.start_pose['x']
        y = self.start_pose['y']
        theta = self.start_pose['theta']
        
        # 1. Aplicar TODAS las instrucciones completadas
        for i in range(min(self.current_instruction_idx, len(self.instructions))):
            instr = self.instructions[i]
            
            if instr['type'] == 'ROTATE':
                if instr['direction'] == 'CW':
                    theta -= instr['angle']
                else:
                    theta += instr['angle']
                theta = self.normalize_angle(theta)
            
            elif instr['type'] == 'FORWARD':
                x += instr['distance'] * math.cos(theta)
                y += instr['distance'] * math.sin(theta)
        
        # 2. Agregar el objetivo de la instrucción ACTUAL en ejecución
        if self.current_instruction_idx < len(self.instructions):
            current_instr = self.instructions[self.current_instruction_idx]
            
            if current_instr['type'] == 'ROTATE':
                if current_instr['direction'] == 'CW':
                    theta -= current_instr['angle']
                else:
                    theta += current_instr['angle']
                theta = self.normalize_angle(theta)
            
            elif current_instr['type'] == 'FORWARD':
                x += current_instr['distance'] * math.cos(theta)
                y += current_instr['distance'] * math.sin(theta)
        
        return x, y, theta
    
    def publish_error_data(self, error_x, error_y, error_theta):
        """Publica datos de error"""
        msg = Float32MultiArray()
        msg.data = [float(error_x), float(error_y), float(error_theta)]
        self.error_pub.publish(msg)
    
    def control_callback(self, msg):
        """
        🔧 CORREGIDO: Recibe comandos de control con LOAD funcional
        """
        cmd = msg.data.strip().upper()
        
        if cmd == "START" and self.state == "IDLE":
            self.start_execution()
        
        elif cmd == "STOP":
            self.stop_execution()
        
        elif cmd == "RESET":
            self.reset_execution()
        
        # ⭐ SECCIÓN CORREGIDA: LOAD
        elif cmd.startswith("LOAD:"):
            filename = self.rrt_file
            self.handle_load_route(filename)
    
    def handle_load_route(self, filename):
        """
        🆕 NUEVA FUNCIÓN: Maneja correctamente el comando LOAD
        
        Pasos:
        1. Verifica que el archivo exista
        2. Carga las instrucciones
        3. Resetea el estado del robot
        4. Prepara todo para ejecutar
        """
        try:
            self.get_logger().info(f"📂 Procesando LOAD: {filename}")
            
            # 1. Verificar archivo
            if not os.path.exists(filename):
                self.get_logger().error(f"❌ Archivo no existe: {filename}")
                self.publish_status("ERROR:FILE_NOT_FOUND")
                return
            
            # 2. Guardar archivo actual y cargar nuevo
            self.rrt_file = filename
            
            # 3. Cargar instrucciones
            load_success = self.load_rrt_instructions()
            
            if not load_success or not self.instructions:
                self.get_logger().error(f"❌ No se pudieron cargar instrucciones de: {filename}")
                self.publish_status("ERROR:LOAD_FAILED")
                return
            
            # 4. ⭐ RESETEAR ESTADO (esto es lo que faltaba)
            self.get_logger().info("🔄 Reseteando estado del robot...")
            
            # Detener cualquier movimiento previo
            self.send_motor_command("STOP:RELEASE")
            
            # Resetear variables de ejecución
            self.state = "IDLE"
            self.current_instruction_idx = 0
            self.start_pose = None
            self.start_time = None
            self.path_history = []
            self.errors = []
            
            # Limpiar flags de ejecución si existen
            if hasattr(self, 'forward_start'):
                delattr(self, 'forward_start')
            if hasattr(self, 'rotation_start'):
                delattr(self, 'rotation_start')
            
            # 5. Confirmar carga exitosa
            self.get_logger().info(f"✅ Ruta cargada exitosamente: {filename}")
            self.get_logger().info(f"   📊 Instrucciones: {len(self.instructions)}")
            
            # Calcular y mostrar info de la ruta
            total_distance = sum(i['distance'] for i in self.instructions if i['type'] == 'FORWARD')
            rotations = sum(1 for i in self.instructions if i['type'] == 'ROTATE')
            
            self.get_logger().info(f"   📏 Distancia total: {total_distance:.2f}m")
            self.get_logger().info(f"   🔄 Rotaciones: {rotations}")
            self.get_logger().info(f"   ✅ Estado: LISTO para START")
            
            # 6. Publicar estado
            self.publish_status(f"LOADED:{len(self.instructions)}")
            
            # 7. OPCIONAL: Auto-start después de cargar
            # Descomenta estas líneas si quieres que inicie automáticamente
            # self.get_logger().info("⏰ Iniciando ejecución en 2 segundos...")
            # self.create_timer(2.0, self.auto_start_after_load, one_shot=True)
            
        except Exception as e:
            self.get_logger().error(f"❌ Excepción en LOAD: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.publish_status("ERROR:EXCEPTION")
    
    def auto_start_after_load(self):
        """🔧 OPCIONAL: Inicia automáticamente después de LOAD"""
        if self.state == "IDLE" and self.instructions:
            self.get_logger().info("🚀 Auto-iniciando ejecución...")
            self.start_execution()
    
    def start_execution(self):
        """Inicia la ejecución"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️ No hay odometría disponible")
            self.publish_status("ERROR:NO_ODOM")
            return
        
        if not self.instructions:
            self.get_logger().warn("⚠️ No hay instrucciones para ejecutar")
            self.publish_status("ERROR:NO_INSTRUCTIONS")
            return
        
        self.state = "EXECUTING"
        self.current_instruction_idx = 0
        self.start_pose = self.current_pose.copy()
        self.start_time = self.get_clock().now()
        self.path_history = []
        self.errors = []
        
        self.get_logger().info(f"🚀 Iniciando ejecución - {len(self.instructions)} instrucciones")
        self.get_logger().info(f"📍 Pose inicial: x={self.start_pose['x']:.3f}, y={self.start_pose['y']:.3f}, θ={math.degrees(self.start_pose['theta']):.1f}°")
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
        
        # Limpiar flags
        if hasattr(self, 'forward_start'):
            delattr(self, 'forward_start')
        if hasattr(self, 'rotation_start'):
            delattr(self, 'rotation_start')
        
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
        if not hasattr(self, 'forward_start'):
            self.forward_start = {
                'pose': self.current_pose.copy(),
                'time': self.get_clock().now()
            }
            speed = instruction.get('speed', 80)
            self.send_motor_command(f"FORWARD:PRESS:{speed}")
            self.get_logger().info(f"▶️ Adelante: {instruction['distance']:.2f}m @ {speed}%")
        
        dx = self.current_pose['x'] - self.forward_start['pose']['x']
        dy = self.current_pose['y'] - self.forward_start['pose']['y']
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        elapsed = (self.get_clock().now() - self.forward_start['time']).nanoseconds / 1e9
        if int(elapsed) % 1 == 0:
            self.get_logger().info(f"   📏 {distance_traveled:.3f}/{instruction['distance']:.3f}m")
        
        if distance_traveled >= instruction['distance'] - self.pos_tolerance:
            self.send_motor_command("FORWARD:RELEASE")
            delattr(self, 'forward_start')
            self.current_instruction_idx += 1
            self.get_logger().info(f"✓ Adelante completado: {distance_traveled:.3f}m")
    
    def execute_rotation(self, instruction):
        """Ejecuta rotación"""
        if not hasattr(self, 'rotation_start'):
            self.rotation_start = {
                'pose': self.current_pose.copy(),
                'time': self.get_clock().now()
            }
            direction = instruction.get('direction', 'CW')
            speed = instruction.get('speed', 40)
            self.send_motor_command(f"{direction}:PRESS:{speed}")
            
            angle_deg = instruction.get('angle_deg', math.degrees(instruction['angle']))
            self.get_logger().info(f"↻ Rotando {angle_deg:.1f}° {direction} @ {speed}%")
        
        current_angle = self.current_pose['theta']
        start_angle = self.rotation_start['pose']['theta']
        angle_diff = current_angle - start_angle
        angle_diff = self.normalize_angle(angle_diff)
        angle_rotated = abs(angle_diff)
        
        elapsed = (self.get_clock().now() - self.rotation_start['time']).nanoseconds / 1e9
        if int(elapsed * 2) % 2 == 0:
            self.get_logger().info(f"   📐 {math.degrees(angle_rotated):.1f}°/{math.degrees(instruction['angle']):.1f}°")
        
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
        """Guarda análisis completo"""
        try:
            if len(self.path_history) < 2:
                return
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            
            # Subplot 1: Trayectoria
            ax1 = axes[0, 0]
            real_x = [p['x'] for p in self.path_history]
            real_y = [p['y'] for p in self.path_history]
            ax1.plot(real_x, real_y, 'b-', label='Real', linewidth=2, alpha=0.7)
            
            ideal_x, ideal_y = self.reconstruct_ideal_path()
            ax1.plot(ideal_x, ideal_y, 'r--', label='Ideal', linewidth=2, alpha=0.6)
            ax1.scatter(ideal_x, ideal_y, c='red', s=80, marker='o', edgecolors='darkred', linewidths=2, zorder=5)
            
            if self.errors:
                target_x = [e['target_x'] for e in self.errors[::10]]
                target_y = [e['target_y'] for e in self.errors[::10]]
                ax1.scatter(target_x, target_y, c='orange', s=30, marker='x', alpha=0.5, label='Target')
            
            ax1.scatter(real_x[0], real_y[0], c='green', s=150, marker='s', label='Inicio', zorder=6)
            ax1.scatter(real_x[-1], real_y[-1], c='purple', s=150, marker='s', label='Fin', zorder=6)
            
            ax1.set_xlabel('X (m)', fontsize=11)
            ax1.set_ylabel('Y (m)', fontsize=11)
            ax1.set_title('Trayectoria Real vs Ideal', fontsize=12, fontweight='bold')
            ax1.legend(loc='best', fontsize=9)
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')
            
            # Subplot 2: Errores de posición
            ax2 = axes[0, 1]
            if self.errors:
                times = [(e['time'] - self.errors[0]['time']) for e in self.errors]
                error_x = [e['error_x'] * 100 for e in self.errors]
                error_y = [e['error_y'] * 100 for e in self.errors]
                error_pos = [math.sqrt(ex**2 + ey**2) for ex, ey in zip(error_x, error_y)]
                
                ax2.plot(times, error_pos, 'g-', label='Error Total', linewidth=2.5, alpha=0.8)
                ax2.plot(times, error_x, 'r-', label='Error X', linewidth=1.5, alpha=0.7)
                ax2.plot(times, error_y, 'b-', label='Error Y', linewidth=1.5, alpha=0.7)
                ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                ax2.axhline(y=self.pos_tolerance*100, color='orange', linestyle=':', alpha=0.5)
                
                ax2.set_xlabel('Tiempo (s)', fontsize=11)
                ax2.set_ylabel('Error (cm)', fontsize=11)
                ax2.set_title('Errores de Posición', fontsize=12, fontweight='bold')
                ax2.legend(loc='best', fontsize=9)
                ax2.grid(True, alpha=0.3)
            
            # Subplot 3: Error angular
            ax3 = axes[1, 0]
            if self.errors:
                error_theta = [math.degrees(e['error_theta']) for e in self.errors]
                ax3.plot(times, error_theta, 'g-', linewidth=2, alpha=0.7)
                ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                
                tol_deg = math.degrees(self.angle_tolerance)
                ax3.axhline(y=tol_deg, color='orange', linestyle=':', alpha=0.5)
                ax3.axhline(y=-tol_deg, color='orange', linestyle=':', alpha=0.5)
                
                ax3.set_xlabel('Tiempo (s)', fontsize=11)
                ax3.set_ylabel('Error Angular (°)', fontsize=11)
                ax3.set_title('Error de Orientación', fontsize=12, fontweight='bold')
                ax3.grid(True, alpha=0.3)
            
            # Subplot 4: Convergencia
            ax4 = axes[1, 1]
            if len(self.errors) > 20:
                window_size = max(10, len(self.errors) // 20)
                error_pos_total = [math.sqrt(e['error_x']**2 + e['error_y']**2) * 100 for e in self.errors]
                
                moving_avg = []
                for i in range(len(error_pos_total)):
                    start_idx = max(0, i - window_size//2)
                    end_idx = min(len(error_pos_total), i + window_size//2)
                    moving_avg.append(np.mean(error_pos_total[start_idx:end_idx]))
                
                ax4.plot(times, error_pos_total, 'gray', alpha=0.3, linewidth=0.5)
                ax4.plot(times, moving_avg, 'purple', linewidth=2.5, label=f'Media móvil')
                
                ax4.set_xlabel('Tiempo (s)', fontsize=11)
                ax4.set_ylabel('Error Total (cm)', fontsize=11)
                ax4.set_title('Convergencia del Error', fontsize=12, fontweight='bold')
                ax4.legend(loc='best', fontsize=9)
                ax4.grid(True, alpha=0.3)
            
            total_distance = sum(i['distance'] for i in self.instructions if i['type'] == 'FORWARD')
            fig.suptitle(f'Análisis Completo - Modo {self.mode}\nDistancia: {total_distance:.2f}m | {timestamp}', 
                        fontsize=14, fontweight='bold')
            
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            
            filename = f"/home/orangepi/odometry_graf/analisis_completo_{self.mode}_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close(fig)
            
            self.get_logger().info(f"📈 Análisis guardado: {filename}")
            self.save_analysis_data(timestamp)
            
        except Exception as e:
            self.get_logger().error(f"Error guardando análisis: {e}")
    
    def reconstruct_ideal_path(self):
        """Reconstruye trayectoria ideal"""
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
        """Guarda datos en JSON"""
        try:
            if not self.errors:
                return
            
            data = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'mode': self.mode,
                    'instructions': len(self.instructions),
                    'path_points': len(self.path_history)
                },
                'final_position': {
                    'x': self.path_history[-1]['x'] if self.path_history else 0,
                    'y': self.path_history[-1]['y'] if self.path_history else 0,
                    'theta': self.path_history[-1]['theta'] if self.path_history else 0
                },
                'errors_summary': {
                    'final_error_x': self.errors[-1]['error_x'] if self.errors else 0,
                    'final_error_y': self.errors[-1]['error_y'] if self.errors else 0,
                    'final_error_theta': self.errors[-1]['error_theta'] if self.errors else 0
                }
            }
            
            filename = f"/home/orangepi/odometry_graf/datos_{self.mode}_{timestamp}.json"
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
        except Exception as e:
            self.get_logger().error(f"Error guardando datos: {e}")
    
    def print_summary(self):
        """Imprime resumen"""
        if not self.path_history or not self.errors:
            return
        
        final_pose = self.path_history[-1]
        total_distance = sum(math.sqrt(
            (self.path_history[i]['x'] - self.path_history[i-1]['x'])**2 +
            (self.path_history[i]['y'] - self.path_history[i-1]['y'])**2
        ) for i in range(1, len(self.path_history)))
        
        if self.start_time:
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        else:
            elapsed_time = 0
        
        final_error = math.sqrt(self.errors[-1]['error_x']**2 + self.errors[-1]['error_y']**2)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📊 RESUMEN DE EJECUCIÓN")
        self.get_logger().info("="*60)
        self.get_logger().info(f"📍 Posición final: ({final_pose['x']:.3f}, {final_pose['y']:.3f})")
        self.get_logger().info(f"📏 Distancia: {total_distance:.3f}m")
        self.get_logger().info(f"⏱️ Tiempo: {elapsed_time:.2f}s")
        self.get_logger().info(f"❌ Error final: {final_error*100:.2f}cm")
        self.get_logger().info("="*60 + "\n")
    
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